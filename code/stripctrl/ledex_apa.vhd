-----------------------------------------------------------------------------------------------------------
--
-- Copyright 2018-2020 - Pixblasters.com
-- All rights reserved - Sva prava pridržana  
--
-----------------------------------------------------------------------------------------------------------
--
-- This file is a part of the Pixblasters_Light Demo adjusted for APA102 LED strips.

-- Pixblasters_Light Demo is free software: you can redistribute it and/or
-- modify it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
-- Pixblasters_Light Demo is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.

-- You should have received a copy of the GNU General Public License
-- along with Pixblasters_Light Demo.  If not, see <https://www.gnu.org/licenses/>.
--
-----------------------------------------------------------------------------------------------------------
--  Version   Date          Description
--
--  v1.00    19.07.2020     First version made from ledex.vhd (original MicroDemo) and adapted for the 
--                          APA102 LED strips - WS2812 component removed
-----------------------------------------------------------------------------------------------------------
-- Description:
--
-- The LEDEX block reads the stored lines of pixels and serializes them. The serialized data
-- is further coded into SPI compatible interface.
--    - strip_clk setup to 48 MHz on the original Pixblasters MS1 controller board
--    - SPI clock speeds are adjustable by DIVIDE_CLK: 24, 12, 6, 3 MHz
--    - the selected SPI clock speeds assure 60 Hz vertical refresh rate (with some reserve)
--    - data sampling on the rising LED (SPI) clock
--    - while inactive, LED_OUT and LED_CLK lines forced to 00
--
-----------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------
--                               LIBRARIES
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

library std;
use std.textio.all;

library unisim;
use unisim.vcomponents.all;

-------------------------------------------------------------------------------
--                                ENTITY
-------------------------------------------------------------------------------

entity ledex_apa is
	   generic (
	      TRAILER     : integer := 31;  -- defines the length of the End Frame (TRAILER * 32 clocks)
	      BRIGTH      : integer := 31;  -- global brightness level; 31 is the brightest!
		  DIVIDE_CLK  : integer :=  1;  -- sets up LED_CLK. Valid values 0-3; Default value 1 => 6MHz SPI intf.
	      LINE_LENGTH : integer := 512  -- defines the length of the display line in pixels - max 512
            );
   port (
      strip_clk             : in  std_logic;
      rgb_clk               : in  std_logic; -- for internal synchronizations on the input video clock
      -- Control Inputs

      video_rst_n            : in  std_logic;
      rgb_vs                 : in  std_logic;
      vs_fedge               : in  std_logic;
	  
      -- Control Outputs

      out_ready              : out std_logic;
      fetch_address          : out std_logic_vector(8 downto 0) := (others => '0'); -- counter fixed to 512 pixels

      -- Data inputs

      out_pixels             : in  std_logic_vector((16 * 24) - 1 downto 0);

      -- Coded LED Outputs
      led_out                : out std_logic_vector(15 downto 0);
	  led_clk                : out std_logic_vector(15 downto 0)

   );
end ledex_apa;

architecture rtl of ledex_apa is

   -----------------------------------------------------------------------------------------------------------
   -- TYPE DECLARATIONS
   -----------------------------------------------------------------------------------------------------------

   type STRIP_DATA is array (15 downto 0) of std_logic_vector(31 downto 0);
   type state_type is (IDLE, START_FRAME, PAYLOAD, END_FRAME);

   -----------------------------------------------------------------------------------------------------------
   -- COMPONENTS
   -----------------------------------------------------------------------------------------------------------

   ----------------------------------------------------------------------------
   -- SIGNALS
   ----------------------------------------------------------------------------

   -- cropping values change during the blanking interval; synchronized on the vertical refresh sync

   signal hor_res_hold      : std_logic_vector(9 downto 0) := (others => '0');
   signal prescal_hold      : std_logic_vector(3 downto 0) := (others => '0');
   signal strip_pixel       : STRIP_DATA := (others => (others => '0'));
   signal strip_prepare     : STRIP_DATA := (others => (others => '0'));
   signal adresa_b          : std_logic_vector(8 downto 0) := (others => '0'); -- counter fixed to 512 pixels

   -- Pixel outputs preparations

   signal new_pixel         : std_logic;
   signal new_pixel_d       : std_logic;
   signal new_pixel_2d      : std_logic;                           
   signal pixel_len_cnt     : std_logic_vector (4 downto 0) := (others => '1');
   signal new_strip_cycle   : std_logic := '0'; -- start new shift cycle at led                           
   signal set_shift_flag    : std_logic := '0'; -- condition that sets the flag active during active pixel shifting                                               -- downstream the strip
   signal shift_flag        : std_logic := '0'; -- flag active during active pixels shift
   signal shift_flag_pipe   : std_logic := '0'; -- flag active during active pixels shift
   signal end_shift         : std_logic := '0'; -- end of led values shifting; now limited by generic                           
   signal out_pixel         : std_logic_vector(15 downto 0);

   -- Fake clocking

   signal bit_slot           : std_logic := '0'; -- period of one serial bit at the strip                            
   signal bit_slot_d         : std_logic := '0'; -- period of one serial bit at the strip                            
   signal oversample_cnt     : std_logic_vector(4 downto 0) := (others => '1');                             
   signal rgb_vs_d           : std_logic;
   signal rgb_vs_2d          : std_logic;
   signal rgb_vs_3d          : std_logic;
   signal rgb_vs_4d          : std_logic;                                                   
   signal bit_slot_pre       : std_logic;
   signal bit_slot_pre_d     : std_logic;
   signal new_pixel_pre      : std_logic;
   signal end_pixel          : std_logic_vector(hor_res_hold'range);
   signal set_start_address  : std_logic_vector(8 downto 0) :=(others => '0');
   signal set_target_address : std_logic_vector(8 downto 0) :=(others => '0'); 
   signal last_shift         : std_logic := '0'; -- active while shifting the last pixel in the row  
   signal out_ready_i        : std_logic := '0'; -- blocks video input and causes frame slips
   signal end_vs             : std_logic := '0'; -- strip_clk synchronized end of the vertical sync
   signal rst_adresa_b       : std_logic := '0';
   signal rgb_vs_gamma       : std_logic := '0';  
   signal kloker             : std_logic := '0';
   signal kloker_d           : std_logic := '0';

   signal led_clk_i          : std_logic := '0';
   signal led_clk_bus        : std_logic_vector(led_clk'range) := (others => '0');
   signal led_clk_bus_i      : std_logic_vector(led_clk'range) := (others => '0');
   signal led_clk_internal   : std_logic_vector(led_clk'range) := (others => '0');
   signal led_source         : std_logic := '0';  
   signal free_runner        : std_logic_vector(3 downto 0) := (others => '0');
   signal free_runner_d      : std_logic_vector(3 downto 0) := (others => '0');
   signal runner_edge        : std_logic_vector(3 downto 0) := (others => '0');                             
   signal brightness         : std_logic_vector(4 downto 0) := (others => '0');                             
   signal current_state      : state_type;
   signal next_state         : state_type;                             
   signal starter            : std_logic := '0';
   signal starter_pipe       : std_logic := '0';
   signal ender              : std_logic := '0';
   signal ender_pipe         : std_logic := '0';
   signal active_line_out    : std_logic := '0';
   signal trailer_tc         : std_logic := '0';
   signal trailer_count      : std_logic_vector(4 downto 0) := (others => '0');
   signal led_out_i          : std_logic_vector(15 downto 0) := (others => '0');
   
begin

   hor_res_hold <= conv_std_logic_vector(LINE_LENGTH, 10);
   prescal_hold <= conv_std_logic_vector(DIVIDE_CLK, 4);
   brightness   <= conv_std_logic_vector(BRIGTH, 5);
     
   ------------------------------------------------------
   --                                                  --
   --    Bit Time Slot Clocking                        --
   --                                                  --
   ------------------------------------------------------
 
   -- LED clock signal is generated (selected) from outputs of this free-running counter
   
   runner: process(strip_clk)
   begin
	if rising_edge(strip_clk) then
		free_runner    <= free_runner + 1;
        free_runner_d  <= free_runner;
	end if;
   end process runner;
   
   -- edge for bit slot definition
   --runner_edge <= free_runner and (not free_runner_d); -- sets the strip_clk falling edge in the middle of data
   runner_edge <= not free_runner and free_runner_d; -- sets the strip_clk rising edge in the middle of data
 
   select_speed: process(prescal_hold, runner_edge, free_runner)
   begin
		case prescal_hold is
		
			when "0000" => 
				           bit_slot_pre <= runner_edge(0);     -- defines the LED data bit time
				           kloker       <= not free_runner(0); -- defines the LED clock
			when "0001" => 
		              	   bit_slot_pre <= runner_edge(1);
		              	   kloker       <= free_runner(1);
			when "0010" =>
				           bit_slot_pre <= runner_edge(2);
				           kloker       <= free_runner(2);				
			when "0011" => 
				           bit_slot_pre <= runner_edge(3);
				           kloker       <= free_runner(3);				
			when others => 
				           bit_slot_pre <= '0';
				           kloker       <= '0';
				
		end case;
   end process select_speed;
   
   ------------------------------------------------------
   --                                                  --
   --    Pixel length definition                       --
   --    32-bit per pixel payload                      --
   ------------------------------------------------------

   -- This clocking also enabled prior to the display enable

   pixel_measure: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
         if (new_pixel = '1') then
            pixel_len_cnt <= (others => '0');
         elsif (bit_slot_pre_d = '1') then
            pixel_len_cnt <= pixel_len_cnt + 1;
         end if;
      end if;
   end process pixel_measure;

   new_pixel_pre <= '1' when (bit_slot_pre_d = '1' and pixel_len_cnt = 31) else '0';

   sampler: process(strip_clk) -- tune up the pipelined logic
   begin
		if rising_edge(strip_clk) then
			bit_slot_pre_d  <= bit_slot_pre;
			bit_slot        <= bit_slot_pre_d;
			bit_slot_d      <= bit_slot;
			new_pixel       <= new_pixel_pre;
			new_pixel_d     <= new_pixel;
			new_pixel_2d    <= new_pixel_d;
			kloker_d        <= kloker;
			led_clk_i       <= kloker_d;
		end if;
   end process sampler;

   ------------------------------------------------------
   --                                                  --
   --    Main State Machine                            --
   --    Start Frame, Payload, End Frame               --
   ------------------------------------------------------
   
   -- LED state machine
   fsm_sequential: process (strip_clk)
      begin
        if rising_edge(strip_clk) then
			if (video_rst_n = '0' or end_vs = '1') then
				current_state <= IDLE;
			else
				current_state <= next_state;
			end if;
         end if;
      end process;

   led_fsm: process(current_state, set_shift_flag, new_pixel, end_shift, trailer_tc)
      begin
         case current_state is
			when IDLE =>
				if(set_shift_flag = '1' and new_pixel = '1') then
					next_state  <= START_FRAME;
				else
					next_state  <= IDLE;
				end if;
            when START_FRAME =>
				if(new_pixel = '1') then
					next_state  <= PAYLOAD; -- one pixel (32-bit) Start Frame, leading 0s
				else
					next_state  <= START_FRAME;
				end if;
            when PAYLOAD =>
				if(end_shift = '1') then
					next_state  <= END_FRAME; -- up to 512 RGB pixels
				else
					next_state  <= PAYLOAD;
				end if;
            when END_FRAME =>
				if(trailer_tc = '1' and new_pixel = '1') then
					next_state  <= IDLE; -- trailing 1s to propagate the whole pixels line
				else
					next_state  <= END_FRAME;
				end if;
            when others =>
               next_state <= IDLE;
         end case;
      end process;

   ------------------------------------------------------
   --                                                  --
   --    Starting up new shifting time                 --
   --                                                  --
   ------------------------------------------------------

   gamma_delay: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
         rgb_vs_gamma    <= rgb_vs;  
      end if;                        
   end process gamma_delay;
   
   vsync_edger: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
         rgb_vs_d    <= rgb_vs_gamma;
         rgb_vs_2d   <= rgb_vs_d;
         rgb_vs_3d   <= rgb_vs_2d;
         rgb_vs_4d   <= rgb_vs_3d;
      end if;
   end process vsync_edger;

   end_vs <= not rgb_vs_3d and rgb_vs_4d;

   --------------------------------------------------------------------------------------
   --                                                                                  --
   -- Due to different clocking domains this is not-fixed in the time                  --
   -- it waits on the vsync, and sets up the shift_flag. The setup                     --
   -- flag switches off the set signal                                                 --
   -- the flag is not set while the output still shifts the previous row (slow LEDs)   --
   --                                                                                  --
   --------------------------------------------------------------------------------------
   
   set_zastavica: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
		 if (video_rst_n = '0' or rgb_vs_gamma = '1' or shift_flag = '1') then --gg
            set_shift_flag <= '0';
         else
            set_shift_flag <= (set_shift_flag or end_vs); --rgb_vs_3d and rgb_vs_4d));
         end if;
      end if;
   end process set_zastavica;

   shift_traka: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
         if (video_rst_n = '0' or end_shift = '1' or (rgb_vs_gamma = '1' and out_ready_i = '1')) then --gg
            shift_flag <= '0';
			starter    <= '0';
         elsif (new_pixel = '1') then
            shift_flag <= shift_flag or set_shift_flag;
			starter    <= set_shift_flag;
	     end if;
      end if;
   end process shift_traka;
   
   -- Marks shifting of the last pixel in the row and prevents the controller to stop too early
   
   endermon: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
	    if (video_rst_n = '0' or end_shift = '1') then
		    last_shift <= '0';
		elsif (shift_flag = '1' and adresa_b = set_target_address) then
			last_shift <= new_pixel or last_shift;
		end if;
	  end if;
   end process endermon;
   
   end_shift <= '1' when (last_shift = '1' and new_pixel = '1') else '0';
 
 -- sets the beginning of the trailing 1s - End of Frame

   trailer_counter: process(strip_clk)
   begin
	if rising_edge(strip_clk) then
        if (video_rst_n = '0' or current_state = IDLE) then
		    trailer_count <= (others => '0');
		elsif (current_state = END_FRAME and new_pixel = '1') then
			trailer_count <= trailer_count + 1;
		end if;
	end if;
   end process trailer_counter;
   
   trailer_tc <= '1' when (trailer_count = conv_std_logic_vector(TRAILER, 5) and new_pixel = '1') else '0';

   -- Active flag during the END_FRAME time
   
   endering: process(strip_clk)
   begin
	if rising_edge(strip_clk) then
	    if (video_rst_n = '0' or trailer_tc = '1') then
		    ender <= '0';
		elsif (last_shift = '1' and new_pixel = '1') then
			ender <= '1';
		end if;	
	end if;
   end process endering;
   
   ------------------------------------------------------
   --                                                  --
   --    Address for fetching stored pixels in         --
   --    line BRAM-implemented dual-FIFO               --
   --                                                  --
   ------------------------------------------------------
      
   end_pixel <= hor_res_hold - 1;
     
   set_start_address  <= (others => '0');
   set_target_address <= end_pixel(set_target_address'range);
 
   rst_adresa_b <= not video_rst_n or (out_ready_i and (rgb_vs_gamma or set_shift_flag));
 
   adr_a_port: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
		 if (video_rst_n = '0' or rst_adresa_b = '1') then 
            adresa_b <= set_start_address;
         elsif ((starter = '0' and shift_flag = '1') and new_pixel_2d = '1') then
            adresa_b <= adresa_b + 1;
         end if;
      end if;
   end process adr_a_port;

   fetch_address <= adresa_b;

   ------------------------------------------------------------
   -- Output slower than the input causes frame drops at     --
   -- the input. out_ready_i blocks new writes and prevents  --
   -- premature stop of the output shifting                  --
   ------------------------------------------------------------
   
   -- now setup by trailer_tc at the end of the trailing 0s
   -- for ws2812 LED drivers, it was setup by shift's end
   
   set_out_ready_i: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
		 if (video_rst_n = '0') then
			out_ready_i <= '1';
		 else
            out_ready_i <= (trailer_tc or out_ready_i) and not set_shift_flag; 
		 end if;
      end if;
   end process set_out_ready_i;
   
   out_ready <= out_ready_i;

   ------------------------------------------------------------
   -- DoubleBuffer Field - take pixel from the BRAM and      --
   -- prepare it for parallel shifting through STRIPS number --
   -- of LED strip driving channels                          --
   ------------------------------------------------------------ž

   -- the MSB goes out as the FIRST bit!

   zadrzivaci: for i in 0 to 15 generate
   
      prepare_data: process(current_state, strip_prepare, brightness, out_pixels)
      begin
   		case current_state is
   			--when START_FRAME => strip_prepare(i) <= "10101010101010101010101010101010"; -- for simulation, check frame beginning
			when START_FRAME => strip_prepare(i) <= (others => '0');
  			when PAYLOAD     => strip_prepare(i) <= "111" & brightness & out_pixels(((i * 24) + 23) downto (i * 24));						
			when END_FRAME   => strip_prepare(i) <= (others => '1');
   			when others      => strip_prepare(i) <= (others => '0');  
           end case;			
      end process prepare_data;
      
      drzi_pixel_even: process(strip_clk)
      begin
         if rising_edge(strip_clk) then
   		 if (video_rst_n = '0' or rgb_vs_gamma = '1') then
               strip_pixel(i) <= (others => '0');
            elsif (new_pixel_d = '1') then
               strip_pixel(i) <= strip_prepare(i);
            elsif (bit_slot_d = '1') then
   			   strip_pixel(i) <= strip_pixel(i)(30 downto 0) & strip_pixel(i)(31);
            end if;
         end if;
      end process drzi_pixel_even;
   
      out_pixel(i)       <= strip_pixel(i)(31); 
   
   end generate zadrzivaci;
   
   -- Clock for LED strips internally multiplied 
 
   led_clocking: process(strip_clk)
   begin
      if rising_edge(strip_clk) then
		for i in 0 to led_clk'left loop
			led_clk_bus_i(i) <= led_clk_i;
			led_clk_bus(i)   <= led_clk_bus_i(i);
		end loop;
      end if;
   end process led_clocking;

   -- Clock for LED strips internally multiplied 
 
  gen_inv: if DIVIDE_CLK = 0 generate
  
   next2final_step: process(active_line_out, out_pixel, led_clk_bus)
   begin
  
  		if (active_line_out = '0') then	
  			led_out_i        <= (others => '0');
  			led_clk_internal <= (others => '0');
  		else
  			led_out_i        <= out_pixel;
  			led_clk_internal <= not led_clk_bus;
  	    end if;
  
   end process next2final_step;
   
  end generate gen_inv;
  
   gen_noinv: if DIVIDE_CLK /= 0 generate
  
   next2final_step: process(active_line_out, out_pixel, led_clk_bus)
   begin
  
  		if (active_line_out = '0') then	
  			led_out_i        <= (others => '0');
  			led_clk_internal <= (others => '0');
  		else
  			led_out_i        <= out_pixel;
  			led_clk_internal <= led_clk_bus;
  	    end if;
  
   end process next2final_step;
   
  end generate gen_noinv;
  
  final_step: process(strip_clk)
  begin
     if rising_edge(strip_clk) then
			led_out <= led_out_i;
			led_clk <= led_clk_internal; -- to assure rising edge sampling
	  end if;
  end process final_step;
   
   aligner: process(strip_clk)
   begin
	  if rising_edge(strip_clk) then
		if (new_pixel_d = '1') then	
			starter_pipe    <= starter;
			shift_flag_pipe <= shift_flag;
			ender_pipe      <= ender;
		end if;
	  end if;
   end process aligner;
   
   active_line_out <= starter_pipe or shift_flag_pipe or ender_pipe; -- flag active during the whole line shift-out
 
end rtl;



