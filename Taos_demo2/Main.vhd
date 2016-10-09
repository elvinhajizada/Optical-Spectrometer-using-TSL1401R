----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:25:58 06/13/2016 
-- Design Name: 
-- Module Name:    Main - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
use work.myTypes.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Main is
		
		generic( pow				: integer := 16; 		 
				inc				: integer := 151;  -- For baud tick generation (for 115.2K 	pow=16 inc=151)
				nofTest			: integer := 12;   -- Number of 21 samples will be transmitted in the testVector
				nofData			: integer := 128;  -- PLEASE MODIFY this number accoording to the number of samples you will transmit
				MCLK_CYCLE		: integer := 3124; -- used to generate 8Khz clk
				SCLK_CYCLE		: integer := 1; 	 -- used in generating sclk
				WAIT_CYCLE		: integer := 6);	 -- for 115.2K WAIT_CYCLE=6  
		
		port(   clk     		 	: in std_logic;   -- 100MHz clock
            ADC_CS   			: out std_logic;  -- ADC chip select
            ADC_SCLK 			: out std_logic;  -- ADC serial clock
            ADC_D0   			: in std_logic;   -- ADC Channel 0
            led      			: out std_logic_vector(7 downto 0);
				tx_start				: in		std_logic;  -- transmit (tx) enable pin			
				txd					: out	std_logic; -- serial out pin
				SI 					: out  STD_LOGIC := '0';
            taos_clk 			: out  STD_LOGIC;
				modelock				: out std_logic
            );
		
end Main;

architecture Behavioral of Main is
	

-- SIGNALS
signal	flag		:	std_logic:='0';	-- control signal
signal	acc		:	std_logic_vector((pow) downto 0):=(others => '0');

-- CLK
signal 	mclk		:	std_logic:='0';
signal	clk_baud	:	std_logic:='0';

-- TX OPERATION SIGNALS
signal buff_in		: 	std_logic_vector (7 downto 0);
signal tmp_txd		:	std_logic:='1';
type state_type is (START,s1,s2,s3,s4,s5,s6,s7,s8,STOP);  --type of state machine.
signal state: state_type := START;  --current and next state declaration.


signal index : integer range 0 to 128 := 0;
signal datain : pixelArray;
signal sample_enable : std_logic := '1';
signal finished : std_logic := '0';	
signal tx_enable: std_logic := '0';	

--modelock--------------------------------
signal mode_lock_pixel : integer range 0 to 127 := 0;
signal treshold : std_logic_vector(7 downto 0) := "00010100";
signal mode_lock : std_logic := '0';
signal total : integer range 0 to 32640:= 0;
------------------------------------------

signal count: integer range 0 to 1000000 := 0;
signal count2, count3 : integer range 0 to 255 := 0;
signal clk1MHz : std_logic := '0' ;

----------------------------------------------------------------------------------------------------------------
------------------------ADC_CODE--------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------


signal data_0 : std_logic_vector(11 downto 0) := (others=>'0');
    
    -----------------------------------------------------------------------------
    -- You can control the sampling frequency with the length of 
    -- sequncer_shift_reg and ce_sr.
    --
    -- F(sclk) =F(clk)/(2*(ce_sr'length+1))
    --
    -- Sampling freqency is F(sclk)/ (sequncer_shift_reg'length+1)
    --
    -- with 100MHz and ce_sr being four bits long SCLK is 10MHz.
    -- with sequncer_shift_reg of 19 bits, that gives a sample rate of 0.5MHz
    -----------------------------------------------------------------------------
    signal ce_sr               : std_logic_vector(3 downto 0) := (others=>'X');    
    signal sequncer_shift_reg  : std_logic_vector(18 downto 0) := (others=>'X');
 
    signal clock_state         : std_logic := 'X';
    signal clock_enable        : std_logic := 'X';
    signal din0_shift_reg      : std_logic_vector(15 downto 0) := (others=>'X');
	 
	 
begin

		led <= finished & sample_enable & tx_ENABLE & '0' & mode_lock & "000";
		modelock <= mode_lock;
						
		
    --ed <=std_logic_vector(data_0(11 downto 4));
    -----------------------------------
    -- Generate the clock_enable signal
    -- For the rest of the design.
    --
    -- Change the length of ce_sr to 
    -- change the Serial clock speed
    -----------------------------------
    clock_divide : process(CLK)
        begin
             if rising_edge(CLK) then
                --------------------------------------
                -- Self-recovering in case of a glitch
                --------------------------------------
                if unsigned(ce_sr) = 0 then
                  ce_sr <= ce_sr(ce_sr'high-1 downto 0) & '1';
                  clock_enable <= '1';
                else
                  ce_sr <= ce_sr(ce_sr'high-1 downto 0) & '0';
                  clock_enable <= '0';
               end if;
            end if;
        end process clock_divide;
        
    main : process (CLK)
        begin
            if rising_edge(CLK) then
               if clock_enable = '1' then
                  if clock_state = '0' then
                     -- Things to do on the rising edge of the clock.
                     
                     -----------------------------------------------------------------
                     -- Capture the bits coming in from the ADC
                     -----------------------------------------------------------------
                     if sequncer_shift_reg(16) = '1' then
                        data_0 <= din0_shift_reg(11 downto 0);
								if sample_enable = '1' then
									datain(index) <= (data_0(11 downto 4));
									total <= total + to_integer(unsigned(datain(index)));
									if datain(index) > treshold   then
										mode_lock_pixel <= mode_lock_pixel+1;
									end if;
									
									index <= index + 1;
								end if;
								
								if index = 128 then
									--datain(127) <= "00001010";
									tx_enable <= '1';
									index <= 0;
									finished <= '0';
									if mode_lock_pixel >= 58 then
										mode_lock <='1';
									end if;
									if mode_lock_pixel < 58 then
										mode_lock <='0';
									end if;
									--treshold <= std_logic_vector(to_unsigned(20, 8));
									mode_lock_pixel <= 0;

									total <= 0;
								end if;
					 
                     end if;
                     din0_shift_reg <= din0_shift_reg(din0_shift_reg'high-1 downto 0) & adc_d0;
   
                     -----------------------------------------------------------------
                     -- And update the sequencing shift register
                     -- Self-recovering in case of a glitch
                     -----------------------------------------------------------------
                     if unsigned(sequncer_shift_reg) = 0 then
                        sequncer_shift_reg <= sequncer_shift_reg(sequncer_shift_reg'high-1 downto 0) & '1'; 
                     else
                        sequncer_shift_reg <= sequncer_shift_reg(sequncer_shift_reg'high-1 downto 0) & '0'; 
                     end if;
                     ----------------------------
                     -- Output rising clock edge
                     ----------------------------
                     adc_sclk    <= '1';
                     clock_state <= '1';
                  else
                     ----------------------------
                     -- Output falling clock edge
                     ----------------------------
                     adc_sclk    <= '0';
                     clock_state <= '0';
                  end if;
               end if;
 
               -----------------------------------------------------------------
               -- A special kludge to get CS to rise and fall while SCLK 
               -- is high on the ADC. This ensures setup and hold times are met.
               -----------------------------------------------------------------
               if ce_sr(ce_sr'length/2) = '1' and clock_state = '1' then
                  if sequncer_shift_reg(0) = '1' then
                     adc_cs <= '1';
                  elsif sequncer_shift_reg(1) = '1' then
                     adc_cs <= '0';
						elsif sequncer_shift_reg(16) = '1' then
                  end if;
               end if;
					
				if tx_enable = '1' then
					count <= count + 1;
				end if;
				if count = 800000 then
					finished <= '1';
					tx_enable <= '0';
					count <= 0;
					
            end if;
			
			end if;
        end process main;
		  

------------------------------------------------------------------------
---------------------------TAOS_code------------------------------------
------------------------------------------------------------------------

		process(clk)
		begin
		if (rising_edge(clk)) then
			count2 <= count2 + 1;
			if count2=100 then
				clk1MHz <= not clk1MHz;
				count2 <= 0;
			end if;
		end if;
		end process;

		process (clk1MHz)
		begin
		if (rising_edge(clk1MHz)) then
			count3 <= count3 + 1;
			if count3=1 then
				SI <= '1';
			elsif count3=2 then
				SI <= '0';	
				if finished = '1' then
					sample_enable <= '1';
				end if;
			elsif count3=140+2 then		--- 1us * 10000 = 10 msec
				count3 <= 0;
			end if;
			
			if index = 128 then
				sample_enable <= '0';
			end if;
		end if;
		end process;

		taos_clk <= clk1MHz;
		
------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------
---------------------SERIAL_TX------------------------------------------------------------------			
			
process (clk)
begin
	if	(rising_edge(clk)) then
		if (flag='1') then
				acc<="01111011110111100"; -- for 115.2K acc <= "01111011110111100";
		else
				acc <= std_logic_vector(to_unsigned((to_integer(unsigned(acc((pow-1) downto 0))) + inc),(pow+1)));
		end if;
	end if;
end process;
clk_baud<=acc(pow);	--baud tick genaration is provided with the carry out signal.


-- DATA BUFFERING PROCESS
-- This process stores the DATAIN or tempRAM (used in test mode) in buff_in
-- Generate flag signal (8Khz baud thic) for the generation 
process (clk)
	variable cntTest: integer range 0 to 2*MCLK_CYCLE:=0;
	variable cntData: integer range 0 to 2*MCLK_CYCLE:=0;
	variable indData: integer range 0 to nofData+1:=0; -- DATA index
	variable indTest: integer range 0 to nofTest+1:=0; -- DATA index
begin
	if rising_edge (clk) then
		if tx_start = '1' and tx_enable = '1' then
				cntTest:=0;
				indTest:=0;
				if indData=nofData then
					indData:=0;
				else
					if cntData=0 then
						buff_in<=datain(inddata); --data is buffered
						flag<='1'; -- baud_clk reset
						cntData:=1; --next state
					else
						flag<='0'; -- enables the clk baud
						cntData:=cntData+1;
						if cntData=2*MCLK_CYCLE then
							cntData:=0;				-- new data is coming
							indData:=indData+1;  -- inrement array index
						end if;
						
					end if;
				end if;
		else
			cntTest:=0;
			cntData:=0;
			flag<='0'; 
			indTest:=0;
			indData:=0;			
		end if;
		
	end if;		
end process;


-- SERIAL TX OPERATION
-- This process is used to transmit buff_in (or DATAIN signal) in a serial manner using RS-232 protocol
-- Start bit + 8 bit data + Stop bit
-- with clk_baud signal (115.2KHz) 
process (clk_baud)
variable counter: integer range 0 to WAIT_CYCLE:=0;
begin
	if (rising_edge(clk_baud)) then
		if tx_start='1' and tx_enable = '1' then		
		  case state is    	  
			-- START BIT
			  when START =>	tmp_txd<='0'; 				state <= s1;    
			-- DATA BITS
			  when 	 s1 =>   tmp_txd<=buff_in(0); 	state <= s2;			
			  when 	 s2 =>   tmp_txd<=buff_in(1); 	state <= s3;			
			  when 	 s3 =>   tmp_txd<=buff_in(2); 	state <= s4;		
			  when 	 s4 =>   tmp_txd<=buff_in(3); 	state <= s5;		
			  when 	 s5 =>   tmp_txd<=buff_in(4); 	state <= s6;		
			  when 	 s6 =>   tmp_txd<=buff_in(5); 	state <= s7;		
			  when 	 s7 =>   tmp_txd<=buff_in(6); 	state <= s8;		
			  when 	 s8 =>   tmp_txd<=buff_in(7); 	state <=STOP;		  
			-- STOP BIT  
			  when STOP  =>	tmp_txd<='1';	
			-- WAIT FOR THE NEXT DATA  
					counter:=counter+1;		
					if (counter=WAIT_CYCLE) then
						state<=START;
						counter:=0;
					else
						state<=STOP;
					end if;			
			 when others =>	tmp_txd<='1';	state<= STOP;
			end case;
		else
			state<=START;
			counter:=0;
			tmp_txd<='1';
		end if; 
	end if;
end process;
txd<=tmp_txd; -- serial out




--------------------------------------------------------------------------------------------
-----------------------Pixel Array (128 Byte)-----------------------------------------------
--------------------------------------------------------------------------------------------
		
--			process (taos_serial)
--			begin
--				if falling_edge (taos_serial)  then
--					if finished = '1' then
--						sample_enable <= '1';
--						
--					else
--						sample_enable <= '0';
--					end if;
--				end if;
--			end process;				  

		


end Behavioral;

