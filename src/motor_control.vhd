library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity motor_control is
    generic (
        CLK_FREQ_HZ   : positive := 50000000;
        DEAD_TIME_MS  : natural  := 30;
        BRAKE_ON_STOP : boolean  := true;
        INVERT_LEFT   : boolean  := false;
        INVERT_RIGHT  : boolean  := false
    );
    port (
        clk   : in STD_LOGIC;
        reset : in STD_LOGIC;

        -- Desde robot_fsm.
        command     : in STD_LOGIC_VECTOR(2 downto 0);
        speed_left  : in STD_LOGIC_VECTOR(7 downto 0);
        speed_right : in STD_LOGIC_VECTOR(7 downto 0);

        -- Hacia el driver TB6612FNG.
        ain1 : out STD_LOGIC;
        ain2 : out STD_LOGIC;
        bin1 : out STD_LOGIC;
        bin2 : out STD_LOGIC;
        stby : out STD_LOGIC;

        -- Hacia pwm_generator.
        duty_left  : out STD_LOGIC_VECTOR(7 downto 0);
        duty_right : out STD_LOGIC_VECTOR(7 downto 0);

        -- Confirmacion hacia robot_fsm.
        motor_ready : out STD_LOGIC
    );
end motor_control;

architecture RTL of motor_control is

    constant CMD_STOP : STD_LOGIC_VECTOR(2 downto 0)
        := "000";

    constant CMD_FORWARD : STD_LOGIC_VECTOR(2 downto 0)
        := "001";

    constant CMD_BACKWARD : STD_LOGIC_VECTOR(2 downto 0)
        := "010";

    constant CMD_LEFT : STD_LOGIC_VECTOR(2 downto 0)
        := "011";

    constant CMD_RIGHT : STD_LOGIC_VECTOR(2 downto 0)
        := "100";

    constant DEAD_CYCLES : natural :=
        (CLK_FREQ_HZ / 1000) * DEAD_TIME_MS;

    signal quiet_counter : natural range
        0 to DEAD_CYCLES := DEAD_CYCLES;

    -- 0 = adelante; 1 = atras.
    signal desired_left  : STD_LOGIC;
    signal desired_right : STD_LOGIC;

    signal last_left  : STD_LOGIC := '0';
    signal last_right : STD_LOGIC := '0';

    signal move_request : STD_LOGIC;

    signal ain1_reg : STD_LOGIC := '0';
    signal ain2_reg : STD_LOGIC := '0';
    signal bin1_reg : STD_LOGIC := '0';
    signal bin2_reg : STD_LOGIC := '0';

    signal stby_reg  : STD_LOGIC := '0';
    signal ready_reg : STD_LOGIC := '0';

    signal applied_command : STD_LOGIC_VECTOR(2 downto 0)
        := CMD_STOP;

begin

    --------------------------------------------------
    -- Decodificacion de los movimientos.
    --------------------------------------------------

    process(command)
    begin

        desired_left  <= '0';
        desired_right <= '0';
        move_request  <= '0';

        case command is

            when CMD_FORWARD =>

                desired_left  <= '0';
                desired_right <= '0';
                move_request  <= '1';

            when CMD_BACKWARD =>

                desired_left  <= '1';
                desired_right <= '1';
                move_request  <= '1';

            when CMD_LEFT =>

                desired_left  <= '1';
                desired_right <= '0';
                move_request  <= '1';

            when CMD_RIGHT =>

                desired_left  <= '0';
                desired_right <= '1';
                move_request  <= '1';

            when others =>

                move_request <= '0';

        end case;

    end process;

    --------------------------------------------------
    -- Control de motores y pausa de inversion.
    --------------------------------------------------

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                ain1_reg <= '0';
                ain2_reg <= '0';
                bin1_reg <= '0';
                bin2_reg <= '0';

                stby_reg  <= '0';
                ready_reg <= '0';

                duty_left  <= (others => '0');
                duty_right <= (others => '0');

                quiet_counter <= DEAD_CYCLES;

                last_left  <= '0';
                last_right <= '0';

                applied_command <= CMD_STOP;

            else

                --------------------------------------
                -- Orden de detencion.
                --------------------------------------

                if move_request = '0' then

                    duty_left  <= (others => '0');
                    duty_right <= (others => '0');

                    if BRAKE_ON_STOP then

                        stby_reg <= '1';

                        ain1_reg <= '1';
                        ain2_reg <= '1';
                        bin1_reg <= '1';
                        bin2_reg <= '1';

                    else

                        stby_reg <= '0';

                        ain1_reg <= '0';
                        ain2_reg <= '0';
                        bin1_reg <= '0';
                        bin2_reg <= '0';

                    end if;

                    if quiet_counter < DEAD_CYCLES then

                        quiet_counter <= quiet_counter + 1;

                    end if;

                    applied_command <= command;
                    ready_reg <= '1';

                --------------------------------------
                -- Pausa si algun motor debe invertir.
                --------------------------------------

                elsif (
                    desired_left /= last_left or
                    desired_right /= last_right
                ) and quiet_counter < DEAD_CYCLES then

                    duty_left  <= (others => '0');
                    duty_right <= (others => '0');

                    -- Frenado de ambas ruedas.
                    stby_reg <= '1';

                    ain1_reg <= '1';
                    ain2_reg <= '1';
                    bin1_reg <= '1';
                    bin2_reg <= '1';

                    quiet_counter <= quiet_counter + 1;

                    ready_reg <= '0';

                --------------------------------------
                -- Ejecutar el movimiento solicitado.
                --------------------------------------

                else

                    stby_reg <= '1';

                    ain1_reg <= not desired_left;
                    ain2_reg <= desired_left;

                    bin1_reg <= not desired_right;
                    bin2_reg <= desired_right;

                    duty_left  <= speed_left;
                    duty_right <= speed_right;

                    last_left  <= desired_left;
                    last_right <= desired_right;

                    quiet_counter <= 0;

                    applied_command <= command;
                    ready_reg <= '1';

                end if;

            end if;

        end if;
    end process;

    --------------------------------------------------
    -- Ajuste de polaridad de los motores.
    --------------------------------------------------

    ain1 <= ain2_reg when INVERT_LEFT else ain1_reg;
    ain2 <= ain1_reg when INVERT_LEFT else ain2_reg;

    bin1 <= bin2_reg when INVERT_RIGHT else bin1_reg;
    bin2 <= bin1_reg when INVERT_RIGHT else bin2_reg;

    stby <= stby_reg when reset = '0' else '0';

    motor_ready <= ready_reg
        when applied_command = command and reset = '0'
        else '0';

end RTL;