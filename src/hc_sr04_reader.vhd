library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity hc_sr04_reader is
    port (
        clk         : in  STD_LOGIC;
        reset       : in  STD_LOGIC;
        start       : in  STD_LOGIC;
        echo        : in  STD_LOGIC;

        trig        : out STD_LOGIC;
        distance_cm : out STD_LOGIC_VECTOR(8 downto 0);
        valid       : out STD_LOGIC;
        no_echo     : out STD_LOGIC;
        busy        : out STD_LOGIC;
        done        : out STD_LOGIC
    );
end hc_sr04_reader;

architecture RTL of hc_sr04_reader is

    -- Reloj de 50 MHz.
    constant TRIG_CYCLES    : positive := 500;
    constant TIMEOUT_CYCLES : positive := 1250000;
    constant CYCLES_PER_CM  : positive := 2900;

    constant MIN_DISTANCE : natural := 2;
    constant MAX_DISTANCE : natural := 400;

    type state_type is (
        IDLE,
        TRIGGER,
        WAIT_ECHO,
        MEASURE_ECHO
    );

    signal state : state_type := IDLE;

    signal trig_counter : natural range
        0 to TRIG_CYCLES - 1 := 0;

    signal timeout_counter : natural range
        0 to TIMEOUT_CYCLES - 1 := 0;

    signal echo_counter : natural range
        0 to TIMEOUT_CYCLES := 0;

    -- Sincronizador de dos etapas.
    signal echo_meta : STD_LOGIC := '0';
    signal echo_sync : STD_LOGIC := '0';

    signal distance_reg : STD_LOGIC_VECTOR(8 downto 0)
        := (others => '1');

    signal valid_reg   : STD_LOGIC := '0';
    signal no_echo_reg : STD_LOGIC := '0';
    signal done_reg    : STD_LOGIC := '0';

begin

    trig <= '1' when state = TRIGGER else '0';

    busy <= '0' when state = IDLE else '1';

    distance_cm <= distance_reg;
    valid       <= valid_reg;
    no_echo     <= no_echo_reg;
    done        <= done_reg;

    process(clk)
        variable calculated_cm : natural;
    begin
        if rising_edge(clk) then

            if reset = '1' then

                state           <= IDLE;
                trig_counter    <= 0;
                timeout_counter <= 0;
                echo_counter    <= 0;

                echo_meta <= '0';
                echo_sync <= '0';

                distance_reg <= (others => '1');
                valid_reg    <= '0';
                no_echo_reg  <= '0';
                done_reg     <= '0';

            else

                echo_meta <= echo;
                echo_sync <= echo_meta;

                -- DONE dura un ciclo de reloj.
                done_reg <= '0';

                case state is

                    ----------------------------------
                    when IDLE =>

                        trig_counter    <= 0;
                        timeout_counter <= 0;
                        echo_counter    <= 0;

                        if start = '1' then

                            if echo_sync = '0' then

                                -- Conservar la medicion anterior
                                -- hasta terminar la nueva.
                                state <= TRIGGER;

                            else

                                -- ECHO estaba alto antes de TRIG.
                                distance_reg <= (others => '1');
                                valid_reg    <= '0';
                                no_echo_reg  <= '0';
                                done_reg     <= '1';

                            end if;

                        end if;

                    ----------------------------------
                    when TRIGGER =>

                        if trig_counter = TRIG_CYCLES - 1 then

                            trig_counter    <= 0;
                            timeout_counter <= 0;
                            state           <= WAIT_ECHO;

                        else

                            trig_counter <= trig_counter + 1;

                        end if;

                    ----------------------------------
                    when WAIT_ECHO =>

                        if echo_sync = '1' then

                            echo_counter <= 1;
                            state        <= MEASURE_ECHO;

                        elsif timeout_counter =
                              TIMEOUT_CYCLES - 1 then

                            -- No se recibio respuesta.
                            distance_reg <= (others => '1');
                            valid_reg    <= '0';
                            no_echo_reg  <= '1';
                            done_reg     <= '1';

                            state <= IDLE;

                        else

                            timeout_counter <= timeout_counter + 1;

                        end if;

                    ----------------------------------
                    when MEASURE_ECHO =>

                        if echo_sync = '0' then

                            calculated_cm :=
                                echo_counter / CYCLES_PER_CM;

                            if calculated_cm >= MIN_DISTANCE and
                               calculated_cm <= MAX_DISTANCE then

                                distance_reg <= STD_LOGIC_VECTOR(
                                    to_unsigned(calculated_cm, 9)
                                );

                                valid_reg <= '1';

                            else

                                -- Distancia fuera del rango.
                                distance_reg <= (others => '1');
                                valid_reg    <= '0';

                            end if;

                            no_echo_reg <= '0';
                            done_reg    <= '1';
                            state       <= IDLE;

                        elsif echo_counter = TIMEOUT_CYCLES then

                            -- ECHO permanecio alto demasiado tiempo.
                            distance_reg <= (others => '1');
                            valid_reg    <= '0';
                            no_echo_reg  <= '0';
                            done_reg     <= '1';

                            state <= IDLE;

                        else

                            echo_counter <= echo_counter + 1;

                        end if;

                end case;

            end if;

        end if;
    end process;

end RTL;