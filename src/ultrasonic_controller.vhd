library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ultrasonic_controller is
    generic (
        G_MIN_INTERVAL_CYCLES : positive := 3000000
    );
    port (
        clk   : in STD_LOGIC;
        reset : in STD_LOGIC;

        -- Entradas fisicas ECHO.
        echo_front : in STD_LOGIC;
        echo_left  : in STD_LOGIC;
        echo_right : in STD_LOGIC;
        echo_back  : in STD_LOGIC;

        -- Salidas fisicas TRIG.
        trig_front : out STD_LOGIC;
        trig_left  : out STD_LOGIC;
        trig_right : out STD_LOGIC;
        trig_back  : out STD_LOGIC;

        -- Distancias en centimetros.
        distance_front : out STD_LOGIC_VECTOR(8 downto 0);
        distance_left  : out STD_LOGIC_VECTOR(8 downto 0);
        distance_right : out STD_LOGIC_VECTOR(8 downto 0);
        distance_back  : out STD_LOGIC_VECTOR(8 downto 0);

        dist_valid        : out STD_LOGIC_VECTOR(3 downto 0);
        no_echo           : out STD_LOGIC_VECTOR(3 downto 0);
        obstacle_detected : out STD_LOGIC_VECTOR(3 downto 0);

        busy      : out STD_LOGIC;
        scan_done : out STD_LOGIC
    );
end ultrasonic_controller;

architecture RTL of ultrasonic_controller is

    type state_type is (
        START_READER,
        WAIT_READER,
        WAIT_INTERVAL,
        ADVANCE_SENSOR,
        FINISH_SCAN
    );

    signal state : state_type := START_READER;

    signal sensor_index : natural range 0 to 3 := 0;

    signal interval_counter : natural range
        0 to G_MIN_INTERVAL_CYCLES := 0;

    signal start_pulse : STD_LOGIC_VECTOR(3 downto 0);
    signal done_reader  : STD_LOGIC_VECTOR(3 downto 0);
    signal busy_reader  : STD_LOGIC_VECTOR(3 downto 0);

    signal valid_reader   : STD_LOGIC_VECTOR(3 downto 0);
    signal no_echo_reader : STD_LOGIC_VECTOR(3 downto 0);

    signal echo_bus : STD_LOGIC_VECTOR(3 downto 0);
    signal trig_bus : STD_LOGIC_VECTOR(3 downto 0);

    type distance_array is array (0 to 3) of
        STD_LOGIC_VECTOR(8 downto 0);

    signal distances : distance_array;

begin

    --------------------------------------------------
    -- Orden: 0 frontal, 1 izquierda,
    --        2 derecha, 3 posterior.
    --------------------------------------------------

    echo_bus(0) <= echo_front;
    echo_bus(1) <= echo_left;
    echo_bus(2) <= echo_right;
    echo_bus(3) <= echo_back;

    trig_front <= trig_bus(0);
    trig_left  <= trig_bus(1);
    trig_right <= trig_bus(2);
    trig_back  <= trig_bus(3);

    distance_front <= distances(0);
    distance_left  <= distances(1);
    distance_right <= distances(2);
    distance_back  <= distances(3);

    dist_valid <= valid_reader;
    no_echo    <= no_echo_reader;

    --------------------------------------------------
    -- Instancias de los cuatro HC-SR04.
    --------------------------------------------------

    GEN_READERS : for i in 0 to 3 generate
				
			start_pulse(i) <= '1'
				when state = START_READER and
						sensor_index = i and reset = '0'
				else '0';

        READER : entity work.hc_sr04_reader

            port map (
                clk         => clk,
                reset       => reset,
                start => start_pulse(i),
                echo        => echo_bus(i),
                trig        => trig_bus(i),
                distance_cm => distances(i),
                valid       => valid_reader(i),
                no_echo     => no_echo_reader(i),
                busy        => busy_reader(i),
                done        => done_reader(i)
            );

    end generate GEN_READERS;

    --------------------------------------------------
    -- Indicadores de obstaculos.
    -- Una lectura invalida se indica como insegura.
    --------------------------------------------------

    obstacle_detected(0) <= '1'
        when valid_reader(0) /= '1' or
             unsigned(distances(0)) <= 25
        else '0';

    obstacle_detected(1) <= '1'
        when valid_reader(1) /= '1' or
             unsigned(distances(1)) < 10
        else '0';

    obstacle_detected(2) <= '1'
        when valid_reader(2) /= '1' or
             unsigned(distances(2)) < 10
        else '0';

    obstacle_detected(3) <= '1'
        when valid_reader(3) /= '1' or
             unsigned(distances(3)) <= 25
        else '0';

    busy <= '0' when reset = '1' else '1';

    scan_done <= '1'
        when state = FINISH_SCAN and reset = '0'
        else '0';

    --------------------------------------------------
    -- Control secuencial de los sensores.
    --------------------------------------------------

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                state            <= START_READER;
                sensor_index     <= 0;
                interval_counter <= 0;

            else

                case state is

                    when START_READER =>

                        interval_counter <= 0;
                        state <= WAIT_READER;

                    when WAIT_READER =>

                        if interval_counter <
                           G_MIN_INTERVAL_CYCLES then

                            interval_counter <=
                                interval_counter + 1;

                        end if;

                        if done_reader(sensor_index) = '1' then

                            if interval_counter >=
                               G_MIN_INTERVAL_CYCLES - 1 then

                                state <= ADVANCE_SENSOR;

                            else

                                state <= WAIT_INTERVAL;

                            end if;

                        end if;

                    when WAIT_INTERVAL =>

                        if interval_counter >=
                           G_MIN_INTERVAL_CYCLES - 1 then

                            state <= ADVANCE_SENSOR;

                        else

                            interval_counter <=
                                interval_counter + 1;

                        end if;

                    when ADVANCE_SENSOR =>

                        if sensor_index = 3 then

                            state <= FINISH_SCAN;

                        else

                            sensor_index <= sensor_index + 1;
                            state <= START_READER;

                        end if;

                    when FINISH_SCAN =>

                        sensor_index <= 0;
                        state <= START_READER;

                end case;

            end if;

        end if;
    end process;

end RTL;