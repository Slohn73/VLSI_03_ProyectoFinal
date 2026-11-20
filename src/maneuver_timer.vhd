library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity maneuver_timer is
    port (
        clk   : in STD_LOGIC;
        reset : in STD_LOGIC;

        start       : in STD_LOGIC;
        cancel      : in STD_LOGIC;
        duration_ms : in STD_LOGIC_VECTOR(15 downto 0);

        done : out STD_LOGIC;
        busy : out STD_LOGIC
    );
end maneuver_timer;

architecture RTL of maneuver_timer is

    -- 50000 ciclos de 50 MHz equivalen a 1 ms.
    constant CYCLES_PER_MS : positive := 50000;

    signal cycle_counter : natural range
        0 to CYCLES_PER_MS - 1 := 0;

    signal remaining_ms : natural range
        0 to 65535 := 0;

    signal busy_reg : STD_LOGIC := '0';
    signal done_reg : STD_LOGIC := '0';

begin

    busy <= busy_reg;
    done <= done_reg;

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                cycle_counter <= 0;
                remaining_ms  <= 0;

                busy_reg <= '0';
                done_reg <= '0';

            else

                -- Pulso DONE de un ciclo.
                done_reg <= '0';

                --------------------------------------
                -- Cancelacion prioritaria.
                --------------------------------------

                if cancel = '1' then

                    cycle_counter <= 0;
                    remaining_ms  <= 0;

                    busy_reg <= '0';

                --------------------------------------
                -- Comenzar un nuevo intervalo.
                --------------------------------------

                elsif busy_reg = '0' then

                    if start = '1' then

                        cycle_counter <= 0;

                        if unsigned(duration_ms) = 0 then

                            remaining_ms <= 0;
                            done_reg <= '1';

                        else

                            remaining_ms <=
                                to_integer(unsigned(duration_ms));

                            busy_reg <= '1';

                        end if;

                    end if;

                --------------------------------------
                -- Temporizacion en progreso.
                --------------------------------------

                else

                    if cycle_counter = CYCLES_PER_MS - 1 then

                        cycle_counter <= 0;

                        if remaining_ms = 1 then

                            remaining_ms <= 0;
                            busy_reg <= '0';
                            done_reg <= '1';

                        else

                            remaining_ms <= remaining_ms - 1;

                        end if;

                    else

                        cycle_counter <= cycle_counter + 1;

                    end if;

                end if;

            end if;

        end if;
    end process;

end RTL;