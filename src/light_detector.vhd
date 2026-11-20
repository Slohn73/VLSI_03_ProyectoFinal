library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity light_detector is
    port (
        clk            : in  STD_LOGIC;
        reset          : in  STD_LOGIC;
        ldr_in         : in  STD_LOGIC_VECTOR(3 downto 0);
        light_detected : out STD_LOGIC_VECTOR(3 downto 0)
    );
end light_detector;

architecture RTL of light_detector is

    -- 1 = sensor activo en bajo.
    -- 0 = sensor activo en alto.

    constant ACTIVE_LOW : STD_LOGIC_VECTOR(3 downto 0)
        := "1111";

    -- Filtro de 10 ms con reloj de 50 MHz.
    constant FILTER_CYCLES : positive := 500000;

    signal sync_1 : STD_LOGIC_VECTOR(3 downto 0)
        := ACTIVE_LOW;

    signal sync_2 : STD_LOGIC_VECTOR(3 downto 0)
        := ACTIVE_LOW;

    signal normalized : STD_LOGIC_VECTOR(3 downto 0);

    signal filtered : STD_LOGIC_VECTOR(3 downto 0)
        := (others => '0');

    type counter_array is array (0 to 3) of
        natural range 0 to FILTER_CYCLES - 1;

    signal counter : counter_array := (others => 0);

begin

    --------------------------------------------------
    -- Sincronizacion de entradas.
    --------------------------------------------------

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                sync_1 <= ACTIVE_LOW;
                sync_2 <= ACTIVE_LOW;

            else

                sync_1 <= ldr_in;
                sync_2 <= sync_1;

            end if;

        end if;
    end process;

    --------------------------------------------------
    -- Normalizacion.
    -- Un 1 representa deteccion de luz.
    --------------------------------------------------

    normalized <= sync_2 xor ACTIVE_LOW;

    --------------------------------------------------
    -- Filtrado independiente de los cuatro sensores.
    --------------------------------------------------

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                filtered <= (others => '0');
                counter  <= (others => 0);

            else

                for i in 0 to 3 loop

                    if normalized(i) = filtered(i) then

                        counter(i) <= 0;

                    elsif counter(i) =
                          FILTER_CYCLES - 1 then

                        filtered(i) <= normalized(i);
                        counter(i)  <= 0;

                    else

                        counter(i) <= counter(i) + 1;

                    end if;

                end loop;

            end if;

        end if;
    end process;

    light_detected <= filtered;

end RTL;