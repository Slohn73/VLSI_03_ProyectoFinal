library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity display_control is
    port (
        light_detected    : in STD_LOGIC_VECTOR(3 downto 0);
        obstacle_detected : in STD_LOGIC_VECTOR(3 downto 0);
        state_in          : in STD_LOGIC_VECTOR(3 downto 0);

        ledr : out STD_LOGIC_VECTOR(9 downto 0);
        hex0 : out STD_LOGIC_VECTOR(6 downto 0);
        hex1 : out STD_LOGIC_VECTOR(6 downto 0)
    );
end display_control;

architecture RTL of display_control is

    -- Displays activos en bajo.
    -- Orden de segmentos: g f e d c b a.

    constant DIGIT_0 : STD_LOGIC_VECTOR(6 downto 0)
        := "1000000";

    constant DIGIT_1 : STD_LOGIC_VECTOR(6 downto 0)
        := "1111001";

    constant DIGIT_2 : STD_LOGIC_VECTOR(6 downto 0)
        := "0100100";

    constant DIGIT_3 : STD_LOGIC_VECTOR(6 downto 0)
        := "0110000";

    constant DIGIT_4 : STD_LOGIC_VECTOR(6 downto 0)
        := "0011001";

    constant DIGIT_5 : STD_LOGIC_VECTOR(6 downto 0)
        := "0010010";

    constant DIGIT_6 : STD_LOGIC_VECTOR(6 downto 0)
        := "0000010";

    constant DIGIT_7 : STD_LOGIC_VECTOR(6 downto 0)
        := "1111000";

    constant DIGIT_8 : STD_LOGIC_VECTOR(6 downto 0)
        := "0000000";

    constant DIGIT_9 : STD_LOGIC_VECTOR(6 downto 0)
        := "0010000";

    constant DASH : STD_LOGIC_VECTOR(6 downto 0)
        := "0111111";

begin

    --------------------------------------------------
    -- LEDs 0 a 3: deteccion de luz.
    --------------------------------------------------

    ledr(3 downto 0) <= light_detected;

    --------------------------------------------------
    -- LEDs 4 a 7: deteccion de obstaculos
    -- o condiciones de medicion inseguras.
    --------------------------------------------------

    ledr(7 downto 4) <= obstacle_detected;

    --------------------------------------------------
    -- LED 8: evasion o recuperacion.
    --------------------------------------------------

    with state_in select

        ledr(8) <= '1' when
            "0100" |
            "0101" |
            "0110" |
            "0111" |
            "1000" |
            "1001",

            '0' when others;

    --------------------------------------------------
    -- LED 9: meta alcanzada.
    --------------------------------------------------

    ledr(9) <= '1'
        when state_in = "1010"
        else '0';

    --------------------------------------------------
    -- Decodificacion de estados.
    --------------------------------------------------

    process(state_in)
    begin

        case state_in is

            when "0000" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_0;

            when "0001" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_1;

            when "0010" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_2;

            when "0011" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_3;

            when "0100" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_4;

            when "0101" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_5;

            when "0110" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_6;

            when "0111" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_7;

            when "1000" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_8;

            when "1001" =>

                hex1 <= DIGIT_0;
                hex0 <= DIGIT_9;

            when "1010" =>

                hex1 <= DIGIT_1;
                hex0 <= DIGIT_0;

            when "1011" =>

                hex1 <= DIGIT_1;
                hex0 <= DIGIT_1;

            when others =>

                hex1 <= DASH;
                hex0 <= DASH;

        end case;

    end process;

end RTL;