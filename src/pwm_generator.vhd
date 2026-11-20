library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pwm_generator is
    generic (
        CLK_FREQ_HZ : positive := 50000000;
        PWM_FREQ_HZ : positive := 20000
    );
    port (
        clk   : in STD_LOGIC;
        reset : in STD_LOGIC;

        duty_left  : in STD_LOGIC_VECTOR(7 downto 0);
        duty_right : in STD_LOGIC_VECTOR(7 downto 0);

        pwm_left  : out STD_LOGIC;
        pwm_right : out STD_LOGIC
    );
end pwm_generator;

architecture RTL of pwm_generator is

    constant PERIOD : positive :=
        CLK_FREQ_HZ / PWM_FREQ_HZ;

    signal counter : natural range
        0 to PERIOD - 1 := 0;

    signal threshold_left : natural range
        0 to PERIOD := 0;

    signal threshold_right : natural range
        0 to PERIOD := 0;

    --------------------------------------------------
    -- Convierte un duty de 8 bits al numero
    -- correspondiente de ciclos en alto.
    --------------------------------------------------

    function duty_to_threshold(
        duty : STD_LOGIC_VECTOR(7 downto 0)
    ) return natural is
    begin

        return (to_integer(unsigned(duty)) *
                PERIOD + 127) / 255;

    end function;

begin

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                counter         <= 0;
                threshold_left  <= 0;
                threshold_right <= 0;

                pwm_left  <= '0';
                pwm_right <= '0';

            else

                --------------------------------------
                -- Contador y actualizacion de duty.
                --------------------------------------

                if counter = PERIOD - 1 then

                    counter <= 0;

                    threshold_left <=
                        duty_to_threshold(duty_left);

                    threshold_right <=
                        duty_to_threshold(duty_right);

                else

                    counter <= counter + 1;

                end if;

                --------------------------------------
                -- PWM del motor izquierdo.
                --------------------------------------

                if counter < threshold_left then

                    pwm_left <= '1';

                else

                    pwm_left <= '0';

                end if;

                --------------------------------------
                -- PWM del motor derecho.
                --------------------------------------

                if counter < threshold_right then

                    pwm_right <= '1';

                else

                    pwm_right <= '0';

                end if;

            end if;

        end if;
    end process;

end RTL;