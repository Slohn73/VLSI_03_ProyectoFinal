library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity robot_fsm is
    port (
        clk   : in STD_LOGIC;
        reset : in STD_LOGIC;

        -- Desde light_detector.
        light_detected : in STD_LOGIC_VECTOR(3 downto 0);

        -- Desde ultrasonic_controller.
        distance_front : in STD_LOGIC_VECTOR(8 downto 0);
        distance_left  : in STD_LOGIC_VECTOR(8 downto 0);
        distance_right : in STD_LOGIC_VECTOR(8 downto 0);
        distance_back  : in STD_LOGIC_VECTOR(8 downto 0);

        dist_valid : in STD_LOGIC_VECTOR(3 downto 0);
        scan_done  : in STD_LOGIC;

        -- Desde motor_control.
        motor_ready : in STD_LOGIC;

        -- Desde maneuver_timer.
        timer_done : in STD_LOGIC;
        timer_busy : in STD_LOGIC;

        -- Hacia motor_control.
        command     : out STD_LOGIC_VECTOR(2 downto 0);
        speed_left  : out STD_LOGIC_VECTOR(7 downto 0);
        speed_right : out STD_LOGIC_VECTOR(7 downto 0);

        -- Hacia maneuver_timer.
        timer_start       : out STD_LOGIC;
        timer_cancel      : out STD_LOGIC;
        timer_duration_ms : out STD_LOGIC_VECTOR(15 downto 0);

        -- Hacia display_control.
        state_out : out STD_LOGIC_VECTOR(3 downto 0)
    );
end robot_fsm;

architecture RTL of robot_fsm is

    --------------------------------------------------
    -- Comandos de movimiento.
    --------------------------------------------------

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

    --------------------------------------------------
    -- Velocidades PWM.
    --------------------------------------------------

    constant V_STOP : STD_LOGIC_VECTOR(7 downto 0)
        := x"00";

    constant V_FORWARD : STD_LOGIC_VECTOR(7 downto 0)
        := x"80";  -- 128, aproximadamente 50 %

    constant V_EVADE : STD_LOGIC_VECTOR(7 downto 0)
        := x"59";  -- 89, aproximadamente 35 %

    constant V_SEARCH : STD_LOGIC_VECTOR(7 downto 0)
        := x"4D";  -- 77, aproximadamente 30 %

    constant V_FAST_CORRECTION : STD_LOGIC_VECTOR(7 downto 0)
        := x"6E";  -- 110

    --------------------------------------------------
    -- Distancias y tiempos configurables.
    --------------------------------------------------

    constant D_STOP       : natural := 25;
    constant D_CLEAR      : natural := 30;
    constant D_SIDE_MIN   : natural := 10;

    constant D_WALL_LOW   : natural := 12;
    constant D_WALL_HIGH  : natural := 18;
    constant D_WALL_END   : natural := 30;

    constant D_GOAL_MIN   : natural := 20;
    constant D_GOAL_MAX   : natural := 25;

    constant TURN_MS      : natural := 500;
    constant REVERSE_MS   : natural := 300;
    constant EDGE_MS      : natural := 300;

    -- 42 barridos de unos 240 ms = unos 10 s.
    constant MAX_EVASION_SCANS : natural := 42;

    constant MAX_RETRIES : natural := 3;

    -- Maximo tiempo sin recibir un barrido completo.
    -- 350 ms a 50 MHz.
    constant WATCHDOG_CYCLES : positive := 17500000;

    -- Patron de luz necesario para evaluar la meta.
    -- Inicialmente solo se exige el LDR frontal.
    constant GOAL_MASK : STD_LOGIC_VECTOR(3 downto 0)
        := "0001";

    constant GOAL_PATTERN : STD_LOGIC_VECTOR(3 downto 0)
        := "0001";

    --------------------------------------------------
    -- Estados.
    -- El orden define el codigo de state_out.
    --------------------------------------------------

    type state_type is (
        INIT,
        SEARCH_LIGHT,
        FOLLOW_LIGHT,
        CHECK_GOAL,
        SELECT_DIRECTION,
        TURN,
        FOLLOW_WALL,
        CLEAR_EDGE,
        REACQUIRE_LIGHT,
        RECOVERY,
        GOAL,
        STOP
    );

    signal state      : state_type := INIT;
    signal next_state : state_type;

    --------------------------------------------------
    -- Senales internas.
    --------------------------------------------------

    signal cmd_int : STD_LOGIC_VECTOR(2 downto 0);

    signal speed_l_int : STD_LOGIC_VECTOR(7 downto 0);
    signal speed_r_int : STD_LOGIC_VECTOR(7 downto 0);

    signal front_cm : natural range 0 to 511;
    signal left_cm  : natural range 0 to 511;
    signal right_cm : natural range 0 to 511;
    signal back_cm  : natural range 0 to 511;

    signal front_valid : boolean;
    signal left_valid  : boolean;
    signal right_valid : boolean;
    signal back_valid  : boolean;

    signal forward_safe : boolean;
    signal reverse_safe : boolean;
    signal turn_safe    : boolean;

    signal left_clear  : boolean;
    signal right_clear : boolean;

    signal goal_candidate : boolean;

    -- 1 = pared a la derecha.
    -- 0 = pared a la izquierda.
    signal wall_on_right : STD_LOGIC := '1';

    signal wall_cm    : natural range 0 to 511;
    signal wall_valid : boolean;

    signal wall_seen : STD_LOGIC := '0';

    signal edge_count : natural range 0 to 3 := 0;

    signal state_scans : natural range 0 to 63 := 0;

    signal evasion_scans : natural range 0 to 63 := 0;

    signal aligned_scans : natural range 0 to 2 := 0;

    signal retries : natural range 0 to MAX_RETRIES := 0;

    signal scan_age : natural range
        0 to WATCHDOG_CYCLES := WATCHDOG_CYCLES;

    -- Control del temporizador.
    signal timer_armed : STD_LOGIC := '0';
    signal start_int   : STD_LOGIC;

    signal turn_finished : STD_LOGIC := '0';

    signal turn_scans : natural range 0 to 2 := 0;

    -- Deteccion aproximada de falta de progreso.
    signal progress_anchor : natural range 0 to 511 := 0;

    signal stall_count : natural range 0 to 10 := 0;

    signal good_progress : natural range 0 to 3 := 0;

begin

    --------------------------------------------------
    -- Conversion de las distancias.
    --------------------------------------------------

    front_cm <= to_integer(unsigned(distance_front));
    left_cm  <= to_integer(unsigned(distance_left));
    right_cm <= to_integer(unsigned(distance_right));
    back_cm  <= to_integer(unsigned(distance_back));

    front_valid <= dist_valid(0) = '1' and
                   front_cm >= 2 and front_cm <= 400;

    left_valid <= dist_valid(1) = '1' and
                  left_cm >= 2 and left_cm <= 400;

    right_valid <= dist_valid(2) = '1' and
                   right_cm >= 2 and right_cm <= 400;

    back_valid <= dist_valid(3) = '1' and
                  back_cm >= 2 and back_cm <= 400;

    --------------------------------------------------
    -- Condiciones de seguridad.
    --------------------------------------------------

    forward_safe <= front_valid and front_cm > D_STOP
        and not (left_valid and left_cm < D_SIDE_MIN)
        and not (right_valid and right_cm < D_SIDE_MIN);

    reverse_safe <= back_valid and back_cm > D_STOP;

    turn_safe <=
        front_valid and left_valid and
        right_valid and back_valid and
        front_cm >= D_SIDE_MIN and
        left_cm  >= D_SIDE_MIN and
        right_cm >= D_SIDE_MIN and
        back_cm  >= D_SIDE_MIN;

    left_clear <= left_valid and left_cm >= D_CLEAR;

    right_clear <= right_valid and right_cm >= D_CLEAR;

    goal_candidate <=
        (light_detected and GOAL_MASK) = GOAL_PATTERN
        and front_valid
        and front_cm >= D_GOAL_MIN
        and front_cm <= D_GOAL_MAX;

    wall_cm <= right_cm
        when wall_on_right = '1'
        else left_cm;

    wall_valid <= right_valid
        when wall_on_right = '1'
        else left_valid;

    --------------------------------------------------
    -- Salidas de control.
    --------------------------------------------------

    command     <= cmd_int;
    speed_left  <= speed_l_int;
    speed_right <= speed_r_int;

    state_out <= STD_LOGIC_VECTOR(
        to_unsigned(state_type'pos(state), 4)
    );

    --------------------------------------------------
    -- Control del temporizador externo.
    --------------------------------------------------

    timer_duration_ms <=
        STD_LOGIC_VECTOR(to_unsigned(TURN_MS, 16))
            when state = TURN else

        STD_LOGIC_VECTOR(to_unsigned(REVERSE_MS, 16))
            when state = RECOVERY else

        STD_LOGIC_VECTOR(to_unsigned(EDGE_MS, 16))
            when state = CLEAR_EDGE else

        (others => '0');

    start_int <= '1'
        when (
            (state = TURN and turn_finished = '0') or
            state = CLEAR_EDGE or
            state = RECOVERY
        )
        and timer_armed = '0'
        and timer_busy = '0'
        and motor_ready = '1'
        and cmd_int /= CMD_STOP
        and next_state = state
        and reset = '0'
        else '0';

    timer_start <= start_int;

    timer_cancel <= '1'
        when next_state /= state or reset = '1'
        else '0';

    --------------------------------------------------
    -- Logica combinacional de la FSM.
    -- VHDL-2008: process(all).
    --------------------------------------------------

    process(all)

        variable ns : state_type;

        variable c : STD_LOGIC_VECTOR(2 downto 0);

        variable vl : STD_LOGIC_VECTOR(7 downto 0);
        variable vr : STD_LOGIC_VECTOR(7 downto 0);

    begin

        ns := state;

        -- Detencion predeterminada.
        c  := CMD_STOP;
        vl := V_STOP;
        vr := V_STOP;

        case state is

            ------------------------------------------
            -- Inicializacion.
            ------------------------------------------

            when INIT =>

                if scan_age < WATCHDOG_CYCLES then

                    ns := SEARCH_LIGHT;

                end if;

            ------------------------------------------
            -- Buscar o recuperar la fuente luminosa.
            ------------------------------------------

            when SEARCH_LIGHT | REACQUIRE_LIGHT =>

                if light_detected(0) = '1' then

                    -- Esperar lecturas frescas despues
                    -- de terminar la orientacion.
                    if aligned_scans >= 2 then

                        ns := FOLLOW_LIGHT;

                    end if;

                elsif state_scans >= MAX_EVASION_SCANS then

                    ns := RECOVERY;

                elsif turn_safe then

                    if light_detected(1) = '1' and
                       light_detected(2) = '0' then

                        c := CMD_LEFT;

                    else

                        -- Busqueda por defecto a derecha.
                        c := CMD_RIGHT;

                    end if;

                    vl := V_SEARCH;
                    vr := V_SEARCH;

                elsif state_scans >= 2 then

                    ns := SELECT_DIRECTION;

                end if;

            ------------------------------------------
            -- Seguimiento de la luz.
            ------------------------------------------

            when FOLLOW_LIGHT =>

                if not front_valid then

                    if state_scans >= 6 then

                        ns := RECOVERY;

                    end if;

                elsif front_cm <= D_STOP then

                    if goal_candidate then

                        ns := CHECK_GOAL;

                    else

                        ns := SELECT_DIRECTION;

                    end if;

                elsif not forward_safe then

                    ns := SELECT_DIRECTION;

                elsif stall_count >= 10 then

                    ns := RECOVERY;

                elsif light_detected(0) = '0' then

                    ns := SEARCH_LIGHT;

                else

                    c  := CMD_FORWARD;
                    vl := V_FORWARD;
                    vr := V_FORWARD;

                    -- Correccion suave izquierda.
                    if light_detected(1) = '1' and
                       light_detected(2) = '0' and
                       left_clear then

                        vl := V_EVADE;

                    -- Correccion suave derecha.
                    elsif light_detected(2) = '1' and
                          light_detected(1) = '0' and
                          right_clear then

                        vr := V_EVADE;

                    end if;

                end if;

            ------------------------------------------
            -- Confirmar llegada a la meta.
            ------------------------------------------

            when CHECK_GOAL =>

                if not goal_candidate then

                    if front_valid and front_cm > D_STOP then

                        ns := FOLLOW_LIGHT;

                    else

                        ns := SELECT_DIRECTION;

                    end if;

                elsif state_scans >= 6 then

                    -- Confirmacion sostenida superior
                    -- a un segundo con el barrido nominal.
                    ns := GOAL;

                end if;

            ------------------------------------------
            -- Evaluar las rutas laterales.
            ------------------------------------------

            when SELECT_DIRECTION =>

                -- Dos barridos para evaluar con
                -- el carrito detenido.
                if state_scans >= 2 then

                    if turn_safe and
                       (left_clear or right_clear) then

                        ns := TURN;

                    else

                        ns := RECOVERY;

                    end if;

                end if;

            ------------------------------------------
            -- Giro hacia el lado seleccionado.
            ------------------------------------------

            when TURN =>

                if turn_finished = '1' then

                    -- Esperar nuevas mediciones despues
                    -- del giro antes de avanzar.
                    if turn_scans >= 2 then

                        if forward_safe and
                           front_cm >= D_CLEAR then

                            ns := FOLLOW_WALL;

                        else

                            ns := RECOVERY;

                        end if;

                    end if;

                elsif timer_done = '1' then

                    -- Detener hasta actualizar distancias.
                    null;

                elsif not turn_safe then

                    ns := RECOVERY;

                else

                    if wall_on_right = '1' then

                        c := CMD_LEFT;

                    else

                        c := CMD_RIGHT;

                    end if;

                    vl := V_EVADE;
                    vr := V_EVADE;

                end if;

            ------------------------------------------
            -- Seguimiento del contorno de una pared.
            ------------------------------------------

            when FOLLOW_WALL =>

                if not front_valid or not wall_valid then

                    if state_scans >= 6 then

                        ns := RECOVERY;

                    end if;

                elsif front_cm <= D_STOP then

                    if turn_safe then

                        -- Otra cara del obstaculo.
                        -- Se conserva el lado de rodeo.
                        ns := TURN;

                    else

                        ns := RECOVERY;

                    end if;

                elsif not forward_safe or
                      wall_cm < D_SIDE_MIN then

                    ns := RECOVERY;

                elsif edge_count >= 3 then

                    ns := CLEAR_EDGE;

                else

                    c  := CMD_FORWARD;
                    vl := V_EVADE;
                    vr := V_EVADE;

                    if wall_on_right = '1' then

                        -- Pared a la derecha.

                        if wall_cm < D_WALL_LOW and
                           left_valid and
                           left_cm >= D_SIDE_MIN then

                            -- Alejarse hacia la izquierda.
                            vl := V_SEARCH;
                            vr := V_FAST_CORRECTION;

                        elsif wall_cm > D_WALL_HIGH and
                              wall_cm < D_WALL_END then

                            -- Acercarse hacia la derecha.
                            vl := V_FAST_CORRECTION;
                            vr := V_SEARCH;

                        end if;

                    else

                        -- Pared a la izquierda.

                        if wall_cm < D_WALL_LOW and
                           right_valid and
                           right_cm >= D_SIDE_MIN then

                            -- Alejarse hacia la derecha.
                            vl := V_FAST_CORRECTION;
                            vr := V_SEARCH;

                        elsif wall_cm > D_WALL_HIGH and
                              wall_cm < D_WALL_END then

                            -- Acercarse hacia la izquierda.
                            vl := V_SEARCH;
                            vr := V_FAST_CORRECTION;

                        end if;

                    end if;

                end if;

            ------------------------------------------
            -- Avance adicional al superar un extremo.
            ------------------------------------------

            when CLEAR_EDGE =>

                if timer_done = '1' then

                    ns := REACQUIRE_LIGHT;

                elsif not forward_safe then

                    if turn_safe then

                        ns := TURN;

                    else

                        ns := RECOVERY;

                    end if;

                else

                    c  := CMD_FORWARD;
                    vl := V_EVADE;
                    vr := V_EVADE;

                end if;

            ------------------------------------------
            -- Recuperacion sin encoders.
            ------------------------------------------

            when RECOVERY =>

                if timer_done = '1' then

                    ns := SELECT_DIRECTION;

                elsif reverse_safe then

                    c  := CMD_BACKWARD;
                    vl := V_EVADE;
                    vr := V_EVADE;

                elsif timer_armed = '1' then

                    -- El camino posterior se bloqueo
                    -- durante el retroceso.
                    ns := SELECT_DIRECTION;

                elsif state_scans >= 2 then

                    if turn_safe and
                       (left_clear or right_clear) then

                        ns := SELECT_DIRECTION;

                    else

                        ns := STOP;

                    end if;

                end if;

            ------------------------------------------
            -- Estados finales.
            ------------------------------------------

            when GOAL | STOP =>

                null;

        end case;

        ------------------------------------------------
        -- Limite acumulado de tiempo de evasion.
        ------------------------------------------------

        if (
            state = SELECT_DIRECTION or
            state = TURN or
            state = FOLLOW_WALL or
            state = CLEAR_EDGE or
            state = REACQUIRE_LIGHT
        ) and evasion_scans >= MAX_EVASION_SCANS then

            ns := RECOVERY;

        end if;

        ------------------------------------------------
        -- Limite de intentos de recuperacion.
        ------------------------------------------------

        if ns = RECOVERY and state /= RECOVERY and
           retries >= MAX_RETRIES then

            ns := STOP;

        end if;

        ------------------------------------------------
        -- Watchdog de mediciones ultrasonicas.
        ------------------------------------------------

        if state /= INIT and state /= GOAL and
           state /= STOP and
           scan_age = WATCHDOG_CYCLES then

            ns := STOP;

        end if;

        ------------------------------------------------
        -- Detener durante cambios de estado.
        ------------------------------------------------

        if ns /= state then

            c  := CMD_STOP;
            vl := V_STOP;
            vr := V_STOP;

        end if;

        if reset = '1' then

            ns := INIT;
            c  := CMD_STOP;
            vl := V_STOP;
            vr := V_STOP;

        end if;

        next_state  <= ns;
        cmd_int     <= c;
        speed_l_int <= vl;
        speed_r_int <= vr;

    end process;

    --------------------------------------------------
    -- Registros y contadores de supervision.
    --------------------------------------------------

    process(clk)
    begin
        if rising_edge(clk) then

            if reset = '1' then

                state <= INIT;

                wall_on_right <= '1';
                wall_seen     <= '0';

                edge_count    <= 0;
                state_scans   <= 0;
                evasion_scans <= 0;
                aligned_scans <= 0;

                retries <= 0;

                scan_age <= WATCHDOG_CYCLES;

                timer_armed   <= '0';
                turn_finished <= '0';
                turn_scans    <= 0;

                progress_anchor <= 0;
                stall_count     <= 0;
                good_progress   <= 0;

            else

                state <= next_state;

                --------------------------------------
                -- Supervision de nuevos barridos.
                --------------------------------------

                if scan_done = '1' then

                    scan_age <= 0;

                elsif scan_age < WATCHDOG_CYCLES then

                    scan_age <= scan_age + 1;

                end if;

                --------------------------------------
                -- Tiempo transcurrido en cada estado.
                --------------------------------------

                if next_state /= state then

                    state_scans <= 0;

                elsif scan_done = '1' then

                    if state_scans < 63 then

                        state_scans <= state_scans + 1;

                    end if;

                end if;

                --------------------------------------
                -- Tiempo acumulado de evasion.
                --------------------------------------

                if state = SELECT_DIRECTION or
                   state = TURN or
                   state = FOLLOW_WALL or
                   state = CLEAR_EDGE or
                   state = REACQUIRE_LIGHT then

                    if scan_done = '1' and
                       evasion_scans < 63 then

                        evasion_scans <= evasion_scans + 1;

                    end if;

                else

                    evasion_scans <= 0;

                end if;

                --------------------------------------
                -- Seleccionar el lado del obstaculo.
                --------------------------------------

                if state = SELECT_DIRECTION and
                   next_state = TURN then

                    if left_clear and
                       (not right_clear or left_cm >= right_cm) then

                        -- Girar a la izquierda:
                        -- la pared queda a la derecha.
                        wall_on_right <= '1';

                    else

                        wall_on_right <= '0';

                    end if;

                end if;

                --------------------------------------
                -- Confirmacion de orientacion luminosa.
                --------------------------------------

                if (
                    state = SEARCH_LIGHT or
                    state = REACQUIRE_LIGHT
                ) and light_detected(0) = '1'
                and next_state = state then

                    if scan_done = '1' and aligned_scans < 2 then

                        aligned_scans <= aligned_scans + 1;

                    end if;

                else

                    aligned_scans <= 0;

                end if;

                --------------------------------------
                -- Control del temporizador.
                --------------------------------------

                if next_state /= state then

                    timer_armed <= '0';

                elsif start_int = '1' then

                    timer_armed <= '1';

                end if;

                --------------------------------------
                -- Confirmar finalizacion del giro.
                --------------------------------------

                if next_state /= state or state /= TURN then

                    turn_finished <= '0';
                    turn_scans <= 0;

                elsif timer_done = '1' then

                    turn_finished <= '1';
                    turn_scans <= 0;

                elsif turn_finished = '1' and
                      scan_done = '1' and turn_scans < 2 then

                    turn_scans <= turn_scans + 1;

                end if;

                --------------------------------------
                -- Confirmacion del extremo de la pared.
                --------------------------------------

                if state /= FOLLOW_WALL or
                   next_state /= state then

                    wall_seen  <= '0';
                    edge_count <= 0;

                elsif scan_done = '1' then

                    if wall_valid and wall_cm < D_WALL_END then

                        wall_seen  <= '1';
                        edge_count <= 0;

                    elsif wall_seen = '1' and
                          wall_valid and
                          wall_cm >= D_WALL_END and
                          forward_safe then

                        if edge_count < 3 then

                            edge_count <= edge_count + 1;

                        end if;

                    else

                        edge_count <= 0;

                    end if;

                end if;

                --------------------------------------
                -- Deteccion aproximada de atasco.
                --------------------------------------

                if state = FOLLOW_LIGHT and
                   cmd_int = CMD_FORWARD and
                   speed_l_int = V_FORWARD and
                   speed_r_int = V_FORWARD and
                   motor_ready = '1' and
                   front_valid and front_cm >= 30
                   and front_cm <= 150 then

                    if scan_done = '1' then

                        if stall_count = 0 then

                            progress_anchor <= front_cm;
                            stall_count <= 1;

                        elsif abs(
                            integer(front_cm) -
                            integer(progress_anchor)
                        ) <= 2 then

                            if stall_count < 10 then

                                stall_count <= stall_count + 1;

                            end if;

                            good_progress <= 0;

                        else

                            if front_cm + 2 < progress_anchor then

                                if good_progress < 3 then

                                    good_progress <=
                                        good_progress + 1;

                                end if;

                                if good_progress >= 2 then

                                    retries <= 0;

                                end if;

                            else

                                good_progress <= 0;

                            end if;

                            progress_anchor <= front_cm;
                            stall_count <= 1;

                        end if;

                    end if;

                else

                    stall_count   <= 0;
                    good_progress <= 0;

                end if;

                --------------------------------------
                -- Contador de intentos de recuperacion.
                --------------------------------------

                if next_state = RECOVERY and
                   state /= RECOVERY then

                    if retries < MAX_RETRIES then

                        retries <= retries + 1;

                    end if;

                end if;

            end if;

        end if;
    end process;

end RTL;