% Simulación de Dinámica Molecular con Potencial de Lennard-Jones para Robots Móviles
clear;
clc;

% Conexión al Robotat
robotat = robotat_connect() % conexión al sistema de Robotat

%% Conexión a los robots
robot1_no = 8; % Identificación del robot 1
robot2_no = 5; % Identificación del robot 2
%robot3_no = 4; % Identificación del robot 3 (si existe)

% Offsets de los markers (ajustes para la orientación del robot)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130];

% Establecer conexión con los robots
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);

robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);

% robot3 = robotat_3pi_connect(robot3_no)  % Si se tiene un tercer robot
% offset3 = marker_offsets(robot3_no);

%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -70, -70);
robotat_3pi_set_wheel_velocities(robot2, -70, -70);

%robotat_3pi_set_wheel_velocities(robot3, -50, -50):

%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, 50, 50);
robotat_3pi_set_wheel_velocities(robot2, 50, 50);
%robotat_3pi_set_wheel_velocities(robot3, -50, -50):
%%  Prueba de Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)
%robotat_3pi_force_stop(robot3)

%% Variables de setup
% Parámetros de simulación
tiempo_simulacion = 50;
paso_tiempo = 0.001;% ******probar 0.01****
num_particulas = 2; % Número de robots

% Variables del robot
wheel_radius = 32; % radio de las ruedas en mm
wheel_distance = 96 - 2*6.8;
velocidad_maxima_rpm = 70; %80;%75; %60 velocidad máxima en rpm
masa_robot = 1.0; % masa del robot en kg

% Parámetros del potencial de Lennard-Jones
epsilon = 600; % intensidad
sigma = 0.2;%0.18;%25 % alcance

%----------------------------------------------
% Inicialización de controlador PID
kpO = 1.0; kiO = 0.0; kdO = 0;
eO_1 = 0; eP = 10; EO = 0;
eP2 = 10; EO2 = 0; eO_2 = 0;

% Acercamiento exponencial:
v0 = 190;
alpha = 100;

%--------------------------------------------------------------------------
% Funciones de conversión entre rpm y rad/s para cambiar entre las
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);
%--------------------------------------------------------------------------
% Meta para el robot 1 (solo un punto)
% trayectoria = [1.2, 0.0;
%                1.2,  1.0;
%                0,  1.0;
%                -0.5,  1.0;
%                -0.5,  0;
%                0,  0;
%                0.5,  0;
%                0, 1.5];
% 
% trayectoria = [
%     0.7707    1.2752-2;
%     0.6958    1.2923-2;
%     0.6192    1.2997-2;
%     0.5424    1.2972-2;
%     0.4665    1.2850-2;
%     0.3928    1.2631-2;
%     0.3225    1.2320-2;
%     0.2567    1.1921-2;
%     0.1966    1.1442-2;
%     0.1431    1.0889-2;
%     0.0971    1.0273-2;
%     0.0594    0.9603-2;
%     0.0306    0.8891-2;
%     0.0111    0.8147-2;
%     0.0012    0.7384-2;
%     0.0012    0.6616-2;
%     0.0111    0.5853-2;
%     0.0306    0.5109-2;
%     0.0594    0.4397-2;
%     0.0971    0.3727-2;
%     0.1431    0.3111-2;
%     0.1966    0.2558-2;
%     0.2567    0.2079-2;
%     0.3225    0.1680-2;
%     0.3928    0.1369-2;
%     0.4665    0.1150-2;
%     0.5424    0.1028-2;
%     0.6192    0.1003-2;
%     0.6958    0.1077-2;
%     0.7707    0.1248-2;
%     0.8429    0.1514-2;
%     0.9110    0.1869-2;
%     0.9741    0.2309-2;
%     1.0310    0.2826-2;
%     1.0808    0.3411-2;
%     1.1228    0.4056-2;
%     1.1562    0.4748-2;
%     1.1804    0.5478-2;
%     1.1951    0.6233-2;
%     1.2000    0.7000-2;
%     1.2000    0.7000-2;
%     1.1951    0.7767-2;
%     1.1804    0.8522-2;
%     1.1562    0.9252-2;
%     1.1228    0.9944-2;
%     1.0808    1.0589-2;
%     1.0310    1.1174-2;
%     0.9741    1.1691-2;
%     0.9110    1.2131-2;
%     0.8429    1.2486-2;
% ];

trayectoria = [
    1.0, 1.2;
    0.8667, 1.2;
    0.7333, 1.2;
    0.6, 1.2;
    0.4667, 1.2;
    0.3333, 1.2;
    0.2, 1.2;
    0.0667, 1.2;
   -0.0667, 1.2;
   -0.2, 1.2;
   -0.3333, 1.2;
   -0.4667, 1.2;
   -0.6, 1.2;
   -0.7333, 1.2;
   -0.8667, 1.2;
   -1.0, 1.2;
   -1.0, 1.12;
   -1.0, 1.04;
   -1.0, 0.96;
   -1.0, 0.88;
   -1.0, 0.8;
   -1.0, 0.72;
   -1.0, 0.64;
   -1.0, 0.56;
   -1.0, 0.48;
   -1.0, 0.4;
   -1.0, 0.32;
   -1.0, 0.24;
   -1.0, 0.16;
   -1.0, 0.08;
   -1.0, 0.0;
   -0.8667, 0.0;
   -0.7333, 0.0;
   -0.6, 0.0;
   -0.4667, 0.0;
   -0.3333, 0.0;
   -0.2, 0.0;
   -0.0667, 0.0;
    0.0667, 0.0;
    0.2, 0.0;
    0.3333, 0.0;
    0.4667, 0.0;
    0.6, 0.0;
    0.7333, 0.0;
    0.8667, 0.0;
    1.0, 0.0;
    1.0, 0.08;
    1.0, 0.16;
    1.0, 0.24;
    1.0, 0.32;
    1.0, 0.4;
    1.0, 0.48;
    1.0, 0.56;
    1.0, 0.64;
    1.0, 0.72;
    1.0, 0.8;
    1.0, 0.88;
    1.0, 0.96;
    1.0, 1.04;
    1.0, 1.12
];

trayectoria = [
    1.0, 1.2;
    0.8667, 1.2;
    0.7333, 1.2;
    0.6, 1.2;
    0.4667, 1.2;
    0.3333, 1.2;
    0.2, 1.2;
    0.0667, 1.2;
   -0.0667, 1.2;
   -0.2, 1.2;
   -0.3333, 1.2;
   -0.4667, 1.2;
   -0.6, 1.2;
   -0.7333, 1.2;
   -0.8667, 1.2;
   -1.0, 1.2;
   -1.0, 1.12]

trayectoria = [
    1.2, -1.2;
    1.2, -1.0;
    1.2, -0.5;
    1.2, 0.0;
    1.2, 0.5;
    1.2, 1.0;
    1.2, 1.5]
 


iteraciones = length(trayectoria);
n = 1;




%--------------------------------------------------------------------------
% Inicialización de matrices de posiciones, velocidades, aceleraciones y fuerzas
posiciones_ant = zeros(num_particulas, 2);  % Almacena las posiciones anteriores
velocidades = zeros(num_particulas, 2);  % Velocidades iniciales
fuerzas = zeros(num_particulas, 2);  % Fuerzas aplicadas a cada robot

%--------------------------------------------------------------------------
% Obtener posiciones iniciales de los robots
xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
posx1 = xi1(1) 
posy1 = xi1(2)
theta = atan2d(sind(xi1(4) - offset1), cosd(xi1(4) - offset1));

xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
posx2 = xi2(1)
posy2 = xi2(2)
theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));

% xi3 = robotat_get_pose(robotat, robot3_no, 'eulzyx');
% posx3 = xi3(1);
% posy3 = xi3(2);

%posiciones_ant = [posx1, posy1; posx2, posy2; posx3, posy3];  % Inicializar posiciones
posiciones_ant = [posx1, posy1; posx2, posy2];  % Inicializar posiciones
num_pasos = tiempo_simulacion / paso_tiempo;  % Número de pasos de la simulación

%% Bucle principal de simulación usando el algoritmo de Verlet

for paso = 1:num_pasos
    
    % Inicializar matrices de fuerzas y aceleraciones
    fuerzas = zeros(num_particulas, 2);
    aceleraciones = zeros(num_particulas, 2); % <-comentar o poner fuera *******
    velocidades_rpm = zeros(num_particulas, 2);  % Matriz para almacenar las velocidades en rpm
    %----------------------------------------------------------------------
    % Calcular fuerzas entre partículas consecutivas
    for i = 2:num_particulas
        % Calcular el vector entre la partícula i-1 y la partícula i
        r_ij = posiciones_ant(i-1, :) - posiciones_ant(i, :);
        r = norm(r_ij);  % Distancia entre las partículas

        % Calcular la fuerza de Lennard-Jones que afecta a la partícula i
        fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
        
        % Aplicar la fuerza a la partícula i
        fuerzas(i, :) = fuerzas(i, :) - fuerza_ij;
    end
    %----------------------------------------------------------------------
    % Calcular aceleraciones usando F = ma
    aceleraciones = fuerzas / masa_robot;
    
    %----------------------------------------------------------------------
    % Actualizar las velocidades de todas las partículas
    for i = 2:num_particulas
        % Actualizar las velocidades utilizando el algoritmo de Verlet
        velocidades(i, :) = velocidades(i, :) + (aceleraciones(i, :) * paso_tiempo);
        
        % Corrección de velocidad negativa
        if velocidades(i, 2) < 0
            velocidades(i, 2) = 0
        end
        if velocidades(i, 1) < 0
            velocidades(i, 1) = 0
        end
        %v_sum = norm(velocidades(i, 1)+velocidades(i, 2)) 
        % Convertir la velocidad en m/s a rpm
        %v = v_sum * 1000 / (2 * pi * wheel_radius);
        v = velocidades(i, 2) * 1000 / (2 * pi * wheel_radius);
        % Limitar la velocidad a los valores máximos permitidos
        v_left = max(min(v, velocidad_maxima_rpm), -velocidad_maxima_rpm);
        v_right = max(min(v, velocidad_maxima_rpm), -velocidad_maxima_rpm);
        %v_right = v_left;

        % Almacenar las velocidades calculadas en rpm
        velocidades_rpm(i, :) = [v_left, v_right];
    end
    %----------------------------------------------------------------------
    if posy1>=1.5
        velocidades_rpm(1, 1) = 0;
        velocidades_rpm(1, 2) = 0;
    else
        velocidades_rpm(1, 1) = 30;
        velocidades_rpm(1, 2) = 30;
        %w = 0;
    end
    %--------------------------------------------------
     % Meta del robot 2 es la posición del robot 1
%      xg2 = xi1(1)
%      yg2 = xi1(2)
     if n > iteraciones
        xg = x_meta(1)
        yg = x_meta(2)
        xg2 = x_meta(1)
        yg2 = x_meta(2)
    else
        x_meta = trayectoria(n,:)
        xg = x_meta(1);
        yg = x_meta(2);
        xg2 = x_meta(1)
        yg2 = x_meta(2)
        %xg2 = xi1(1)
        %yg2 = xi1(2)        
     end

    %----------------------------------------------------------------------
    % Controlador Robot 1
    e = [xg - posx1; yg - posy1];
    thetag = atan2d(e(2), e(1));
    eP = norm(e);
    eO = thetag - theta;
    eO = atan2(sind(eO), cosd(eO));
    
    % Control de velocidad lineal
    kP = v0 * (1 - exp(-alpha*eP^2)) / eP;
    v = kP * eP;

    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO * eO + kiO * EO + kdO * eO_D;
    eO_1 = eO;
    %------------------------------------------
    % Controlador Robot 2
    e2 = [xg2 - posx2; yg2 - posy2];
    thetag2 = atan2d(e2(2), e2(1));
    eP2 = norm(e2)
    eO2 = thetag2 - theta2;
    eO2 = atan2(sind(eO2), cosd(eO2));
    
    % Control de velocidad lineal
    kP2 = v0 * (1 - exp(-alpha*eP2^2)) / eP2;
    v2 = kP2 * eP2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_2;
    EO2 = EO2 + eO2;
    w2 = kpO * eO2 + kiO * EO2 + kdO * eO_D2;
    eO_2 = eO2;
    
    %------------------------------------------
    % Detener robot si llega a su meta
    if eP < 0.15
        n= n+1
        if n > iteraciones
          v = 0;
          w = 0;
        end

    end
    if eP2 < 0.25
        v2 = 0; w2 = 0;% revisar posible de mas
    end

    %----------------------------------------------------------------------
    % ROBOT1:
    v_left = (v - wheel_distance * w) / wheel_radius;
    v_right = (v + wheel_distance * w) / wheel_radius;
    
    % ROBOT2:
    v_left2 = (0 - wheel_distance * w2) / wheel_radius;
    v_right2 = (0 + wheel_distance * w2) / wheel_radius;
     
    % Convertir de rad/s a rpm
    %ROBOT1
    v_left = rads2rpm(v_left);
    v_right = rads2rpm(v_right);
    
    % ROBOT2:
    v_left2 = rads2rpm(v_left2);
    v_right2 = rads2rpm(v_right2);
    
    velocidades_rpm(1, 1) =  + v_left;
    velocidades_rpm(1, 2) =  + v_right; 
    
    velocidades_rpm(2, 1) =  velocidades_rpm(2, 1) + v_left2
    velocidades_rpm(2, 2) =  velocidades_rpm(2, 2) + v_right2

    %----------------------------------------------------------------------
    % Enviar las velocidades a los motores de los robots
    robotat_3pi_set_wheel_velocities(robot1, velocidades_rpm(1, 1), velocidades_rpm(1, 2));
    robotat_3pi_set_wheel_velocities(robot2, velocidades_rpm(2, 1), velocidades_rpm(2, 2));
    
    %if num_particulas >= 3
    %    robotat_3pi_set_wheel_velocities(robot3, velocidades_rpm(3, 1), velocidades_rpm(3, 2));
    %end
    %---------------------------------------------------------------------- 
    % Obtener las posiciones actuales de los robots
    xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
    posx1 = xi1(1);
    posy1 = xi1(2);
    theta = atan2d(sind(xi1(4) - offset1), cosd(xi1(4) - offset1))
    
    xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
    posx2 = xi2(1);
    posy2 = xi2(2);
    theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2))

%     xi3 = robotat_get_pose(robotat, robot3_no, 'eulzyx');
%     posx3 = xi3(1);
%     posy3 = xi3(2);
%     
    % Actualizar las posiciones anteriores
    posiciones_ant(1, :) = [posx1, posy1];
    posiciones_ant(2, :) = [posx2, posy2];
    %posiciones_ant(3, :) = [posx3, posy3];

    %----------------------------------------------------------------------
    pause(0.0001);  % Pausa para permitir que los robots actualicen sus posiciones
    %pause(0.0001);
end
