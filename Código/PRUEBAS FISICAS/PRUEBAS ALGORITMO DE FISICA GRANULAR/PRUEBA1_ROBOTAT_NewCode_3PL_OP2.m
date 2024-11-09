% Simulación de Dinámica Molecular con Potencial de Lennard-Jones para Robots Móviles
clear;
clc;

% Conexión al Robotat
robotat = robotat_connect(); % conexión al sistema de Robotat

%% Conexión a los robots
robot1_no = 2; % Identificación del robot 1
robot2_no = 3; % Identificación del robot 2
robot3_no = 4; % Identificación del robot 3 (si existe)

% Offsets de los markers (ajustes para la orientación del robot)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130];

% Establecer conexión con los robots
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);

robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);

robot3 = robotat_3pi_connect(robot3_no)  % Si se tiene un tercer robot
offset3 = marker_offsets(robot3_no);

%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -50, -50);
robotat_3pi_set_wheel_velocities(robot2, -50, -50)
robotat_3pi_set_wheel_velocities(robot3, -50, -50)

%%  Prueba de Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)
robotat_3pi_force_stop(robot3)

%% Variables de setup
wheel_radius = 32; % radio de las ruedas en mm
velocidad_maxima_rpm = 60; % velocidad máxima en rpm
masa_robot = 1.0; % masa del robot en kg

% Parámetros del potencial de Lennard-Jones
epsilon = 600; % intensidad
sigma = 0.25; % alcance

% Parámetros de simulación
tiempo_simulacion = 50;
paso_tiempo = 0.001;
num_particulas = 3; % Número de robots

% Inicialización de matrices de posiciones, velocidades, aceleraciones y fuerzas
posiciones_ant = zeros(num_particulas, 2);  % Almacena las posiciones anteriores
velocidades = zeros(num_particulas, 2);  % Velocidades iniciales
fuerzas = zeros(num_particulas, 2);  % Fuerzas aplicadas a cada robot

% Obtener posiciones iniciales de los robots
xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
posx1 = xi1(1); 
posy1 = xi1(2);

xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
posx2 = xi2(1);
posy2 = xi2(2);

xi3 = robotat_get_pose(robotat, robot3_no, 'eulzyx');
posx3 = xi3(1);
posy3 = xi3(2);

posiciones_ant = [posx1, posy1; posx2, posy2; posx3, posy3];  % Inicializar posiciones

num_pasos = tiempo_simulacion / paso_tiempo;  % Número de pasos de la simulación

%% Bucle principal de simulación usando el algoritmo de Verlet


for paso = 1:num_pasos
    % Inicializar matrices de fuerzas y aceleraciones
    fuerzas = zeros(num_particulas, 2);
    aceleraciones = zeros(num_particulas, 2);
    velocidades_rpm = zeros(num_particulas, 2);  % Matriz para almacenar las velocidades en rpm
    
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
    
    % Calcular aceleraciones usando F = ma
    aceleraciones = fuerzas / masa_robot;

    % Actualizar las velocidades de todas las partículas
    for i = 2:num_particulas
        % Actualizar las velocidades utilizando el algoritmo de Verlet
        velocidades(i, :) = velocidades(i, :) + (aceleraciones(i, :) * paso_tiempo);
        
        % Corrección de velocidad negativa
        if velocidades(i, 2) < 0
            velocidades(i, 2) = 0;
        end

        % Convertir la velocidad en m/s a rpm
        v = velocidades(i, 2) * 1000 / (2 * pi * wheel_radius);
        
        % Limitar la velocidad a los valores máximos permitidos
        v_left = max(min(v, velocidad_maxima_rpm), -velocidad_maxima_rpm);
        v_right = max(min(v, velocidad_maxima_rpm), -velocidad_maxima_rpm);
        
        % Almacenar las velocidades calculadas en rpm
        velocidades_rpm(i, :) = [v_left, v_right];
    end
    
    if posy1>=1.5
        velocidades_rpm(1, 1) = 0;
        velocidades_rpm(1, 2) = 0;
    else
        velocidades_rpm(1, 1) = 30;
        velocidades_rpm(1, 2) = 30;
        %w = 0;
    end

    % Enviar las velocidades a los motores de los robots
    robotat_3pi_set_wheel_velocities(robot1, velocidades_rpm(1, 1), velocidades_rpm(1, 2));
    robotat_3pi_set_wheel_velocities(robot2, velocidades_rpm(2, 1), velocidades_rpm(2, 2));
    
    if num_particulas >= 3
        robotat_3pi_set_wheel_velocities(robot3, velocidades_rpm(3, 1), velocidades_rpm(3, 2));
    end

    % Obtener las posiciones actuales de los robots
    xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
    posx1 = xi1(1);
    posy1 = xi1(2);
    
    xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
    posx2 = xi2(1);
    posy2 = xi2(2);
        
    xi3 = robotat_get_pose(robotat, robot3_no, 'eulzyx');
    posx3 = xi3(1);
    posy3 = xi3(2);
    
    % Actualizar las posiciones anteriores
    posiciones_ant(1, :) = [posx1, posy1];
    posiciones_ant(2, :) = [posx2, posy2];
    posiciones_ant(3, :) = [posx3, posy3];
    
    pause(0.01);  % Pausa para permitir que los robots actualicen sus posiciones
end
