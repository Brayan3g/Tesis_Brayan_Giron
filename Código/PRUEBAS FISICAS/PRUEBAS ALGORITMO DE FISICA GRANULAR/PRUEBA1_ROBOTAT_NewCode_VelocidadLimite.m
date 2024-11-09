% Simulación de Dinámica Molecular con Potencial de Lennard-Jones para Robots Móviles

%% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect()

%
DelayTime = 0.1;

% CONEXIÓN A LOS ROBOTS 
% CONEXIÓN POLOLU #1 
robot1_no = 14; % Seleccionar el agente específico a emplear

% CONEXIÓN POLOLU #2 
robot2_no = 10; % Seleccionar el agente específico a emplear

% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 
%%
% Se establece la conexión con los robots
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);
%%
robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);


%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -60, -60);
robotat_3pi_set_wheel_velocities(robot2, 60, -60);

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)

%%

% Variables para las dimensiones del robot
wheel_radius = 32; % radio de las ruedas en mm
wheel_distance = 96 - 2*6.8; % mm

% Variables para las velocidades de las ruedas (en rpm)
v_left = 0; 
v_right = 0;
v_left2 = 0;
v_right2 = 0;

% Parámetros de la simulación
tiempo_simulacion = 50;
paso_tiempo = 0.01;
masa_robot = 1.0;

% Parámetros del potencial de Lennard-Jones
epsilon = 0.4; % 0.4
sigma = 0.3; % 0.3
usar_equilibrio = true;
tiempo_movimiento = 1;
velocidad_lenta = 0.5; %0.5
%-------------------
velocidad = [0, 0];
%-------------------
% Configuración de movimiento
modo_movimiento = 'punto'; % Seleccionamos si queremos usar una 'trayectoria' o un 'punto'
posicion_final = [4, -1.5];

num_pasos = tiempo_simulacion / paso_tiempo;

% Límite de velocidad en rpm
velocidad_maxima_rpm = 60;


%-------------------------------------------
% inicicalizar fuerzas en 0
num_particulas =2
fuerzas = zeros(num_particulas, 2);

%-------------------------------------------

for paso = 1:num_pasos
    tiempo_actual = paso * paso_tiempo;

    % Obtener la posición actual del ROBOT1 (fijo)
    xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
    posx1 = xi1(1) % en m
    posy1 = xi1(2) % en m
% ->    theta1 = atan2d(sind(xi1(4) - offset1), cosd(xi1(4) - offset1));

    % Obtener la posición actual del ROBOT2 (móvil)
    xi2 = robotat_get_pose(robotat, 10, 'eulzyx');
    posx2 = xi2(1) % en m
    posy2 = xi2(2) % en m
    theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));

    

%     % Calcular la dirección y velocidad para el ROBOT2
%     if usar_equilibrio && tiempo_actual > tiempo_movimiento
%         if strcmp(modo_movimiento, 'punto')
%             direccion = posicion_final - [posx2, posy2];
%         else
%             % Aquí se podría implementar el caso para trayectorias, si es necesario
%         end
%         
%         if norm(direccion) > 0
%             direccion_normalizada = direccion / norm(direccion);
%             velocidad = velocidad_lenta * direccion_normalizada;
%         else
%             velocidad = [0, 0];
%         end
    %end

    % Calcular las fuerzas solo para el ROBOT2
    r_ij = [posx2, posy2] - [posx1, posy1];
    r = norm(r_ij)

    fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
    %-------------------------------------------
    fuerzas(2, :) = fuerzas(2, :) + fuerza_ij;
    %-------------------------------------------
    aceleracion = fuerzas / masa_robot;
    velocidad = velocidad + (aceleracion * paso_tiempo);
    %---------------
    vel_1 = velocidad(1);
    %---------------
    % Convertir la velocidad calculada a velocidad de las ruedas en rpm
    %v = norm(velocidad) * 1000 / (2 * pi * wheel_radius) % Convertir m/s a mm/s y luego a rpm
    v = velocidad(2) * 1000 / (2 * pi * wheel_radius) % Convertir m/s a mm/s y luego a rpm
    v_left2 = v;
    v_right2 = v;

    % Restringir las velocidades a estar dentro del rango [-60, 60] rpm
    v_left2 = max(min(v_left2, velocidad_maxima_rpm), -velocidad_maxima_rpm);
    v_right2 = max(min(v_right2, velocidad_maxima_rpm), -velocidad_maxima_rpm);

    % Enviar las velocidades a los motores de las ruedas del ROBOT2
    robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);
    
    pause(0.01);
end
