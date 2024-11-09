% Simulación de Dinámica Molecular con Potencial de Lennard-Jones para Robots Móviles
%% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect() % conexion al robotat



%% CONEXIÓN A LOS ROBOTS 

% CONEXIÓN POLOLU #1 
robot1_no = 7; % Seleccionar el agente específico a emplear

% CONEXIÓN POLOLU #2 
robot2_no = 8; % Seleccionar el agente específico a emplear

% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 
%%
% Se establece la conexión con los robots
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);
%
robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);

%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -80, -80);
robotat_3pi_set_wheel_velocities(robot2, -80, -80);

%%  Prueba de Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)

%% variables de setup
% Variables para las dimensiones del robot
wheel_radius = 32; % radio de las ruedas en mm
wheel_distance = 96 - 2*6.8; % mm
velocidad_maxima_rpm = 60;
% Variables para las velocidades de las ruedas (en rpm)
v_left = 0; 
v_right = 0;
v_left2 = 0;
v_right2 = 0;

% Parámetros de la simulación
tiempo_simulacion = 50;
paso_tiempo = 0.001;
masa_robot = 1.0;

% Parámetros del potencial de Lennard-Jones
epsilon = 600; % intesidad -> VALORES PROBADOS ANTERIORMENTE : 400; 220.0; 120  
sigma = 0.20; % Alcanse -> VALORES PROBADOS ANTERIORMENTE : 0.2; 0.3  alcanse
% usar_equilibrio = true;
% tiempo_movimiento = 1;

num_particulas = 2;

% Configuración de movimiento
posiciones = zeros(num_particulas, 2);  % Inicializar la matriz de posiciones

% Matriz de velocidades iniciales (en metros/segundo)
velocidades = zeros(num_particulas, 2);  % Inicializar la matriz de velocidades a cero

% Matriz de aceleraciones iniciales (en metros/segundo^2)
aceleraciones = zeros(num_particulas, 2);  % Inicializar la matriz de aceleraciones a cero

% Bucle de simulación utilizando el algoritmo de Verlet
num_pasos = tiempo_simulacion / paso_tiempo;  % Número total de pasos de simulación

% OBTENER POSICIONES INICIALES DE LOS ROBOTS MOVILES
% Obtener la posición actual del ROBOT1: 
xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
posx1 = xi1(1) % en m
posy1 = xi1(2) % en m
theta1 = atan2d(sind(xi1(4) - offset1), cosd(xi1(4) - offset1));

% Obtener la posición actual del ROBOT2 :
xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
posx2 = xi2(1) % en m
posy2 = xi2(2) % en m
theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));

posiciones_ant = [posx1,posy1; posx2, posy2];  % Almacena posiciones anteriores para el algoritmo de Verlet

num_pasos = tiempo_simulacion / paso_tiempo;
fuerzas = zeros(num_particulas, 2);  % Inicializar la matriz de fuerzas

%% Ciclo principal 
% Calcular fuerzas entre partículas utilizando el potencial de Lennard-Jones
for paso = 1:num_pasos
    
    fuerzas = zeros(num_particulas, 2);  % Inicializar la matriz de fuerzas

    for i = 1:num_particulas
            for j = i+1:num_particulas
                % Calcula el vector entre las partículas i y j
                r_ij = posiciones_ant(i, :) - posiciones_ant(j, :);
                r = norm(r_ij);  % Magnitud del vector (distancia entre las partículas)
    
                % Calcular la fuerza usando el potencial de Lennard-Jones
                fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
% sin constantes                %fuerza_ij = epsilon * ( (sigma / r)^12 - 2*(sigma / r)^6) * r_ij / r^2;
                
                % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
                fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
                fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
            end
    end



    % Calcular aceleraciones usando F = ma
    aceleraciones = fuerzas / masa_robot
    
    % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo)
    
    % Eliminar velocidad negativa:
    %for i = 1:num_particulas

    %end
    if velocidades(2,2) < 0
        velocidades(2,2) = 0;
    else 
        velocidades(2,2) = velocidades(2,2);
    end


    % Convertir la velocidad calculada a velocidad de las ruedas en rpm
    %v = velocidades(2,1) * 1000 / (2 * pi * wheel_radius); % Convertir m/s a mm/s y luego a rpm XXXXXX
    v = velocidades(2,2) * 1000 / (2 * pi * wheel_radius); % Convertir m/s a mm/s y luego a rpm YYYYY    
    
    v_left2 = v;
    v_right2 = v;
    
    % Limite de velocidad
    v_left2 = max(min(v_left2, velocidad_maxima_rpm), -velocidad_maxima_rpm);
    v_right2 = max(min(v_right2, velocidad_maxima_rpm), -velocidad_maxima_rpm);

    if posy1>=1.5
    v_left = 0;
    v_right = 0;
    else
      %w = 0;
      v_left = 30;
      v_right = 30;
    end
 

    % Enviar las velocidades a los motores de las ruedas del ROBOT2
    robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);
    robotat_3pi_set_wheel_velocities(robot1, v_left, v_right);

    
    % Obtener la posición actual del ROBOT1 (fijo)
    xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
    posx1 = xi1(1); % en m
    posy1 = xi1(2); % en m
    %theta1 = atan2d(sind(xi1(4) - offset1), cosd(xi1(4) - offset1));

    % Obtener la posición actual del ROBOT2 (móvil)
    xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
    posx2 = xi2(1); % en m
    posy2 = xi2(2); % en m
    theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));



    posiciones_ant = [posx1,posy1; posx2, posy2];  % Almacena posiciones anteriores para el algoritmo de Verlet


    pause(0.01);
end
