% Simulación de Dinámica Molecular con Potencial de Lennard-Jones para Robots Móviles
%% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect() % conexion al robotat


%% CONEXIÓN A LOS ROBOTS 

% CONEXIÓN POLOLU #1 
robot1_no = 3; % Seleccionar el agente específico a emplear

% CONEXIÓN POLOLU #2 
robot2_no = 5; % Seleccionar el agente específico a emplear

% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 

% Se establece la conexión con los robots
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);

robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);

%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -60, -60);
robotat_3pi_set_wheel_velocities(robot2, -60, -60);

%% prueba para verificar conexion y previamente el envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, 60, 60);
robotat_3pi_set_wheel_velocities(robot2, 60, 60);

%%  Prueba de Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)  

%% variables de setup
% Variables para las dimensiones del robot
% wheel_radius = 32; % radio de las ruedas en mm
% wheel_distance = 96 - 2*6.8; % mm

wheel_radius = 32/1000; % radio de las ruedas en m
wheel_distance = (96 - 2*6.8)/1000; % m

velocidad_maxima_rpm = 60; % / 201.06 mm/s / 0.201 m/s o 0.168 (50) m/s
masa_robot = 1.0;

% Variables para las velocidades de las ruedas (en rpm)
v_left = 0; 
v_right = 0;
v_left2 = 0;
v_right2 = 0;

% Parámetros de la simulación
tiempo_simulacion = 50;% eliminar esto y los pasos de tiempo
paso_tiempo = 0.001;

% Parámetros del potencial de Lennard-Jones
epsilon = 600; % intesidad -> VALORES PROBADOS ANTERIORMENTE : 400; 220.0; 120  
sigma = 0.25; % Alcanse -> VALORES PROBADOS ANTERIORMENTE : 0.2; 0.3  alcanse

num_particulas = 2;

%--------------------------------------------------------------------------
% Inicialización de controlador PID
kpO = 1.0; kiO = 0.0; kdO = 0;
%kpO = 1; kiO = 0.0; kdO = 0;
eO_1 = 0; eP = 10; EO = 0;
eP2 = 10; EO2 = 0;

% Acercamiento exponencial:
v0 = 190;
alpha = 100;

% Funciones de conversión entre rpm y rad/s para cambiar entre las
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);

%------------------------------------
% SELECCIÓN DE MODO (PUNTO VS TRAYECTORIA)
modo = 2; %'Seleccionar modo 1 para punto, 2 para trayectoria'

if modo == 2
    % Meta para el robot 1 (solo un punto)
    trayectoria = [1.3, 0;
                   1.3, -1
                   1.3, -1.2];
elseif modo == 2
    % Definir una trayectoria de varios puntos
% Arreglo original desplazado 0.6 unidades hacia el eje x positivo Y -2 en
% y
% trayectoria = [
%    -0.4000  -1.2000;
%    -0.4052  -1.1358;
%    -0.4206  -1.0733;
%    -0.4458  -1.0141;
%    -0.4802  -0.9597;
%    -0.5229  -0.9115;
%    -0.5728  -0.8708;
%    -0.6285  -0.8386;
%    -0.6887  -0.8158;
%    -0.7518  -0.8029;
%    -0.8161  -0.8003;
%    -0.8800  -0.8081;
%    -0.9418  -0.8260;
%    -1.0000  -0.8536;
%    -1.0530  -0.8902;
%    -1.0994  -0.9348;
%    -1.1381  -0.9862;
%    -1.1680  -1.0432;
%    -1.1884  -1.1043;
%    -1.1987  -1.1678;
%    -1.1987  -1.2322;
%    -1.1884  -1.2957;
%    -1.1680  -1.3568;
%    -1.1381  -1.4138;
%    -1.0994  -1.4652;
%    -1.0530  -1.5098;
%    -1.0000  -1.5464;
%    -0.9418  -1.5740;
%    -0.8800  -1.5919;
%    -0.8161  -1.5997;
%    -0.7518  -1.5971;
%    -0.6887  -1.5842;
%    -0.6285  -1.5614;
%    -0.5728  -1.5292;
%    -0.5229  -1.4885;
%    -0.4802  -1.4403;
%    -0.4458  -1.3859;
%    -0.4206  -1.3267;
%    -0.4052  -1.2642;
%    -0.4000  -1.2000
% ];


% trayectoria = [
%     0.7707-1.6    1.2752-0.2;
%     0.6958-1.6    1.2923-0.2;
%     0.6192-1.6    1.2997-0.2;
%     0.5424-1.6    1.2972-0.2;
%     0.4665-1.6    1.2850-0.2;
%     0.3928-1.6    1.2631-0.2;
%     0.3225-1.6    1.2320-0.2;
%     0.2567-1.6    1.1921-0.2;
%     0.1966-1.6    1.1442-0.2;
%     0.1431-1.6    1.0889-0.2;
%     0.0971-1.6    1.0273-0.2;
%     0.0594-1.6    0.9603-0.2;
%     0.0306-1.6    0.8891-0.2;
%     0.0111-1.6    0.8147-0.2;
%     0.0012-1.6    0.7384-0.2;
%     0.0012-1.6    0.6616-0.2;
%     0.0111-1.6    0.5853-0.2;
%     0.0306-1.6    0.5109-0.2;
%     0.0594-1.6    0.4397-0.2;
%     0.0971-1.6    0.3727-0.2;
%     0.1431-1.6    0.3111-0.2;
%     0.1966-1.6    0.2558-0.2;
%     0.2567-1.6    0.2079-0.2;
%     0.3225-1.6    0.1680-0.2;
%     0.3928-1.6    0.1369-0.2;
%     0.4665-1.6    0.1150-0.2;
%     0.5424-1.6    0.1028-0.2;
%     0.6192-1.6    0.1003-0.2;
%     0.6958-1.6    0.1077-0.2;
%     0.7707-1.6    0.1248-0.2;
%     0.8429-1.6    0.1514-0.2;
%     0.9110-1.6    0.1869-0.2;
%     0.9741-1.6    0.2309-0.2;
%     1.0310-1.6    0.2826-0.2;
%     1.0808-1.6    0.3411-0.2;
%     1.1228-1.6    0.4056-0.2;
%     1.1562-1.6    0.4748-0.2;
%     1.1804-1.6    0.5478-0.2;
%     1.1951-1.6    0.6233-0.2;
%     1.2000-1.6    0.7000-0.2;
%     1.2000-1.6    0.7000-0.2;
%     1.1951-1.6    0.7767-0.2;
%     1.1804-1.6    0.8522-0.2;
%     1.1562-1.6    0.9252-0.2;
%     1.1228-1.6    0.9944-0.2;
%     1.0808-1.6    1.0589-0.2;
%     1.0310-1.6    1.1174-0.2;
%     0.9741-1.6    1.1691-0.2;
%     0.9110-1.6    1.2131-0.2;
%     0.8429-1.6    1.2486-0.2;
% ];
end
iteraciones = length(trayectoria);

n = 2;
n2 = 1;

% Inicialización de la matriz de posiciones
posiciones_robot1 = []; % Matriz para guardar las posiciones [x, y] del robot 1
posiciones_robot2 = []; % Matriz para guardar las posiciones [x, y] del robot 2

%--------------------------------------------------------------------------

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

%posiciones_ant = [posx1,posy1; posx2, posy2; posx3, posy3; posx4, posy4];  % Almacena posiciones anteriores para el algoritmo de Verlet
posiciones_ant = [posx1,posy1; posx2, posy2];  % Almacena posiciones anteriores para el algoritmo de Verlet

num_pasos = tiempo_simulacion / paso_tiempo;
fuerzas = zeros(num_particulas, 2);  % Inicializar la matriz de fuerzas

%% Ciclo principal 
% Calcular fuerzas entre partículas utilizando el potencial de Lennard-Jones
for paso = 1:num_pasos
    
    fuerzas = zeros(num_particulas, 2);  % Inicializar la matriz de fuerzas
    % ---------------------------------------------------------------------
    for i = 2:num_particulas
            
                % Calcula el vector entre las partículas i y j
                r_ij = posiciones_ant(i, :) - posiciones_ant(i-1, :);
                r = norm(r_ij);  % Magnitud del vector (distancia entre las partículas)

                % Calcular la fuerza usando el potencial de Lennard-Jones
                fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;               
                
                % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
                fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
                %fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;            
    end
    % ---------------------------------------------------------------------
    % Calcular aceleraciones usando F = ma
    aceleraciones = fuerzas / masa_robot;
    
    % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    
    % Eliminar velocidad negativa:
    if velocidades(2,2) < 0
        velocidades(2,2) = 0;
    end
    if velocidades(2,1) < 0
        velocidades(2,1) = 0;
    end
    
    % ---------------------------------------------------------------------
    % Para robot 1, usar un solo punto o una trayectoria
    if n > iteraciones
        n = 1;
    else
        x_meta = trayectoria(n,:)
        xg = x_meta(1);
        yg = x_meta(2);
    end
    % -------------------------------------------
    % Meta del robot 2 es la posición del robot 1
    if n2 < iteraciones
        xg2 = trayectoria(n2,1);
        yg2 = trayectoria(n2,2);
    else
        n2 = 1;
    end
    % -------------------------------------------
    % Controlador Robot 1
    e = [xg - posx1; yg - posy1];
    thetag = atan2d(e(2), e(1));
    eP = norm(e);
    eO = thetag - theta1;
    eO = atan2(sind(eO), cosd(eO));
    
    % Control de velocidad lineal
    kP = v0 * (1 - exp(-alpha*eP^2)) / eP;
    v = kP * eP;

    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO * eO + kiO * EO + kdO * eO_D;
    eO_1 = eO;
    % ---------------------------------------------
    % Controlador Robot 2
    e2 = [xg2 - posx2; yg2 - posy2];
    thetag2 = atan2d(e2(2), e2(1));
    eP2 = norm(e2);
    eO2 = thetag2 - theta2;
    eO2 = atan2(sind(eO2), cosd(eO2));
    eO_1 = eO2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_1;
    EO2 = EO2 + eO2;
    w2 = kpO * eO2 + kiO * EO2 + kdO * eO_D2;
    
    % ---------------------------------------------
    % ---------------------------------------------------------------------
    % Detener robot si llega a su meta
    if eP < 0.15
        n= n+1;
        w= 0;
        if n > iteraciones
          v = 0;
          w =0;
        end
    end

    % -------------------------------------------
    if eP2 < 0.15
        n2= n2+1;
        w=0;
    end

    error = [posx1 - posx2, posy1 - posy2];
    error2 = norm(error);
    if error2 < 0.25 || n2 > iteraciones
        w2 =0;
    end 

    % -------------------------------------------------
    % Velocidades de las ruedas - Robot1:
    % -------------------------------------------------
    v_left = ((v - (wheel_distance*1000)* w) / (wheel_radius*1000));
    v_right = ((v + (wheel_distance*1000) * w) / (wheel_radius*1000));
    
    % Convertir de rad/s a rpm
    v_left = rads2rpm(v_left)
    v_right = rads2rpm(v_right)
    
    % -------------------------------------------------
    % Velocidades de las ruedas - Robot2:
    % -------------------------------------------------
    % Calcular la velocidad lineal para el robot 2 usando el algoritmo Lennard-Jones
    v2 = norm(velocidades(2,1) + velocidades(2,2)); % Velocidad lineal del robot 2
    
    % Limitar la velocidad lineal antes de calcular las ruedas
    v2_limitada = min(v2, velocidad_maxima_rpm * (2 * pi * wheel_radius) / 60); % Convertimos de RPM a m/s y limitamos

    % Calcular las velocidades de las ruedas del robot 2
    v_left2 = (v2_limitada - wheel_distance * w2) / wheel_radius;   % Velocidad de la rueda izquierda [rad/s]
    v_right2 = (v2_limitada + wheel_distance * w2) / wheel_radius;  % Velocidad de la rueda derecha [rad/s]
    
    
    % ROBOT2:
    % Convertir de rad/s a RPM
    v_left2_rpm = rads2rpm(v_left2)   % Velocidad izquierda en RPM
    v_right2_rpm = rads2rpm(v_right2) % Velocidad derecha en RPM

    % Enviar velocidades a los robots
    robotat_3pi_set_wheel_velocities(robot1, v_left, v_right);
    %robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);
    robotat_3pi_set_wheel_velocities(robot2, v_left2_rpm, v_right2_rpm);
    
    % Obtener la posición actual del ROBOT1 (fijo)
    xi1 = robotat_get_pose(robotat, robot1_no, 'eulzyx');
    posx1 = xi1(1); % en m
    posy1 = xi1(2); % en m
    theta1 = atan2d(sind(xi1(4) - offset1), cosd(xi1(4) - offset1));
    
    % Guardar las posiciones del robot 1 en el arreglo
    posiciones_robot1 = [posiciones_robot1; posx1, posy1];
    
    % Obtener la posición actual del ROBOT2 (móvil)
    xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
    posx2 = xi2(1); % en m
    posy2 = xi2(2); % en m
    theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));
    
    % Guardar las posiciones del robot 2 en el arreglo
    posiciones_robot2 = [posiciones_robot2; posx2, posy2];

    %posiciones_ant = [posx1,posy1; posx2, posy2; posx3, posy3; posx4, posy4];  % Almacena posiciones anteriores para el algoritmo de Verlet
    posiciones_ant = [posx1,posy1; posx2, posy2];  % Almacena posiciones anteriores para el algoritmo de Verlet


    pause(0.0001);
end
