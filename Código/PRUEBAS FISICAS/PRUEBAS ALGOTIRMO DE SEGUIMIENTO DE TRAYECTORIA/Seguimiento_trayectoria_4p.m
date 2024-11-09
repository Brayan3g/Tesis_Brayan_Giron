% =========================================================================
% Brayan Gabriel Girón
% PRUEBA DE ALINEACION Y SEGUIMIENTO 
% =========================================================================

%% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect()

%% CONEXIÓN POLOLU #1 
robot1_no = 6; % Seleccionar el agente específico a emplear

% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 

% Se establece la conexión con el robot y se extrae el offset respectivo
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);


%% CONEXIÓN POLOLU #2 
robot2_no = 7; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot y se extrae el offset respectivo
robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);

% CONEXIÓN POLOLU #3 
robot3_no = 8; % Seleccionar el agente específico a emplear
% Se establece la conexión con el robot y se extrae el offset respectivo
robot3 = robotat_3pi_connect(robot3_no)
offset3 = marker_offsets(robot3_no);

% CONEXIÓN POLOLU #4 
robot4_no = 10; % Seleccionar el agente específico a emplear
% Se establece la conexión con el robot y se extrae el offset respectivo
robot4 = robotat_3pi_connect(robot4_no)
offset4 = marker_offsets(robot4_no);


%% Ejemplo de envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -60, 60);
robotat_3pi_set_wheel_velocities(robot2, -60, 60);
robotat_3pi_set_wheel_velocities(robot3, -60, 60);
robotat_3pi_set_wheel_velocities(robot4, -60, 60);

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)
robotat_3pi_force_stop(robot3)
robotat_3pi_force_stop(robot4)


%% _________________________________________________________________________
% Variables de los robots
%__________________________________________________________________________

% Dimensiones del robot
wheel_radius = 32; % mm
wheel_distance = 96 - 2*6.8; % mm

% Variables para las velocidades de las ruedas (en rpm <- OJO)
% Robot1:
v_left = 0;
v_right = 0;
% Robot2:
v_left2 = 0;
v_right2 = 0;
% Robot3:
v_left3 = 0;
v_right3 = 0;
% Robot4:
v_left4 = 0;
v_right4 = 0;

% Inicialización de variables para el punto de meta:
% Robot1 (Solo el robot1 tiene una meta establecida, luego los otros robot siguen al de adelante)
thetag = 0;
% Robot2:
thetag2 = 0;
% Robot3:
thetag3 = 0;
% Robot4:
thetag4 = 0;
%--------------------------------------------------------------------------
% Variables de controlador PID con acercamiento exponencial
% PID orientación (MISMAS VARIABLES PARA TODOS LOS ROBOTS):
kpO = 1;
kiO = 0.0; 
kdO = 0;

eO_1 = 0;
%-----------------

% Robot1:
eP = 10;
EO = 0;

% Robot2:
eP2 = 10;
EO2 = 0;

% Robot3:
eP3 = 10;
EO3 = 0;

% Robot4:
eP4 = 10;
EO4 = 0;

% Acercamiento exponencial:
v0 = 300; %400
alpha = 100;

%--------------------------------------------------------------------------
% Funciones de conversión entre rpm y rad/s para cambiar entre las
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);

% SELECCIÓN DE MODO (PUNTO VS TRAYECTORIA)
modo = 2; %'Seleccionar modo 1 para punto, 2 para trayectoria'

if modo == 1
    % Meta para el robot 1 (solo un punto)
    trayectoria = [1.3, -1];
elseif modo == 2
    % Definir una trayectoria de varios puntos
%     trayectoria = [0, 1.2;
%         -1, 1.2;
%         -1, 0;
%         -1, -1.5;
%         0, -1.5;
%         1, -1.5;
%         1, 0
%         1, 1.2]; % trayectoria cuadrada
    trayectoria = [;
       -0.25, 1.2;
       -0.5, 1.2;
       -0.75, 1.2;
       -1, 1;
       -1, 0.6;
       -1, 0;
       -1, -0.75;
       -1, -1.3;
       -0.7, -1.5
       -0.5, -1.5;
       -0.25, -1.5;
        0, -1.5;
        0.5, -1.5;
        0.7, -1.5;
        1, -0.75;
        1, 0;
        1, 0.6;
        1, 1;
        0.75, 1.2;
        0.5, 1.2;
        0, 1.2];

end

punto_actual = 1; % Indica en qué punto de la trayectoria se encuentra


%%__________________________________________________________________________
% Ciclo de simulación
%__________________________________________________________________________

while abs(eP) > 0.15 || abs(eP2) > 0.25  || abs(eP3) > 0.25 || abs(eP4) > 0.25   
  %Condiciones de paro: Se verifica si el robot esta lo suficientemente cerca
  %y se detiene para que no colisione con el marcador.
  %------------------------------------------------------------------------
  % Se obtiene la posición y orientación actual del ROBOT1:
  xi = robotat_get_pose(robotat, robot1_no, 'eulzyx');
  posx = xi(1) % en mm
  posy = xi(2) % en mm
  theta = atan2d(sind(xi(4) - offset1), cosd(xi(4) - offset1));
  %------------------------------------------------------------------------
  % se establecen el punto de meta del robot 2 como la posicion del robot 1
  xg2 = xi(1);
  yg2 = xi(2);

  % Se obtiene la posición y orientación actual del ROBOT2:
  xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
  posx2 = xi2(1) % en mm
  posy2 = xi2(2) % en mm
  theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));
  %------------------------------------------------------------------------

  % se establecen el punto de meta del robot 3 como la posicion del robot 1
  xg3 = xi2(1);
  yg3 = xi2(2);
  
  % Se obtiene la posición y orientación actual del ROBOT3:
  xi3 = robotat_get_pose(robotat, robot3_no, 'eulzyx');
  posx3 = xi3(1) % en mm
  posy3 = xi3(2) % en mm
  theta3 = atan2d(sind(xi3(4) - offset3), cosd(xi3(4) - offset3));
  %------------------------------------------------------------------------

  % se establecen el punto de meta del robot 4 como la posicion del robot 1
  xg4 = xi3(1);
  yg4 = xi3(2);
  
  % Se obtiene la posición y orientación actual del ROBOT3:
  xi4 = robotat_get_pose(robotat, robot4_no, 'eulzyx');
  posx4 = xi4(1) % en mm
  posy4 = xi4(2) % en mm
  theta4 = atan2d(sind(xi4(4) - offset4), cosd(xi4(4) - offset4));
  
  % Para robot 1, usar un solo punto o una trayectoria
  xg = trayectoria(punto_actual, 1);
  yg = trayectoria(punto_actual, 2);
  %------------------------------------------------------------------------
  % Se calcula el controlador
  %------------------------------------------------------------------------
    %--------------------------------------
    % ROBOT1
    %--------------------------------------     
    % Controlador Robot 1
    e = [xg - posx; yg - posy];
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
    
    %--------------------------------------
    % ROBOT2
    %--------------------------------------     
    % Controlador Robot 2
    e2 = [xg2 - posx2; yg2 - posy2];
    thetag2 = atan2d(e2(2), e2(1));
    eP2 = norm(e2);
    eO2 = thetag2 - theta2;
    eO2 = atan2(sind(eO2), cosd(eO2));
    
    % Control de velocidad lineal
    kP2 = v0 * (1 - exp(-alpha*eP2^2)) / eP2;
    v2 = kP2 * eP2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_1;
    EO2 = EO2 + eO2;
    w2 = kpO * eO2 + kiO * EO2 + kdO * eO_D2;

    %--------------------------------------
    % ROBOT3
    %--------------------------------------     
    % Controlador Robot 3
    e3 = [xg3 - posx3; yg3 - posy3];
    thetag3 = atan2d(e3(2), e3(1));
    eP3 = norm(e2);
    eO3 = thetag3 - theta3;
    eO3 = atan2(sind(eO3), cosd(eO3));
    
    % Control de velocidad lineal
    kP3 = v0 * (1 - exp(-alpha*eP3^2)) / eP3;
    v3 = kP3 * eP3;
    
    % Control de velocidad angular
    eO_D3 = eO3 - eO_1;
    EO3 = EO3 + eO3;
    w3 = kpO * eO3 + kiO * EO3 + kdO * eO_D3;

    
    %--------------------------------------
    % ROBOT4
    %--------------------------------------     
        % Controlador Robot 2
    e4 = [xg4 - posx4; yg4 - posy4];
    thetag4 = atan2d(e4(2), e4(1));
    eP4 = norm(e2);
    eO4 = thetag4 - theta4;
    eO4 = atan2(sind(eO4), cosd(eO4));
    
    % Control de velocidad lineal
    kP4 = v0 * (1 - exp(-alpha*eP4^2)) / eP4;
    v4 = kP4 * eP4;
    
    % Control de velocidad angular
    eO_D4 = eO4 - eO_1;
    EO4 = EO4 + eO4;
    w4 = kpO * eO4 + kiO * EO4 + kdO * eO_D4;

    %--------------------------------------    
%--------------------------------------------------------------------------
%Condiciones para detener los robots.
  if eP < 0.15
        
        % Si el robot 1 llega a su meta y hay más puntos, cambiar al siguiente
        if punto_actual < size(trayectoria, 1)
            punto_actual = punto_actual + 1;
        elseif punto_actual >= size(trayectoria,1)
            v = 0; w = 0;
        end
    end
  if  eP2 <0.25  % 0.25 es la distancia entre robots para que no colisionen
      v2 = 0;
      w2 = 0;
  end
  if  eP3 <0.25  % 0.25 es la distancia entre robots para que no colisionen
      v3 = 0;
      w3 = 0;
  end
  if  eP4 <0.25  % 0.25 es la distancia entre robots para que no colisionen
      v4 = 0;
      w4 = 0;
  end
            
%--------------------------------------------------------------------------

% Mapeo de velocidades a las llantas del carrito: 
% Se mapea del uniciclo de regreso al robot diferencial.
% OJO: los resultados de estas fórmulas están en rad/s, DEBEN
% cambiarse a rpm (usar la función auxiliar de conversión)
  % ROBOT1:
  v_left = (v - wheel_distance*w)/wheel_radius;
  v_right = (v + wheel_distance*w)/wheel_radius;
  % ROBOT2:
  v_left2 = (v2 - wheel_distance*w2)/wheel_radius;
  v_right2 = (v2 + wheel_distance*w2)/wheel_radius;
  % ROBOT3:
  v_left3 = (v3 - wheel_distance*w3)/wheel_radius;
  v_right3 = (v3 + wheel_distance*w3)/wheel_radius;
  % ROBOT4:
  v_left4 = (v4 - wheel_distance*w4)/wheel_radius;
  v_right4 = (v4 + wheel_distance*w4)/wheel_radius;


%__________________________________________________________________________  
% Cambio de velocidades de radianes a rpm: 
  % ROBOT1:
  v_left = rads2rpm(v_left);
  v_right = rads2rpm(v_right); 
  % ROBOT2:
  v_left2 = rads2rpm(v_left2);
  v_right2 = rads2rpm(v_right2);
  % ROBOT3:
  v_left3 = rads2rpm(v_left3);
  v_right3 = rads2rpm(v_right3);
  % ROBOT2:
  v_left4 = rads2rpm(v_left4);
  v_right4 = rads2rpm(v_right4);
      
%------------------------------------------------------------  
% Se envían las velocidades a los motores de las ruedas:  
  robotat_3pi_set_wheel_velocities(robot1, v_left, v_right);
  robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);
  robotat_3pi_set_wheel_velocities(robot3, v_left, v_right);
  robotat_3pi_set_wheel_velocities(robot4, v_left2, v_right2);

%------------------------------------------------------------
  pause(0.1)

end

%__________________________________________________________________________
%__________________________________________________________________________


%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Pololu 3Pi
robotat_3pi_disconnect(robot1);
robotat_3pi_disconnect(robot2);
%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)
robotat_3pi_force_stop(robot3)
robotat_3pi_force_stop(robot4)

