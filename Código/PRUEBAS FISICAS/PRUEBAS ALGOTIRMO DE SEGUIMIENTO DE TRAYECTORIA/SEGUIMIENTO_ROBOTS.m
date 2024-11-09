% =========================================================================
% Brayan Gabriel Girón
% PRUEBA DE ALINEACION Y SEGUIMIENTO 
% =========================================================================

%% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect()

%% CONEXIÓN POLOLU #1 
robot1_no = 2; % Seleccionar el agente específico a emplear

% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 

% Se establece la conexión con el robot y se extrae el offset respectivo
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);


%% CONEXIÓN POLOLU #2 
robot2_no = 5; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot y se extrae el offset respectivo
robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);


%% Ejemplo de envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -60, 60);
robotat_3pi_set_wheel_velocities(robot2, -60, 60);

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)


%% CONTROL DE 3 POLOLU - ALINEACION  EN EJE VERTICAL

%__________________________________________________________________________
% Varibale para cambiar de controlador:
% 1 -> Control PID con acercamiento exponencial
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



% Inicialización de variables para el punto de meta:
% Robot1:
xg = -1 ;
yg = 0 ;
thetag = 0;
% Robot2:
xg2 = -1 ;
yg2 = -0.5 ;
thetag2 = 0;

% Robot3:
xg3 = -1 ;
yg3 = -1.0 ;
thetag3 = 0;

%--------------------------------------------------------------------------
% Variables de controlador PID con acercamiento exponencial
% PID orientación:
kpO = 1;
kiO = 0.0; 
kdO = 0;

EO = 0;
eO_1 = 0;
eP = 10;

eP2 = 10;
EO2 = 0;

% Acercamiento exponencial:
v0 = 200; %400
alpha = 100;

%--------------------------------------------------------------------------
% Funciones de conversión entre rpm y rad/s para cambiar entre las
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);
%__________________________________________________________________________
% Ciclo de simulación
%__________________________________________________________________________

while abs(eP) > 0.15 | abs(eP2) > 0.15     
  %Condiciones de paro: Se verifica si el robot esta lo suficientemente cerca
  %y se detiene para que no colisione con el marcador.
  %------------------------------------------------------------------------
  % Se obtiene la posición y orientación actual del ROBOT1:
  xi = robotat_get_pose(robotat, robot1_no, 'eulzyx');
  posx = xi(1) % en mm
  posy = xi(2) % en mm
  theta = atan2d(sind(xi(4) - offset1), cosd(xi(4) - offset1));
  %------------------------------------------------------------------------
  % Se obtiene la posición y orientación actual del ROBOT2:
  xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
  posx2 = xi2(1) % en mm
  posy2 = xi2(2) % en mm
  theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));
 
  %------------------------------------------------------------------------
  % Se calcula el controlador
  %------------------------------------------------------------------------
    % ROBOT1
    %--------------------------------------     
    x = posx; y = posy;
    e = [xg - x; yg - y];
    thetag = atan2d(e(2), e(1));
    
    eP = norm(e)
    eO = thetag - theta;
    eO = atan2(sind(eO), cosd(eO))
    
    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP *eP;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    %--------------------------------------
    % ROBOT2
    %--------------------------------------     
    x2 = posx2; y2 = posy2;
    e2 = [xg2 - x2; yg2 - y2];
    thetag2 = atan2d(e2(2), e2(1));
    
    eP2 = norm(e2)
    eO2 = thetag2 - theta2;
    eO2 = atan2(sind(eO2), cosd(eO2))
    
    % Control de velocidad lineal
    kP2 = v0 * (1-exp(-alpha*eP2^2)) / eP2;
    v2 = kP2 *eP2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_1;
    EO2 = EO2 + eO2;
    w2 = kpO*eO2 + kiO*EO2 + kdO*eO_D2;
    eO_1 = eO2;
    %--------------------------------------
    
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Condiciones de paro: Se verifica si el robot esta lo suficientemente cerca
%y se detiene para que no colisione con el marcador.
  if  eP <0.15
      v = 0;

  end
  if  eP2 <0.15
      v2 = 0;
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

%__________________________________________________________________________  
% Cambio de velocidades de radianes a rpm: 
  % ROBOT1:
  v_left = rads2rpm(v_left);
  v_right = rads2rpm(v_right); 
  % ROBOT2:
  v_left2 = rads2rpm(v_left2);
  v_right2 = rads2rpm(v_right2);
  
  
%------------------------------------------------------------  
% Se envían las velocidades a los motores de las ruedas:  
  robotat_3pi_set_wheel_velocities(robot1, v_left, v_right);
  robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);

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


%%









%% CONTROL DE 2 POLOLU - seguimiento de 1 a otro
%__________________________________________________________________________

% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect()

% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 


%% CONEXIÓN POLOLU #1 
robot1_no = 2; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot y se extrae el offset respectivo
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);

%% CONEXIÓN POLOLU #2 
robot2_no = 7; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot y se extrae el offset respectivo
robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);


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

% Inicialización de variables para el punto de meta:
% Robot1 (Solo el robot1 tiene una meta establecida, luego los otros robot siguen al de adelante):
xg = 1.3 ;
yg = -1 ;
thetag = 0;
% Robot2:
thetag2 = 0;
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

% Acercamiento exponencial:
v0 = 300; %400
alpha = 100;

%--------------------------------------------------------------------------
% Funciones de conversión entre rpm y rad/s para cambiar entre las
rads2rpm = @(x) x * ( 60 / (2*pi));
rpm2rads = @(x) x * ( 2*pi / 60);

%__________________________________________________________________________
% Ciclo de simulación
%__________________________________________________________________________

while abs(eP) > 0.15 | abs(eP2) > 0.25    
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
  % Se calcula el controlador
  %------------------------------------------------------------------------
    %--------------------------------------
    % ROBOT1
    %--------------------------------------     
    x = posx; y = posy;
    e = [xg - x; yg - y];
    thetag = atan2d(e(2), e(1));
    
    eP = norm(e);
    eO = thetag - theta;
    eO = atan2(sind(eO), cosd(eO));
    
    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP *eP;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
    %--------------------------------------
    % ROBOT2
    %--------------------------------------     
    x2 = posx2; y2 = posy2;
    e2 = [xg2 - x2; yg2 - y2];
    thetag2 = atan2d(e2(2), e2(1));
    
    eP2 = norm(e2);
    eO2 = thetag2 - theta2;
    eO2 = atan2(sind(eO2), cosd(eO2));
    
    % Control de velocidad lineal
    kP2 = v0 * (1-exp(-alpha*eP2^2)) / eP2;
    v2 = kP2 *eP2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_1;
    EO2 = EO2 + eO2;
    w2 = kpO*eO2 + kiO*EO2 + kdO*eO_D2;
    eO_1 = eO2;
    %--------------------------------------
   
%--------------------------------------------------------------------------
%Condiciones para detener los robots.
  if  eP <0.15
      v = 0;
      w = 0;
  end
  if  eP2 <0.25  % 0.25 es la distancia entre robots para que no colisionen
      v2 = 0;
      w2 = 0;
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

%__________________________________________________________________________  
% Cambio de velocidades de radianes a rpm: 
  % ROBOT1:
  v_left = rads2rpm(v_left);
  v_right = rads2rpm(v_right); 
  % ROBOT2:
  v_left2 = rads2rpm(v_left2);
  v_right2 = rads2rpm(v_right2);
  
%------------------------------------------------------------  
% Se envían las velocidades a los motores de las ruedas:  
  robotat_3pi_set_wheel_velocities(robot1, v_left, v_right);
  robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);

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

 El siguiente codigo se conecta y controla 2 robots, el programa le asigna
 un punto de meta al robot1 y este por medio del controlador llega a dicho
 punto, el robot2 sigue al robot 1 cuando esta a una distancia establecida
 del robot1. analizalo y encuentra posibles mejoras y optimizaciones.
 luego agrega una opcion para selecionar entre usar un punto y una
 trayectoria, la trayectoria estaria sera un arreglo con varios puntos
 x,y, los cuales seran la meta del robot1, cuando llegue al primer punto
 del arreglo, debera cambiar su meta al segundo punto, y asi sucesivamente
 hasta completar la trayectoria.