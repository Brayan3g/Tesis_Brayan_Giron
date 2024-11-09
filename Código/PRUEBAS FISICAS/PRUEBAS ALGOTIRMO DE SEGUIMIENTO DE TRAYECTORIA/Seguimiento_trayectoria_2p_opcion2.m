%% Conexión al Robotat 
clear;
clc;
robotat = robotat_connect()

% Offsets de los markers
marker_offsets = [178, 39, -3, -48, -90, -54, -86, 9, 0, -130]; 

%% CONEXIÓN POLOLU #1 
robot1_no = 6;
robot1 = robotat_3pi_connect(robot1_no)
offset1 = marker_offsets(robot1_no);

%% CONEXIÓN POLOLU #2 
robot2_no = 7;
robot2 = robotat_3pi_connect(robot2_no)
offset2 = marker_offsets(robot2_no);

%% Ejemplo de envío de velocidades al robot
% Las velocidades se envían en rpm, siendo el máximo permitido 400 rpm
robotat_3pi_set_wheel_velocities(robot1, -60, 60);
robotat_3pi_set_wheel_velocities(robot2, -60, 60);

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot1)
robotat_3pi_force_stop(robot2)


%% CONTROL DE 2 POLOLU - seguimiento de 1 a otro
% Dimensiones del robot
wheel_radius = 32;
wheel_distance = 96 - 2*6.8;

% Inicialización de variables para las velocidades
v_left = 0; v_right = 0; v_left2 = 0; v_right2 = 0;

% Inicialización de controlador PID
kpO = 1; kiO = 0.0; kdO = 0;
eO_1 = 0; eP = 10; EO = 0;
eP2 = 10; EO2 = 0;

% Acercamiento exponencial:
v0 = 175;
v02 = 200;
alpha = 100;

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
%     trayectoria = [;
%        -0.25, 1.2;
%        -0.5, 1.2;
%        -0.75, 1.2;
%        -1, 0.8;
%        -1, 0.4;
%        -1, 0;
%        -1, -0.75;
%        -1, -1.3;
%        -0.7, -1.75;
%        -0.5, -1.75;
%        -0.25, -1.75;
%         0, -1.75;
%         0.5, -1.75;
%         0.7, -1.75;
%         1.25, -0.75;
%         1.25, 0;
%         1.25, 0.6;
%         1.25, 1;
%         0.75, 1.2;
%         0.5, 1.2;
%         0, 1.2];
% trayectoria = [
%          0    1.2000;
%    -0.0833    1.2000;
%    -0.1667    1.2000;
%    -0.2500    1.2000;
%    -0.3333    1.2000;
%    -0.5000    1.0000;
%    -0.5000    0.8000;
%    -0.5000    0.6000;
%    -0.5000    0.4000;
%    -0.5000    0.2000;
%    -0.5000         0;
%    -0.5000   -0.2000;
%    -0.5000   -0.4000;
%    -0.5000   -0.6000;
%    -0.5000   -0.8000;
%    -0.4167   -1.2000;
%    -0.3333   -1.2000;
%    -0.2500   -1.2000;
%    -0.1667   -1.2000;
%    -0.0833   -1.2000;
%          0   -1.2000;
%     0.0833   -1.2000;
%     0.1667   -1.2000;
%     0.2500   -1.2000;
%     0.3333   -1.2000;
%     0.5000   -1.0000;
%     0.5000   -0.8000;
%     0.5000   -0.6000;
%     0.5000   -0.4000;
%     0.5000   -0.2000;
%     0.5000         0;
%     0.5000    0.2000;
%     0.5000    0.4000;
%     0.5000    0.6000;
%     0.5000    0.8000;
%     0.4167    1.2000;
%     0.3333    1.2000;
%     0.2500    1.2000;
%     0.1667    1.2000;
%     0.0833    1.2000;
% ];
% 
% trayectoria = [
%     0.5000    1.2000;
%     0.4167    1.2000;
%     0.3333    1.2000;
%     0.2500    1.2000;
%     0.1667    1.2000;
%     0.0000    0.8000;
%     0.0000    0.6000;
%     0.0000    0.4000;
%     0.0000    0.2000;
%     0.0000         0;
%     0.0000   -0.2000;
%     0.0000   -0.4000;
%     0.0000   -0.6000;
%     0.1667   -1.2000;
%     0.2500   -1.2000;
%     0.3333   -1.2000;
%     0.4167   -1.2000;
%     0.5000   -1.2000;
%     0.5833   -1.2000;
%     0.6667   -1.2000;
%     0.7500   -1.2000;
%     0.8333   -1.2000;
%     1.0000   -0.8000;
%     1.0000   -0.6000;
%     1.0000   -0.4000;
%     1.0000   -0.2000;
%     1.0000         0;
%     1.0000    0.2000;
%     1.0000    0.4000;
%     1.0000    0.6000;
%     1.0000    0.8000;
%     0.8333    1.2000;
%     0.7500    1.2000;
%     0.6667    1.2000;
%     0.5833    1.2000;
% ];

trayectoria = [
    1.0, 1.2;
    0.8, 1.2;
    0.6, 1.2;
    0.4, 1.2;
    0.2, 1.2;
    0.0, 1.2;
   -0.2, 1.2;
   -0.4, 1.2;
   -0.6, 1.2;
   -0.8, 1.2;
   -1.0, 1.2;
   -1.0, 1.08;
   -1.0, 0.96;
   -1.0, 0.84;
   -1.0, 0.72;
   -1.0, 0.6;
   -1.0, 0.48;
   -1.0, 0.36;
   -1.0, 0.24;
   -1.0, 0.12;
   -1.0, 0.0;
   -1.0, -0.12;
   -1.0, -0.24;
   -1.0, -0.36;
   -1.0, -0.48;
   -1.0, -0.6;
   -1.0, -0.72;
   -1.0, -0.84;
   -1.0, -0.96;
   -1.0, -1.08;
   -1.0, -1.2;
   -0.8, -1.2;
   -0.6, -1.2;
   -0.4, -1.2;
   -0.2, -1.2;
    0.0, -1.2;
    0.2, -1.2;
    0.4, -1.2;
    0.6, -1.2;
    0.8, -1.2;
    1.0, -1.2;
    1.0, -1.08;
    1.0, -0.96;
    1.0, -0.84;
    1.0, -0.72;
    1.0, -0.6;
    1.0, -0.48;
    1.0, -0.36;
    1.0, -0.24;
    1.0, -0.12;
    1.0, 0.0;
    1.0, 0.12;
    1.0, 0.24;
    1.0, 0.36;
    1.0, 0.48;
    1.0, 0.6;
    1.0, 0.72;
    1.0, 0.84;
    1.0, 0.96;
    1.0, 1.08
];

end
iteraciones = length(trayectoria);

n = 1;
n2 =1;
%punto_actual = 1; % Indica en qué punto de la trayectoria se encuentra

% Inicialización de la matriz de posiciones
posiciones_robot1 = []; % Matriz para guardar las posiciones [x, y] del robot 1
posiciones_robot2 = []; % Matriz para guardar las posiciones [x, y] del robot 2
posiciones_robot3 = []; % Matriz para guardar las posiciones [x, y] del robot 3 (si tienes más robots)


%% Ciclo de simulación
while 1     
    % Posición del robot 1
    xi = robotat_get_pose(robotat, robot1_no, 'eulzyx');
    posx = xi(1)
    posy = xi(2)
    theta = atan2d(sind(xi(4) - offset1), cosd(xi(4) - offset1));

    % Guardar las posiciones del robot 1 en el arreglo
    posiciones_robot1 = [posiciones_robot1; posx, posy];
    

    % Posición del robot 2
    xi2 = robotat_get_pose(robotat, robot2_no, 'eulzyx');
    posx2 = xi2(1)
    posy2 = xi2(2)
    theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));
    
    % Guardar las posiciones del robot 2 en el arreglo
    posiciones_robot2 = [posiciones_robot2; posx2, posy2];


    % Para robot 1, usar un solo punto o una trayectoria
%     xg = trayectoria(punto_actual, 1);
%     yg = trayectoria(punto_actual, 2);
    if n > iteraciones
        n=1;
%         xg = x_meta(1);
%         yg = x_meta(2);
    else
        x_meta = trayectoria(n,:)
        xg = x_meta(1);
        yg = x_meta(2);
    end
    
%     x_meta = trayectoria(n,:)
%     xg = x_meta(1)
%     yg = x_meta(2)

    % Meta del robot 2 es la posición del robot 1
    if n2 < iteraciones 
        x_meta2 = trayectoria(n2,:)
        xg2 = x_meta2(1);
        yg2 = x_meta2(2);
    else 
        n2=1;
    end
%     xg2 = xg;
%     yg2 = yg;

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

    % Controlador Robot 2
    e2 = [xg2 - posx2; yg2 - posy2];
    thetag2 = atan2d(e2(2), e2(1));
    eP2 = norm(e2);
    eO2 = thetag2 - theta2;
    eO2 = atan2(sind(eO2), cosd(eO2));
    eO_1 = eO2;
    % Control de velocidad lineal
    kP2 = v02 * (1 - exp(-alpha*eP2^2)) / eP2;
    v2 = kP2 * eP2;
    
    % Control de velocidad angular
    eO_D2 = eO2 - eO_1;
    EO2 = EO2 + eO2;
    w2 = kpO * eO2 + kiO * EO2 + kdO * eO_D2;

    % Detener robot si llega a su meta
    if eP < 0.15
        n= n+1;
        if n > iteraciones
          v = 0;
          w =0;
        end
    end

%     if eP2 < 0.15
%         n2= n2+1;
%     end
% %     
%     if eP2 < 0.4 %0.25 + 0.15 
%         v2 = 0; w2 = 0;
%     end


    if eP2 < 0.15
        n2= n2+1;
%         if n2 > iteraciones
%           v2 = 0;
%           w2 =0;
%         end
    end
    
    error = [posx - posx2, posy - posy2];
    error2 = norm(error);

    if error2 < 0.25 || n2 > iteraciones
        v2 = 0;
        w2 =0;
    end

    % Mapeo de velocidades a las llantas del carrito: 
    % Se mapea del uniciclo de regreso al robot diferencial.
    % OJO: los resultados de estas fórmulas están en rad/s, DEBEN
    % cambiarse a rpm (usar la función auxiliar de conversión)
    v_left = (v - wheel_distance * w) / wheel_radius;
    v_right = (v + wheel_distance * w) / wheel_radius;
    % ROBOT2:
    v_left2 = (v2 - wheel_distance * w2) / wheel_radius;
    v_right2 = (v2 + wheel_distance * w2) / wheel_radius;

    % Convertir de rad/s a rpm
    v_left = rads2rpm(v_left);
    v_right = rads2rpm(v_right);
    % ROBOT2:
    v_left2 = rads2rpm(v_left2);
    v_right2 = rads2rpm(v_right2);

    % Enviar velocidades a los robots
    robotat_3pi_set_wheel_velocities(robot1, v_left, v_right);
    robotat_3pi_set_wheel_velocities(robot2, v_left2, v_right2);

    pause(0.0001);
end

%% Graficar las trayectorias formadas por las posiciones
figure;
hold on;

% Graficar trayectoria del robot 1
plot(posiciones_robot1(:, 1), posiciones_robot1(:, 2), '-o', 'DisplayName', 'Robot 1', 'LineWidth', 2);

% Graficar trayectoria del robot 2
plot(posiciones_robot2(:, 1), posiciones_robot2(:, 2), '-x', 'DisplayName', 'Robot 2', 'LineWidth', 2);

% % Graficar trayectoria del robot 3 (si se usa)
% if ~isempty(posiciones_robot3)
%     plot(posiciones_robot3(:, 1), posiciones_robot3(:, 2), '-s', 'DisplayName', 'Robot 3', 'LineWidth', 2);
% end

% Añadir leyenda y etiquetas
legend show;
xlabel('Posición X');
ylabel('Posición Y');
title('Trayectorias de los robots');
grid on;
hold off;
