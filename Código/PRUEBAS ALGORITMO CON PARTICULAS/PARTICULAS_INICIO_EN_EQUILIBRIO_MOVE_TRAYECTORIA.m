% BRAYAN GABRIEL GIRON GARCIA
% TESIS: Validación de algoritmos de física granular con énfasis en el
% estudio y análisis del tránsito fantasma en entornos reales a escala.
%_________________________________________________________________________


% Simulación de Dinámica Molecular con Potencial de Lennard-Jones

clear;
clc;

DelayTime = 0.1;
f = figure(1);

% Parámetros de la simulación
tiempo_simulacion = 50;
paso_tiempo = 0.01;
masa_particula = 1.0;

% Tamaño del espacio en X y Y
espacio_x = 2;  % Tamaño del espacio en la dimensión X
espacio_y = 2.5;  % Tamaño del espacio en la dimensión Y

% Parámetros del potencial de Lennard-Jones
epsilon = 0.4; % 0.4
sigma = 0.3; % 0.3
usar_equilibrio = true;
tiempo_movimiento = 1;
velocidad_lenta = 0.5; %0.5
 
% Configuración de movimiento
modo_movimiento = 'trayectoria' % Selecionamos si queremos usar una 'trayectoria' o un 'punto'
posicion_final = [4, -1.5];
trayectoria = [0.0000   -1.5000;
    0.4870   -1.4187;
    0.9213   -1.1837;
    1.2557   -0.8204;
    1.4541   -0.3682;
    1.4949    0.1239;
    1.3737    0.6025;
    1.1036    1.0159;
    0.7139    1.3192;
    0.2469    1.4795;
   -0.2469    1.4795;
   -0.7139    1.3192;
   -1.1036    1.0159;
   -1.3737    0.6025;
   -1.4949    0.1239;
   -1.4541   -0.3682;
   -1.2557   -0.8204;
   -0.9213   -1.1837;
   -0.4870   -1.4187;
   -0.0000   -1.5000];
indice_trayectoria = 1;

% Configuración inicial de las partículas
num_particulas = 4;  % Cambiar este valor según el número de partículas deseado
posicion_inicial = [0, -1.5];

alineacion = 'horizontal'; % 'vertical'  'horizontal'

if usar_equilibrio
    distancia_inicial = 2^(1/6) * sigma;
else
    distancia_inicial = 0.5;
end

posiciones = zeros(num_particulas, 2);
posiciones(1, :) = posicion_inicial;

for i = 2:num_particulas
    if strcmp(alineacion, 'horizontal')
        posiciones(i, :) = posiciones(i-1, :) - [distancia_inicial, 0];
    elseif strcmp(alineacion, 'vertical')
        posiciones(i, :) = posiciones(i-1, :) - [0, distancia_inicial];
    end
end

velocidades = zeros(num_particulas, 2);
aceleraciones = zeros(num_particulas, 2);

% Colores de las partículas
colores = [
    1, 0, 0;   % Rojo (Primario)
    1, 1, 0;   % Amarillo (Secundario)
    0, 0, 1;   % Azul (Primario)
    0, 1, 0;   % Verde (Primario)
    0.5, 0, 0.5; % Púrpura (Terciario)
    1, 0.5, 0; % Naranja (Terciario)
    1, 0, 1;   % Magenta (Secundario)
    0, 1, 1;   % Cian (Secundario)
    0.5, 0.5, 0; % Verde Oliva (Terciario)
];

num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

for paso = 1:num_pasos
    tiempo_actual = paso * paso_tiempo;

    if usar_equilibrio && tiempo_actual > tiempo_movimiento
        if strcmp(modo_movimiento, 'punto')
            direccion = posicion_final - posiciones(1, :);
        elseif strcmp(modo_movimiento, 'trayectoria')
            direccion = trayectoria(indice_trayectoria, :) - posiciones(1, :);
            distancia = norm(direccion);
            
            if distancia < 0.01 && indice_trayectoria < size(trayectoria, 1)
                indice_trayectoria = indice_trayectoria + 1;
            elseif distancia < 0.01 && indice_trayectoria == size(trayectoria, 1)
                velocidades(1, :) = [0, 0];
                direccion = [0, 0];
            end
        end
        
        if norm(direccion) > 0
            direccion_normalizada = direccion / norm(direccion);
            velocidades(1, :) = velocidad_lenta * direccion_normalizada;
        else
            velocidades(1, :) = [0, 0];
        end
    end

    fuerzas = zeros(num_particulas, 2);

    for i = 1:num_particulas
        for j = i+1:num_particulas
            r_ij = posiciones_ant(i, :) - posiciones_ant(j, :);
            r = norm(r_ij);

            fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
            
            fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
        end
    end

    aceleraciones = fuerzas / masa_particula;
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);

    % Verificar colisiones con las paredes para espacios X y Y diferentes
    for i = 1:num_particulas
        if posiciones(i, 1) < -espacio_x || posiciones(i, 1) > espacio_x
            velocidades(i, 1) = -velocidades(i, 1);
        end
        if posiciones(i, 2) < -espacio_y || posiciones(i, 2) > espacio_y
            velocidades(i, 2) = -velocidades(i, 2);
        end
    end
    
    posiciones_ant = posiciones;

    % Dibujar las partículas con diferentes colores
    scatter(posiciones(:, 1), posiciones(:, 2), 30, colores(1:num_particulas, :), 'filled', 'MarkerFaceAlpha', 0.7);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Dinámica molecular: potencial de Lennard-Jones');
    grid on;
    axis([-espacio_x, espacio_x, -espacio_y, espacio_y]);
    
    pause(0.01);
end
