% BRAYAN GABRIEL GIRON GARCIA
% TESIS: Validación de algoritmos de física granular con énfasis en el
% estudio y análisis del tránsito fantasma en entornos reales a escala.
%_________________________________________________________________________

%% Simulación de Dinámica Molecular con Potencial de Lennard-Jones en
% particulas.
clear;
clc;

DelayTime = 0.1;
f = figure(1);

% Parámetros de la simulación
tiempo_simulacion = 50;
paso_tiempo = 0.01;
masa_particula = 1.0;

% Tamaño del espacio en X y Y
espacio_x = 5;  % Tamaño del espacio en la dimensión X
espacio_y = 2.5;  % Tamaño del espacio en la dimensión Y

% Parámetros del potencial de Lennard-Jones
epsilon = 0.4;
sigma = 0.3;
usar_equilibrio = false;
tiempo_movimiento = 1;
velocidad_lenta = 0.5;

% Configuración de movimiento
modo_movimiento = 'punto'; % Seleccionamos si queremos usar una 'trayectoria' o un 'punto'
posicion_final = [4, -1.5];

% Configuración inicial de las partículas
num_particulas = 2;  % Dos partículas
posicion_fija = [0, -1.5];  % La primera partícula está fija

if usar_equilibrio
    distancia_inicial = 2^(1/6) * sigma;
else
    distancia_inicial = 0.5;
end

posiciones = zeros(num_particulas, 2);
posiciones(1, :) = posicion_fija;  % Primera partícula en posición fija
posiciones(2, :) = posicion_fija - [distancia_inicial, 0];  % Segunda partícula en una posición inicial

velocidades = zeros(num_particulas, 2);  % Velocidades de las partículas
aceleraciones = zeros(num_particulas, 2);  % Aceleraciones de las partículas

% Colores de las partículas
colores = [
    1, 0, 0;   % Rojo para la primera partícula fija
    0, 0, 1;   % Azul para la segunda partícula móvil
];

num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

for paso = 1:num_pasos
    tiempo_actual = paso * paso_tiempo;

    if usar_equilibrio && tiempo_actual > tiempo_movimiento
        if strcmp(modo_movimiento, 'punto')
            direccion = posicion_final - posiciones(2, :);
        end
        
        if norm(direccion) > 0
            direccion_normalizada = direccion / norm(direccion);
            velocidades(2, :) = velocidad_lenta * direccion_normalizada;
        else
            velocidades(2, :) = [0, 0];
        end
    end

    fuerzas = zeros(num_particulas, 2);

    % Calcular fuerzas solo para la segunda partícula
    r_ij = posiciones_ant(2, :) - posiciones_ant(1, :);
    r = norm(r_ij);

    fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;

    fuerzas(2, :) = fuerzas(2, :) + fuerza_ij;

    aceleraciones(2, :) = fuerzas(2, :) / masa_particula;
    velocidades(2, :) = velocidades(2, :) + (aceleraciones(2, :) * paso_tiempo);
    posiciones(2, :) = posiciones(2, :) + (velocidades(2, :) * paso_tiempo) + (0.5 * aceleraciones(2, :) * paso_tiempo^2);

    % Verificar colisiones con las paredes para la segunda partícula
    if posiciones(2, 1) < -espacio_x || posiciones(2, 1) > espacio_x
        velocidades(2, 1) = -velocidades(2, 1);
    end
    if posiciones(2, 2) < -espacio_y || posiciones(2, 2) > espacio_y
        velocidades(2, 2) = -velocidades(2, 2);
    end
    
    posiciones_ant = posiciones;

    % Dibujar las partículas
    scatter(posiciones(:, 1), posiciones(:, 2), 30, colores(1:num_particulas, :), 'filled', 'MarkerFaceAlpha', 0.7);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Dinámica molecular: potencial de Lennard-Jones');
    grid on;
    axis([-espacio_x, espacio_x, -espacio_y, espacio_y]);
    
    pause(0.01);
end
