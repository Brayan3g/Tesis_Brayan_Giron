% BRAYAN GABRIEL GIRON GARCIA
% TESIS: Validación de algoritmos de física granular con énfasis en el
% estudio y análisis del tránsito fantasma en entornos reales a escala.
%_________________________________________________________________________


% Simulación de Dinámica Molecular con Potencial de Lennard-Jones
% Este script simula la interacción entre varias partículas en un espacio 2D
% utilizando el algoritmo de Verlet y el potencial de Lennard-Jones.

clear;  % Limpia todas las variables del espacio de trabajo
clc;    % Limpia la ventana de comandos

DelayTime = 0.1;   % Tiempo en segundos que dura cada plot en el GIF (no se usa en este código)
f = figure(1);     % Creación de la figura para visualizar la simulación

% Parámetros de la simulación
tiempo_simulacion = 50;  % Tiempo total de simulación en segundos
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula (asumida igual para todas)
espacio = 2;             % Tamaño del espacio a utilizar en ambas dimensiones (de -espacio a espacio)

% Parámetros del potencial de Lennard-Jones
epsilon = 0.5;  % Intensidad del potencial
sigma = 0.5;    % Alcance del potencial (distancia a la que la energía potencial es mínima)
usar_equilibrio = true;  % Opción para colocar las partículas en su distancia de equilibrio
tiempo_movimiento = 3;  % Tiempo después del cual la partícula 1 comenzará a moverse
velocidad_lenta = 0.5;  % Velocidad lenta de la partícula 1 en metros/segundo
posicion_final = -1.5;    % Posición final en metros en la que la partícula 1 se detendrá

% Configuración inicial de las partículas
num_particulas = 4;  % Número de partículas en la simulación (cambiar este valor para más partículas)
posicion_inicial = [0, 0];  % Posición inicial de la primera partícula

% Opción de alineación: 'horizontal' o 'vertical'
alineacion = 'vertical';  % Cambiar a 'vertical' para alinear las partículas en el eje Y

% Determinar la distancia inicial entre partículas
if usar_equilibrio
    distancia_inicial = 2^(1/6) * sigma;  % Distancia de equilibrio donde la fuerza neta es cero
else
    distancia_inicial = 0.5;  % Distancia fija entre partículas (se puede ajustar)
end

% Generación automática de posiciones iniciales de las partículas
posiciones = zeros(num_particulas, 2);  % Inicializar la matriz de posiciones
posiciones(1, :) = posicion_inicial;  % Asignar la posición de la primera partícula

% Colocar las demás partículas a la misma distancia a partir de la primera
for i = 2:num_particulas
    if strcmp(alineacion, 'horizontal')
        posiciones(i, :) = posiciones(i-1, :) - [distancia_inicial, 0];  % Alineación en el eje X (horizontal)
    elseif strcmp(alineacion, 'vertical')
        posiciones(i, :) = posiciones(i-1, :) - [0, distancia_inicial];  % Alineación en el eje Y (vertical)
    end
end

% Matriz de velocidades iniciales (en metros/segundo)
velocidades = zeros(num_particulas, 2);  % Inicializar la matriz de velocidades a cero

% Matriz de aceleraciones iniciales (en metros/segundo^2)
aceleraciones = zeros(num_particulas, 2);  % Inicializar la matriz de aceleraciones a cero

% Bucle de simulación utilizando el algoritmo de Verlet
num_pasos = tiempo_simulacion / paso_tiempo;  % Número total de pasos de simulación
posiciones_ant = posiciones;  % Almacena posiciones anteriores para el algoritmo de Verlet

for paso = 1:num_pasos
    tiempo_actual = paso * paso_tiempo;  % Calcular el tiempo actual de la simulación

    % Activar el movimiento de la partícula 1 después de `tiempo_movimiento`
    if usar_equilibrio && tiempo_actual > tiempo_movimiento
        if (posicion_final > 0 && posiciones(1, 1) < posicion_final) || ...
           (posicion_final < 0 && posiciones(1, 1) > posicion_final)
            velocidades(1, 1) = velocidad_lenta * sign(posicion_final);  % Asignar la velocidad en la dirección correcta
        else
            velocidades(1, 1) = 0;  % Detener la partícula 1 si ha alcanzado la posición final
        end
    end

    % Calcular fuerzas entre partículas utilizando el potencial de Lennard-Jones
    fuerzas = zeros(num_particulas, 2);  % Inicializar la matriz de fuerzas

    % Calcular la fuerza entre cada par de partículas
    for i = 1:num_particulas
        for j = i+1:num_particulas
            % Calcula el vector entre las partículas i y j
            r_ij = posiciones_ant(i, :) - posiciones_ant(j, :);
            r = norm(r_ij);  % Magnitud del vector (distancia entre las partículas)

            % Calcular la fuerza usando el potencial de Lennard-Jones
            fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
            
            % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
            fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
        end
    end

    % Calcular aceleraciones usando F = ma
    aceleraciones = fuerzas / masa_particula;
    
    % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    
    % Actualizar posiciones utilizando el algoritmo de Verlet
    posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
    
    % Verificar colisiones con las paredes considerando límites negativos
    % Invertir la velocidad de la partícula si choca con una pared
    for i = 1:num_particulas
        for dim = 1:2
            if posiciones(i, dim) < -espacio || posiciones(i, dim) > espacio
                velocidades(i, dim) = -velocidades(i, dim);  % Invertir la velocidad
            end
        end
    end
    
    % Actualizar posiciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;

    % Dibujar las partículas en la figura actualizada con nuevos límites
    scatter(posiciones(:, 1), posiciones(:, 2), 30, 'filled', 'red', 'MarkerFaceAlpha', 0.7);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Dinámica molecular: potencial de Lennard-Jones');
    grid on;
    axis([-espacio, espacio, -espacio, espacio]);  % Ajustar los límites de la visualización
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(0.01);
end
