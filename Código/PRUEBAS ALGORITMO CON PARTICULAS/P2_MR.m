% BRAYAN GABRIEL GIRON GARCIA
% TESIS: Validación de algoritmos de física granular con énfasis en el
% estudio y análisis del tránsito fantasma en entornos reales a escala.
%_________________________________________________________________________


%filename = '2P_LJP.gif';          % Nombre del archivo
DelayTime = 0.1;                    % T. en seg. que dura cada plot en el GIF
f = figure(1);                      % Creación de la figura

% Parámetros de la simulación
tiempo_simulacion = 50; % Tiempo de simulación en segundos
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula
espacio = 2.0;           % Tamaño del espacio a utilizar en metros

% Algunos casos específicos 2D posiblemente interesantes
% Dos partículas a una distancia de 1 
% sin velocidades iniciales
num_particulas = 2;

posiciones = ...
    [0.5 1 0;...
     1.5 1 0];

velocidades = ...
    [0 0 0;...
     0 0 0];

aceleraciones = ...
    [0 0 0;...
     0 0 0];


% Matrices para almacenar las coordenadas de posición de cada partícula 
% a lo largo del tiempo
trayectorias_x_acum = ...
    zeros(num_particulas, tiempo_simulacion / paso_tiempo);
trayectorias_y_acum = ...
    zeros(num_particulas, tiempo_simulacion / paso_tiempo);
trayectorias_z_acum = ...
    zeros(num_particulas, tiempo_simulacion / paso_tiempo);

% Parámetros del potencial de Lennard-Jones
epsilon = 1.0; % Parámetro epsilon (intensidad)
sigma = 1.0;   % Parámetro sigma (Alcance)

% Bucle de simulación con el algoritmo de Verlet
num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

%%
%Distancia inicial 1
% distancia max  paso = 1:115
% distancia min  paso = 1:200
% Simulacion completa 1:num_pasos

for paso = 1:5000
    % Calcular fuerzas entre partículas    
    fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
    
    % Calcular aceleraciones
    aceleraciones = fuerzas / masa_particula;
    
   % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo)
    vel_1 = velocidades(1)
    vel_2 = velocidades(2)
    % Actualizar posiciones utilizando el algoritmo de Verlet
    posiciones = ...
        posiciones + (velocidades * paso_tiempo) ...
        + (0.5 * aceleraciones * paso_tiempo^2)
    
     % Verificar colisiones con las paredes
    for i = 1:num_particulas
        for dim = 1:3
            if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end

    % Almacenar las coordenadas de posición de cada partícula 
    % en este paso de tiempo
    trayectorias_x_acum(:, paso) = posiciones(:, 1);
    trayectorias_y_acum(:, paso) = posiciones(:, 2);
    trayectorias_z_acum(:, paso) = posiciones(:, 3);
    
    % Actualizar posiciones y aceleraciones para el siguiente 
    % paso de tiempo
    posiciones_ant = posiciones;
%--------------------------------------------------------------------------    
    % Dibujar las partículas en la figura actualizada
    scatter3(posiciones(:, 1), ...
             posiciones(:, 2), ...
             posiciones(:, 3), ...
             30, 'filled','red' ,'MarkerFaceAlpha', 0.7);
    xlabel('X(m)');
    ylabel('Y(m)');
    %zlabel('Z');
    title('Dinámica molecular: potencial de Lennard-Jones');
    grid on;
    % Establece los límites del cubo
    axis([0, espacio, 0, espacio, 0, espacio]); 
    axis([0, espacio, 0, espacio]);     
    % Pausa para permitir que MATLAB actualice la figura
    pause(0.01);
end


% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
    num_particulas = size(posiciones, 1);
    dimension = size(posiciones, 2);
    fuerzas = zeros(num_particulas, dimension);
    
    for i = 1:num_particulas
        for j = i+1:num_particulas
            % Calcula el vector entre las partículas
            r_ij = posiciones(i, :) - posiciones(j, :);
            % Calcular la distancia entre particulas
            r = norm(r_ij);       
            
            % Calcular la fuerza 
            fuerza_ij = ...
                24*epsilon*( 2*(sigma/r)^12 - (sigma/r)^6 )*r_ij/r^2;
            
            % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
            fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
        end
    end
end
