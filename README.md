# Validación de algoritmos de física granular con énfasis en el estudio y análisis del tránsito fantasma 👻 en entornos reales a escala


## Descripción del Proyecto

Este proyecto se centra en la validación de algoritmos de física granular para el estudio del fenómeno de "tránsito fantasma" en entornos a escala, utilizando agentes robóticos. Los algoritmos modelan las interacciones de atracción y repulsión entre partículas, simulando el efecto de "acordeón" característico de los atascos en el tráfico fantasma 👻. Se implementa el algoritmo de Lennard-Jones, adaptado para funcionar en un sistema bidimensional con robots móviles. Las pruebas incluyen simulaciones y experimentos físicos en la plataforma Robotat.


### Objetivos 
- Optimizar y validar algoritmos de física granular para el estudio del fenómeno de tránsito fantasma mediante agentes robóticos móviles, en entornos de simulación y físicos.
- Diseñar experimentos con simulaciones y robots móviles en escenarios que replican entornos de tráfico.
- Evaluar la capacidad del algoritmo para reproducir el efecto de tráfico fantasma.


## Alcance
El proyecto se limita a simulaciones bidimensionales en entornos controlados, simulados y físicos. Utiliza la plataforma Robotat, la cual permite realizar experimentos en un espacio de 3x5 metros, con un número limitado de robots. Las pruebas incluyen trayectorias lineales y circulares, observando el comportamiento de los robots ante fuerzas de atracción y repulsión.


## Algoritmo de Física Granular

Para emular el fenómeno de tráfico fantasma, se utiliza el **potencial de Lennard-Jones**, que define fuerzas de atracción y repulsión entre partículas. Este potencial permite que las partículas (o robots) mantengan una distancia de equilibrio mediante fuerzas atractivas a distancias mayores y repulsivas a distancias menores. Al modelar este comportamiento en robots móviles, se simula el efecto de "acordeón", donde los vehículos desaceleran y aceleran sin una causa aparente, reproduciendo el comportamiento de los atascos fantasma.

## Trayectorias Utilizadas

Los experimentos incluyen trayectorias variadas para analizar el comportamiento en situaciones de tráfico:
- **Trayectorias rectas:** Simulan vehículos avanzando en línea.
- **Trayectorias circulares y ovaladas:** Evalúan la capacidad de los robots para coordinarse en curvas y mantener la distancia de equilibrio.
- **Circuito vehicular:** Simulación de tráfico en un circuito más complejo que replica condiciones de tráfico en carretera.
![TRAYECTORIA_RECTA](https://github.com/user-attachments/assets/2a177d46-9ca8-4918-bc1f-22ec22eca21a)

![TRAYECTORIA_CIRCULAR](https://github.com/user-attachments/assets/c75d875a-bbcd-4cb2-9339-966e13e47527)

![TRAYECTORIA_OVALADA](https://github.com/user-attachments/assets/728ef2d5-e5a2-4b05-92d9-60ad53f45ff1)

![CIRCUITO_PRUEBAS_ROBOTAT](https://github.com/user-attachments/assets/6b55e95c-4d61-4213-b2f2-d212d8dc666b)

## Plataforma Robotat

**Robotat** es una plataforma de pruebas ubicada en la Universidad del Valle de Guatemala. Está equipada con un sistema de captura de movimiento, permitiendo la experimentación con robots en un espacio controlado de 3x5 metros. Robotat fue esencial para validar el algoritmo en condiciones físicas y para ajustar el comportamiento de los robots en tiempo real.
![ROBOTAT1](https://github.com/user-attachments/assets/8b24669c-1fb8-4527-93c6-045f48974767)


## Programas Utilizados

- **Webots:** Utilizado para las simulaciones de las trayectorias y para evaluar el algoritmo de Lennard-Jones en entornos virtuales. Webots permite la captura de datos de movimiento y la replicación del entorno Robotat en un simulador.
![WEBOTS_TRAYECTORIA_RECTA_3](https://github.com/user-attachments/assets/21f78a15-6900-491d-b9fd-05b8622f7940)

- **MATLAB:** Se utilizo como herramienta de cálculo y análisis para generar codigo, asi como optimizar y ajustar los parámetros del potencial de Lennard-Jones en el algoritmo.  .
  ![VAL_SIM_TJ_2](https://github.com/user-attachments/assets/1b471bca-2091-44f1-b569-3b867c261b16)  

## Robot Pololu 3pi

Para las pruebas físicas, se utilizan robots móviles **Pololu 3pi**. Estos robots son ideales para simular vehículos en un entorno de tráfico controlado, gracias a su capacidad de movimiento rápido y estable, además de sus sensores de línea que facilitan la detección de trayectorias y de obstáculos.

El Pololu 3pi+ 32U4 es un robot móvil de alto rendimiento diseñado para proyectos de robótica educativa y de investigación. Basado en el microcontrolador ATmega32U4 compatible con Arduino, el 3pi+ 32U4 incorpora sensores avanzados, como encoders duales para control de velocidad y posición, sensores de línea, sensores de choque frontales y una IMU de 9 ejes (acelerómetro, giroscopio y magnetómetro). Su tamaño compacto, que cabe en la palma de la mano, lo hace ideal para tareas de seguimiento de líneas, laberintos y pruebas de navegación en entornos pequeños, además de ser fácil de programar y personalizar para diversas aplicaciones robóticas​
![ROBOTAT1](https://github.com/user-attachments/assets/8b24669c-1fb8-4527-93c6-045f48974767)


## Resultados

- **Pruebas Simuladas:** Se logra emular el fenómeno de tráfico fantasma, manteniendo distancias seguras y ajustando las velocidades de los robots para reproducir patrones de tráfico característicos.
- **Pruebas Físicas:** Los robots en la plataforma Robotat reaccionan correctamente a fuerzas de atracción y repulsión, y el controlador PID ajusta la orientación en trayectorias complejas, logrando un movimiento fluido y coordinado.

## Conclusiones

El algoritmo de Lennard-Jones permite emular eficazmente el fenómeno del tránsito fantasma en condiciones controladas. Su implementación en la plataforma Robotat demuestra la viabilidad de estudiar patrones de tráfico en entornos a escala, abriendo oportunidades para mejorar la simulación y el análisis del tráfico en robótica y vehículos autónomos.

---
