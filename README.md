# Validación de algoritmos de física granular con énfasis en el estudio y análisis del tránsito fantasma en entornos reales a escala

**Autor:** Brayan Gabriel Girón García  
**Institución:** Universidad del Valle de Guatemala  
**Facultad:** Ingeniería  
**Fecha:** 2024  

## Resumen
Este proyecto se centra en la validación de algoritmos de física granular para el estudio del fenómeno de "tránsito fantasma" en entornos a escala, utilizando agentes robóticos. Los algoritmos modelan las interacciones de atracción y repulsión entre partículas, simulando el efecto de "acordeón" característico de los atascos en el tráfico sin causas evidentes. Se implementa el algoritmo de Lennard-Jones, adaptado para funcionar en un sistema bidimensional con robots móviles.

## Objetivos
1. Optimizar y validar algoritmos de física granular en entornos de simulación y físicos.
2. Replicar el fenómeno de tránsito fantasma utilizando agentes robóticos móviles.
3. Evaluar y ajustar el algoritmo de Lennard-Jones en escenarios de tránsito a escala.

## Alcance
El proyecto se limita a simulaciones bidimensionales en entornos controlados y físicos. Utiliza la plataforma Robotat, la cual permite realizar experimentos en un espacio de 3x5 metros, con un número limitado de robots. Las pruebas incluyen trayectorias lineales y circulares, observando el comportamiento de los robots ante fuerzas de atracción y repulsión.

## Metodología
1. **Implementación del Algoritmo**: Adaptación del algoritmo de Lennard-Jones para robots móviles, ajustando fuerzas de interacción entre partículas.
2. **Diseño Experimental**: Pruebas en trayectorias variadas para evaluar el comportamiento de los robots, incluyendo pruebas de control sin el algoritmo y pruebas con el algoritmo implementado.
3. **Optimización**: Reducción de costos computacionales y ajuste de parámetros clave (ε = 0.3, σ = 0.4) para mejorar la simulación.

## Resultados
- **Pruebas Simuladas**: El algoritmo logró replicar el fenómeno de tránsito fantasma en simulaciones bidimensionales, con robots manteniendo distancias seguras y ajustando velocidades.
- **Pruebas Físicas**: La implementación en la plataforma Robotat mostró comportamientos realistas, ajustando la orientación de los robots mediante un controlador PID para trayectorias complejas.

## Conclusiones
El algoritmo de Lennard-Jones permite replicar eficazmente el fenómeno de tránsito fantasma. La implementación en entornos controlados y la optimización de parámetros garantizan la viabilidad del uso de agentes robóticos para el estudio de problemas de tráfico. 

---

Este resumen es una estructura base para tu README; puedes expandir cada sección con información técnica adicional según sea necesario para los colaboradores del repositorio.
