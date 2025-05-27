# pf_adr

Este paquete de ROS2 ha sido creado para implementar un filtro de partículas para estimar la posición de un número establecido de balizas. Se han creado tres posibles implementaciones: 
- Filtro de partículas básico: lanza un nodo de filtro de partículas por cada baliza, estimando la posición de la baliza con un número constante de puntos en cada nube de partículas.
- Filtro de partículas "smart": este filtro también lanza un filtro de partículas por cada baliza, pero recibe información sobre el número de balizas activas que hay en cada momento, permitiendo mantener un número constante de partículas en general, es decir, la suma de las partículas de todos los nodos es constante.
- Filtro de partículas con EKF: por último, se ha creado un filtro de partículas, basado en el filtro de partículas básico, que realiza pruebas de gausianidad sobre la nube de partículas y, cuando supera un cierto umbral, aproxima la nube de partículas por una media y una covarianza y se le pasa a un nodo que implementa EKF, permitiendo reducir el coste computacional. El EKF continuará estimando la posición de la baliza con el dato que ha recibido del PF. 


## Instalación:
Primero crearemos un workspace y una carpeta *src*:
```Terminal
mkdir -p ros2_ws/src
cd ros2_ws/src
```
Para clonar el repositorio con el submodulo del sjtu:

```Terminal
git clone --recurse-submodules https://github.com/rme19/pf_adr.git 
```

## Utilización:
Para una mayor sencillez, se han creado archivos para lanzar todas las aplicaciones y nodos necesarios, además de la configuración del número de balizas que estimaremos y parámetros del filtro de partículas (número de partículas, ruidos...). 

### Filtro de partículas básico
Como hemos dicho anteriormente, esta implementación consiste en la utilización de un filtro de partículas con un número constante de partículas para la estimación de la posición de una baliza fija. 

Para cambiar el número de balizas que se van a estimar, deberemos modificar el archivo `launch_completo.py` y el archivo `sjtu_drone_fp.launch.py` y modificar la variable `num_beacons`. Además, en el archivo `launch_completo.py` podremos modificar varios parámetros que se utilizarán en el filtro, como el número de partículas, ruido de resampleo...

Si queremos ejecutar esta implementación: 
```Terminal
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch pf_adr launch_completo.py 
```
Automáticamente se nos debería abrir la simulación de Gazebo, el teleop para poder controlar el dron y una pestaña de RViz con la configuración para poder visualizar las nubes de partículas. Además, tendremos funcionando N nodos de balizas y N nodos de filtros de partículas, siendo N el número de balizas que hemos configurado. 

### Filtro de partículas "smart"
Esta implementación se diferencia de la anterior en que recibimos datos de un nodo `activity_beacon_control`, que nos informa sobre el número de balizas activas (dentro de rango) que hay en cada momento y nos permite ajustar el número de partículas con las que trabaja cada filtro en tiempo de ejucion, manteniendo siempre un número global de partículas constante para la estimación.

Para cambiar el número de balizas que se van a estimar, deberemos modificar el archivo `launch_completo_2.py` y el archivo `sjtu_drone_fp.launch.py` y modificar la variable `num_beacons`. Además, en el archivo `launch_completo_2.py` podremos modificar varios parámetros que se utilizarán en el filtro, como el número de partículas, ruido de resampleo...


Si queremos ejecutar esta implementación: 
```Terminal
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch pf_adr launch_completo_2.py 
```
Automáticamente se nos debería abrir la simulación de Gazebo, el teleop para poder controlar el dron y una pestaña de RViz con la configuración para poder visualizar las nubes de partículas. Además, tendremos funcionando N nodos de balizas, N nodos de filtros de partículas y un nodo que monitoriza el número de balizas activas, siendo N el número de balizas que hemos configurado.

### Filtro de partículas con EKF
En esta versión, combinaremos el filtro de partículas básico con un filtro de Kalman extendido. Para ello, hemos incorporado un método que comprueba el nivel de gausianidad y, cuando supera un cierto umbral preestablecido, estima una media y una matriz de covarianza y se lo manda al EKF. 

La conexión entre un filtro y otro es sencilla, ya que ambos nodos se lanzan al iniciar la simulación, pero el nodo del EKF queda bloqueado a la espera de recibir una estimación inicial, mientras que es el filtro de partículas el que comienza a estimar la posición de la baliza. Una vez el filtro de partículas ha convergido y ha alcanzado un nivel de gausianidad lo suficientemente alto, se publica la estimación de la media y la matriz de covarianza en un topic que está esperando el EKF, permitiendo que el EKF comience su estimación. Además, se activa una flag en el filtro de partículas que bloquea la actualización de las partículas. De esta forma, conseguimos que el EKF y el filtro de partículas no funcionen simultáneamente en ningún momento.

Para cambiar el número de balizas que se van a estimar, deberemos modificar el archivo `launch_completo_ekf.py` y el archivo `sjtu_drone_fp.launch.py` y modificar la variable `num_beacons`. Además, en el archivo `launch_completo_ekf.py` podremos modificar varios parámetros que se utilizarán en los filtro, como el número de partículas, niveles de ruido...


Si queremos ejecutar esta implementación: 
```Terminal
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch pf_adr launch_completo_ekf.py 
```
Automáticamente se nos debería abrir la simulación de Gazebo, el teleop para poder controlar el dron y una pestaña de RViz con la configuración para poder visualizar las nubes de partículas. Además, tendremos funcionando N nodos de balizas, N nodos de filtros de partículas y N nodos de filtro de Kalman extendido, siendo N el número de balizas que hemos configurado.
