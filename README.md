# pf_adr

Para clonar el repositorio con el submodulo del sjtu:

` git clone --recurse-submodules https://github.com/rme19/pf_adr.git `

Para ejecutar todo: 

` ros2 launch pf_adr launch_completo.py `

Esto lanza la simulación con un mundo vacío, una baliza y el teleop

Para lanzar el filtro de partículas y el nodo de la baliza:

`ros2 run pf_adr beacon_node --ros-args -p target_position:="[3.0, 1.0, 1.5]"`

`ros2 run pf_adr particle_filter_node --ros-args -p num_particles:="5000" -p sigma:="0.02" -p noise_std:=0.02 -p radius:=2.5`

Si quereis ver las partículas moviendose, abris rviz2 es otro terminal y añadis el topic de pf/beacon_estimate, el array para las particulas y el pose para la posicion estimada



