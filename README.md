# pf_adr

Para clonar el repositorio con el submodulo del sjtu:

` git clone --recurse-submodules https://github.com/rme19/pf_adr.git `

Para ejecutar todo: 

` ros2 launch pf_adr launch_completo.py `

Si quereis ver las partículas moviendose, abrís rviz2 es otro terminal y añadis el topic de pf/beacon_estimate, el array para las particulas y el pose para la posicion estimada
