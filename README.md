# ROS2: tests, pruebas y comandos básicos
Pruebas, comandos básicos y ejemplos de ROS2 Galactic para Ubuntu 20.04 focal. 

*Nota*: En diciembre de 2021 hay dos versiones soportadas: ROS2 FOXY y ROS2 GALACTIC. Los ejemplos son para FOXY (tiene más paquetes por el momento)

*Nota 2*: He tenido problemas al tener instalados ROS1 Noetic y ROS2 Foxy en la misma máquina (tipo symbol lookup error: hasta en las demos). Se han solucionado desinstalando Noetic

## Documentación oficial
Disponible en
- FOXY: https://docs.ros.org/en/foxy/index.html
- GALACTIC: https://docs.ros.org/en/galactic/index.html

## Test básicos tras instalación
Extraídos de https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- `ros2 run demo_nodes_cpp talker`
- `ros2 run demo_nodes_py listener`

## Comandos básicos
- Ejecutables de un paquete:  `ros2 run turtlesim turtle_teleop_key`
- Listar nodos, topics, acciones y servicios:
```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```
- Buscar paquetes instalados: `ros2 pkg list | grep paquete_en_busqueda`

## Creación de paquetes
- Siempre en el directorio src del workspace: `ros2 pkg create --build-type ament_python nombredelpaquete`

## COLCON (trabajo con workspaces en ROS2)
- Documentación oficial en https://colcon.readthedocs.io/en/released/
- Instalación colcon y extensiones: `sudo apt install python3-colcon-common-extensions`
- Para hacer source de un colcon_ws: `source nombre_ws/install/setup.bash`
### Build con COLCON
- Instrucciones en https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html
- Uso básico: `colcon build --symlink-install`

## Instalación de paquetes básicos:
- rqt y sus plugins: `sudo apt install ros-foxy-rqt*`
    - Ejecutar con ´ros2 run rviz2 rviz2´
    - También con ´rviz2´ directamente

## Simuladores 
### Gazebo
- gazebo (11) y sus plugins: `sudo apt-get install ros-foxy-gazebo-*`
    -Ejecución básica de test: ´ros2 launch gazebo_ros gazebo.launch.py´
### Webots
- Paquete de instalación para ROS2: https://github.com/cyberbotics/webots_ros2
    - Instalación con ´sudo apt-get install ros-foxy-webots-ros2´
    - La primera vez pide descargar y ejecutar webots R2021b
    - Ejecución básica de test: ´ros2 launch webots_ros2_universal_robot multirobot_launch.py´

## Paquetes de ejemplo:
### Simulación del turtlebot 3
- Info e instalación en https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation

