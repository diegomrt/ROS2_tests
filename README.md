# ROS2: tests, pruebas y comandos básicos
Pruebas, comandos básicos y ejemplos de ROS2 Galactic para Ubuntu 20.04 focal. 

*Nota*: En diciembre de 2021 hay dos versiones soportadas: ROS2 FOXY y ROS2 GALACTIC. Los ejemplos son para FOXY (tiene más paquetes por el momento)

## Documentación oficial
Disponible en
- FOXY: https://docs.ros.org/en/foxy/index.html
- GALACTIC: https://docs.ros.org/en/galactic/index.html

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

## Instalación de paquetes básicos
- rqt y sus plugins: `sudo apt install ros-foxy-rqt*`
- gazebo (11) y sus plugins: `sudo apt-get install ros-foxy-gazebo-*`

## Paquetes de ejemplo:
### Simulación del turtlebot 3
- Info e instalación en https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation

