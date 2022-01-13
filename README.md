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
- RQT y sus plugins: `sudo apt install ros-foxy-rqt*`
    - Ejecutar con `ros2 run rviz2 rviz2`
    - También con `rviz2` directamente
- NAV2: El sucesor del navigation stack de ROS1: https://navigation.ros.org/getting_started/index.html
    - Instalar con `sudo apt install ros-foxy-navigation2` y `sudo apt install ros-foxy-nav2-bringup`
- CARTOGRAPHER (SLAM): https://index.ros.org/r/cartographer_ros/
    - Instalar con `sudo apt install ros-foxy-cartographer` y `sudo apt install ros-foxy-cartographer-ros`  

## Simuladores 
### Gazebo classic
- gazebo classic (11) y sus plugins: `sudo apt-get install ros-foxy-gazebo-*`
    - Ejecución básica de test: ´ros2 launch gazebo_ros gazebo.launch.py´. Funciona
### Ignition Gazebo
- Varias "releases" disponibles: https://ignitionrobotics.org/docs/all/releases
    - Instalo "Citadel", ya que la última LTS "Fortress" es de septiembre de 2021 y quizás no esté madura
- Instalación (independiente de ROS) en https://ignitionrobotics.org/docs/citadel/install_ubuntu
    - Test con `ign gazebo shapes.sdf`. OK!
- Integración con ROS2 descrita en https://ignitionrobotics.org/docs/citadel/ros2_integration 
    - Instalación desde binarios con `sudo apt install ros-foxy-ros-ign*`
    - Test básico (pendiente).   
### Webots
- Paquete de instalación para ROS2: https://github.com/cyberbotics/webots_ros2
    - Instalación con `sudo apt-get install ros-foxy-webots-ros2`
    - La primera vez pide descargar y ejecutar webots R2021b
    - Ejecución básica de test: `ros2 launch webots_ros2_universal_robot multirobot_launch.py`
- Listado de robots incluidos en https://cyberbotics.com/doc/guide/robots 

## Paquetes de ejemplo
### Simulación del turtlebot 3
- Info e instalación en https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
- Simulación: 
    - GAZEBO: Instalar paquetes simulación + `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
    - WEBOTS: Instrucciones en https://github.com/cyberbotics/webots_ros2/wiki/Navigate-TurtleBot3
        - Lanzar simulación con `ros2 launch webots_ros2_turtlebot robot_launch.py`
        - Mover robot básico con `ros2 topic pub -t 3 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}`
## Robótica industrial. MoveIt2
### Simulación del manipulador UR5e
#### UR5e en Ignition Gazebo: 
Repo en https://github.com/gezp/universal_robot_ign
    - Problema: es para RO2 Galactic, no compila bien (error con ignition_transport10)
    - Solución: mejor usar webots
#### UR5e en Webots
Repo en el github del propio desarrollador de Webots https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots
1. UR5e en Webots sin RViz ni MoveIt: `ros2 launch webots_ros2_universal_robot robot_launch.py`
    - Se lanzan 5 nodos: /UR5e, /controller_manager, /robot_state_publisher, /jount_state_broadcaster, /joint_trajectory_controller
2. UR5e en Webots con RViz y MoveIt2: `ros2 launch webots_ros2_universal_robot moveit_demo_launch.py`
3. UR5e + ABB en Webots: `ros2 launch webots_ros2_universal_robot multirobot_launch.py`
