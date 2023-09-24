# ROS2: tests, pruebas y comandos básicos
Pruebas, comandos básicos y ejemplos de ROS2 Galactic para Ubuntu 20.04 focal. 

*Nota*: En diciembre de 2021 hay dos versiones soportadas: ROS2 FOXY y ROS2 GALACTIC. Los ejemplos son para FOXY (tiene más paquetes por el momento)

*Nota 2*: Update octubre 2022: instalo Ubuntu 22.04 y ROS2 Humble (LTS hasta 2027)

## Documentación oficial
Disponible en
- FOXY: https://docs.ros.org/en/foxy/index.html
- GALACTIC: https://docs.ros.org/en/galactic/index.html
- HUMBLE: https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html

Instalación desde binarios Debian :
- ROS2 FOXY (en Ubuntu 20.04): https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- ROS2 HUMBLE (en Ubuntu 22.04): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.htmlsud

## Test básicos tras instalación
Extraídos de https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- `ros2 run demo_nodes_cpp talker`
- `ros2 run demo_nodes_py listener`
Tutoriales (HUMBLE)
- https://docs.ros.org/en/humble/Tutorials.html
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

## COLCON (trabajo con workspaces en ROS2)
- Documentación oficial en https://colcon.readthedocs.io/en/released/
- Instalación colcon y extensiones: `sudo apt install python3-colcon-common-extensions`
- Para hacer source de un colcon_ws: `source nombre_ws/install/setup.bash`
### Autocomplete de COLCON:
- Incluir en .bashrc `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`
### Build con COLCON
- Instrucciones en https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html
- Uso básico: `colcon build --symlink-install` (opción recomendada para usar código Python actualizado sin recompilar workspace)
- Compilar paquetes seleccionados: `colcon build --packages-select nombre-del-paquete`

## Creación de paquetes Python
- En el directorio src del workspace: `ros2 pkg create nombredelpaquete --build-type ament_python --dependencies rclpy`
- NOTA: a septiembre de 2023 la versión 58.6.0 de setuptools da problemas de compilación, hay que hacer un downgrade con:
    - `pip3 install setuptools==58.2.0`
- INSTALL: para ue aparezcan al hacer `ros2 run paquete...` importante incluir información de entry_points en el fichero setup.py del paquete
 
## Creación de paquetes C++
- En el directorio src del workspace: `ros2 pkg create nombredelpaquete --build-type ament_cmake --dependencies rclcpp`

## Instalación de paquetes básicos (ROS2 HUMBLE):
- RVIZ:
    - Ejecutar con `ros2 run rviz2 rviz2`
    - También con `rviz2` directamente
- RQT y sus plugins: `sudo apt install ros-humble-rqt*`
- NAV2: El sucesor del navigation stack de ROS1: https://navigation.ros.org/getting_started/index.html
    - Instalar con `sudo apt install ros-humble-navigation2` y `sudo apt install ros-foxy-nav2-bringup`
- CARTOGRAPHER (SLAM): https://index.ros.org/r/cartographer_ros/
    - Instalar con `sudo apt install ros-humble-cartographer` y `sudo apt install ros-humble-cartographer-ros`  

## Simuladores (ROS2 HUMBLE)
### Gazebo classic
- gazebo classic (11) y sus plugins: `sudo apt-get install ros-humble-gazebo-*`
    - Ejecución básica de test: ´ros2 launch gazebo_ros gazebo.launch.py´. Funciona
### Ignition Gazebo (pendiente revisión para HUMBLE)
- Varias "releases" disponibles: https://ignitionrobotics.org/docs/all/releases
    - Instalo "Citadel", ya que la última LTS "Fortress" es de septiembre de 2021 y quizás no esté madura
- Instalación (independiente de ROS) en https://ignitionrobotics.org/docs/citadel/install_ubuntu
    - Test con `ign gazebo shapes.sdf`. OK!
- Integración con ROS2 descrita en https://ignitionrobotics.org/docs/citadel/ros2_integration 
    - Instalación desde binarios con `sudo apt install ros-foxy-ros-ign*`
    - Demos disponibles en https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos 
       - Demo de cámara RGBD: `ros2 launch ros_ign_gazebo_demos rgbd_camera_bridge.launch.py`    
### Webots
- Paquete de instalación para ROS2: https://github.com/cyberbotics/webots_ros2
    - Instalación con `sudo apt-get install ros-humble-webots-ros2`
    - Ejecución básica de test: `ros2 launch webots_ros2_universal_robot multirobot_launch.py`
        - La primera vez pide descargar y ejecutar webots R2022a (octubre 2022)
- Listado de robots incluidos en https://cyberbotics.com/doc/guide/robots 
- Tutorial creación de paquete para la simulacion (HUMBLE): https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots.html

## Paquetes de ejemplo
### Simulación del turtlebot 4 (HUMBLE)
- Info e instalación en https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
- Simulación: 
    - GAZEBO:
        - Instalar paquete simulación con `sudo apt install ros-humble-turtlebot4-simulator`
        - Lanzar con `ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py`

### Simulación del turtlebot 3
- Info e instalación en https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
- Simulación: 
    - GAZEBO: Instalar paquetes simulación + `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
    - WEBOTS: Instrucciones en https://github.com/cyberbotics/webots_ros2/wiki/Navigate-TurtleBot3
        - Lanzar simulación con `ros2 launch webots_ros2_turtlebot robot_launch.py`
        - Mover robot básico con `ros2 topic pub -t 3 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}`


## TO DO LIST (as September 23)
- Test ros1_bridge: https://github.com/ros2/ros1_bridge/blob/master/README.md
