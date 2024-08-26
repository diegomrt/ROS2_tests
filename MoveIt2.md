# Robótica industrial con ROS2 y MoveIt2

## Instalación de MoveIt2 desde binarios
No se instala con ROS2, hay que instalarlo aparte.
Recomendado instalar MOVEIT2 para HUMBLE, ya que está más desarrollado (Moveitconfig, soporte MOveitCommander python -comprobar-)
- ROS2 HUMBLE: `sudo apt install ros-humble-moveit`
- ROS2 FOXY: `sudo apt install ros-foxy-moveit`

## Documentación y tutoriales oficiales
- Documentación oficial y tutoriales (HUMBLE): https://moveit.picknik.ai/humble/index.html
- Documentación oficial y tutoriales (FOXY): https://moveit.picknik.ai/foxy/index.html
- Curso ROS2 Industrial Training (únicamente en FOXY): https://industrial-training-master.readthedocs.io/en/foxy/index.html#

### Tutorial oficial MoveIt2 (HUMBLE) con el Franka Emika Panda 
Seguir instrucciones completas de https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html
- Hay que crear un repo (ws_moveit2_diego en mi caso) y clonar. Importante actualizar rosdep!!
- Se descarga TODO MOVEIT2 (creo que esto no sería necesario si se tiene instalado desde binarios)
- Se compila con `colcon build --mixin release`. Tarda mucho. En mi caso han sido necesarias tres pasadas para que termine los 55 paquetes
- Ejecutar el inicial con `ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true`

## Moveit2 con Python
- moveit_commander (la interfaz en Python de move_group) está siendo desarrollada en el verano de 2022 para Humble
   - Comprobar estado. A 14 de octubre de 2022 no está operativa
- Por el momento para trabajar con Python hay estos dos repositorios interesantes:
    1. PyMoveIt2: https://github.com/AndrejOrsula/pymoveit2
    2. GEZP (parte del repo anterior): https://github.com/gezp/universal_robot_ign/blob/main/scripts/moveit2.py

## Simulación del manipulador UR5e
### UR5e repo oficial Ignition Gazebo
Repo oficial mantenido por Universal Robots: https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation
  - Recordatorio: instalación desde binarios Ignition Gazebo para FOXY con `sudo apt install ros-foxy-ros-ign`
    - Versión "Citadel" para FOXY (más infor sobre versiones en https://github.com/gazebosim/ros_gz)   
CONCLUSIÓN: la simulación está verde, y además sigue habiendo problemas de grasping (va mejor que Gazebo classic pero sigue fallando a vecess)

### Alternativa para el UR5e en Ignition Gazebo: 
Repo en https://github.com/gezp/universal_robot_ign
    - Problema: es para RO2 Galactic, no compila bien (error con ignition_transport10)
    - Solución: mejor usar webots
    
### UR5e en Webots + planificación con MoveIt 2
Repo en el github del propio desarrollador de Webots https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots
Más información en https://cyberbotics.com/doc/guide/ure
1. UR5e + ABB en Webots: `ros2 launch webots_ros2_universal_robot multirobot_launch.py`
    - Simulada incluso una cinta transportadora
3. UR5e en Webots sin RViz ni MoveIt: `ros2 launch webots_ros2_universal_robot robot_launch.py`
    - Se lanzan 5 nodos: /UR5e, /controller_manager, /robot_state_publisher, /jount_state_broadcaster, /joint_trajectory_controller
4. UR5e en Webots con RViz y MoveIt2: `ros2 launch webots_ros2_universal_robot moveit_demo_launch.py`

#### CONSTRUCCIÓN DE ESCENA CON UR5 Y CÁMARA RGB Ó RGBD
Idea: generar un paquete con un mundo similar al de los 2 robots. Eliminar ABB e incluir una cámara. hacer movimientos simples con moveit
paquete 
   - El UR5 del ejemplo es completamente funcional en MoveIt2 FOXY: ros2 launch webots_ros2_universal_robot moveit_demo_launch.py 
       - Nota: En RVIZ2 falla la visualización del robot (pero sí se ven bien las TF)
Copiar y pegar el paquete no funciona. Hay que seguir estas instrucciones:
   - https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots.html


### Comentarios y Notas de uso:
14.10.22. Muchos más fallos de los esperados en HUMBLE
 - El Panda no lanza (Tutoriales)
 - Webots R2022a no carga bien enlaces y assets, y casca. Por ello, el UR no funciona! Pendiente de ver cómo enlazar R2022b con ROS. 
 - No parece estar disponible todavía moveit_commander (python)
 - Tampoco parece estar Moveit Task Constructor!
 - Recomendación: permanecer en FOXY y probar PyMoveIt2!
Note: As MoveIt! seems to have difficulties with finding plans for the robot with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use instead the ur5e_joint_limited launch file instead of the ur5e.launch one, i.e.: roslaunch ur_e_webots ur5e_joint_limited.launch
