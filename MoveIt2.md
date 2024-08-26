# Robótica industrial con ROS2 y MoveIt2

## Instalación de MoveIt2 desde binarios
No se instala con ROS2, hay que instalarlo aparte.
Recomendado instalar MOVEIT2 para HUMBLE, ya que está más desarrollado (Moveitconfig, soporte MOveitCommander python -comprobar-)
- ROS2 HUMBLE: `sudo apt install ros-humble-moveit`
- ROS2 FOXY: `sudo apt install ros-foxy-moveit`

## Tutoriales oficiales MoveIt2 (HUMBLE) con el Franka Emika Panda 
Documentación oficial y tutoriales (HUMBLE): https://moveit.picknik.ai/humble/index.html
Hay 6 tutoriales oficiales, todos recomendables como toma de contacto: 
   1. Getting started
   2. MoveIt QuickStart in RVIZ
   3. Your First C++ Moveit Project
   4. Visualizing in RVIZ
   5. Planning Around Objects
   6. Pick and place with Moveit Task Constructor

### 1. TUTORIAL "Getting Started"
Seguir instrucciones completas de https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html
- Hay que crear un repo (ws_moveit2_diego en mi caso) y clonar. Importante actualizar rosdep!!
- Importante: al ejecutar `vcs import < moveit2_tutorials/moveit2_tutorials.repos` se descarga TODO MOVEIT2 (creo que esto no sería necesario si se tiene instalado desde binarios)
- Se compila (todo MOVEIT2!) con `colcon build --mixin release`. Tarda mucho. En mi caso han sido necesarias tres pasadas para que termine los 55 paquetes
- Source del repo con `source ~/ws_moveit2/install/setup.bash`.
- Cambio de RMW a Cyclone DDS (había un problema con las DDS por defecto). Descargar y ´export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

### 2. TUTORIAL "MoveIt Quickstart in RViz2"
Tutorial de uso del plugin de Moveit2 para RViz2, colisiones, workspace, movimientos cartesianos, velocity scaling...
- IMPORTANTE: Si se lanza este tutorial inicial con `ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true` el robot Panda no aparece en RVIZ2, debido a un problema con las locales. 
- SOLUCION extraída de https://github.com/moveit/moveit2/issues/1049:
   - Lanzarlo con `LC_NUMERIC=en_US.UTF-8 ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true`, FUNCIONA!
 
### 3. TUTORIAL "Your First C++ MoveIt Project"
Clave para entender el proceso completo de creación de un ws para código C++
- https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html
- Al recompilar con `colcon build --mixin debug` a veces se recompilan algunos de los otros repos. Si falla tratar de volver a recompilar
- Si no hay fallos hacer source con `source install/setup.bash` y ejecutar con `ros2 run hello_moveit hello_moveit`

#### DOCKER para Tutoriales oficiales
Si se quiere trabajar sobre contenedir, seguir instrucciones de https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html

## Otros tutoriales y documentación 
- Curso ROS2 Industrial Training (únicamente en FOXY): https://industrial-training-master.readthedocs.io/en/foxy/index.html#

## Moveit2 con Python
moveit_commander (la interfaz en Python de move_group) está supuestamente lista desde el 28 de abril de 2023 (fue desarrollada en el verano de 2022 para Humble):
- https://picknik.ai/moveit/ros/python/google/2023/04/28/GSOC-MoveIt-2-Python-Bindings.html
   - Comprobar estado. A 14 de octubre de 2022 no estaba operativa
- Aparte, para trabajar con Python hay estos dos repositorios interesantes:
    1. PyMoveIt2: es un proyecto vivo que está operativo para Humble, Galactic e Iron!
       - https://github.com/AndrejOrsula/pymoveit2
    3. GEZP (parte del repo anterior): https://github.com/gezp/universal_robot_ign/blob/main/scripts/moveit2.py

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


## Comentarios y Notas de uso:

### 26.08.24. MoveIt2 para HUMBLE mucho más depurado
- Los tutoriales del Panda funcionan bien (arreglado con Locale)
- La simulacion del UR5e va muy bien el webots
- Probar task constructor (hay tutorial)
- PyMoveIt2 

### 14.10.22. Muchos más fallos de los esperados en HUMBLE
 - El Panda no lanza (Tutoriales)
 - Webots R2022a no carga bien enlaces y assets, y casca. Por ello, el UR no funciona! Pendiente de ver cómo enlazar R2022b con ROS. 
 - No parece estar disponible todavía moveit_commander (python)
 - Tampoco parece estar Moveit Task Constructor!
 - Recomendación: permanecer en FOXY y probar PyMoveIt2!
Note: As MoveIt! seems to have difficulties with finding plans for the robot with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use instead the ur5e_joint_limited launch file instead of the ur5e.launch one, i.e.: roslaunch ur_e_webots ur5e_joint_limited.launch
