**driver_nav3d**

Funcionamento:
1. Reconhece a sua equipa e as equipas adversárias
2. Recebe informações do script player_finder
3. Decide, baseado nas informações que recebe, se usa pontos 3D ou pixeis da camara para definir o objetivo. (Não funcional XD)


Todos os algoritmos de decisão precisam de ser melhorados, pois contêm muitas falhas.


This script subscribe to:
1. /gazebo/get_model_state -> get position of all robots (GPS alike)
2. /blue1/scan -> get reads from the lidar to avoid obstacles

publish on:
1. /blue1/cmd_vel -> to control the motor wheels

**Launch gazebo with game arena:**

    roslaunch p_g06_bringup th_arenas.launch

**Launch 3 robots of each color (blue, red and green):**
    
    roslaunch p_g06_multi_robot multi_robot.launch

**Launch the referee:**

    rosrun th_referee th_referee

**Launch the script for one robot (blue1):**

    roslaunch oriented_goal one_robot_cmd_vel.launch 
