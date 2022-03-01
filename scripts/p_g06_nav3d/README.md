**driver_nav3d**

Funcionamento:
1. Reconhece a sua equipa e as equipas adversárias
2. Recebe informações do script player_finder
3. Decide, baseado nas informações que recebe, se usa pontos 3D ou pixeis da camara para definir o objetivo. (Não funcional XD)

This script subscribe to:
1. /\<player>/player_location -> get position of that can see (with or withou sensors)

publish on:
1. /\<player>/cmd_vel -> to control the motor wheels

**Launch gazebo with game arena:**

    roslaunch p_g06_bringup th_arenas.launch

**Launch 3 robots of each color (blue, red and green):**
    
    roslaunch p_g06_multi_robot multi_robot.launch

**Launch the referee:**

    rosrun th_referee th_referee

**Launch the script for the sensors:**

    rosrun p_g06_sensors player_finder __name:=<playe>_sensors

**Launch the script for the navigation:**

    rosrun p_g06_sensors driver_nav3d __name:=<player>_nav3d

**(MAYEBE)Launch the script for the navigation:**
    
Só usar se for necessário imobilizar o robô. Apenas será usado em fase de teste e desenvolvimento de código

    rosrun p_g06_sensors driver_nav3d __name:=<player>_reset

