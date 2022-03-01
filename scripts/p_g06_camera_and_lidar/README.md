Video: https://www.youtube.com/watch?v=xCXbXHZ2jH4

This script subscribe to:
camera topic
scan topic

Publishes on
command velocity topic

Use camera and lidar data to make decisions.

**Launch gazebo with game arena:**

    roslaunch p_g06_bringup th_arenas.launch arena:=3

**Launch 3 robots of each color (blue, red and green):**
    
    roslaunch p_g06_multi_robot multi_robot.launch

**Launch the script for 3 robots of each team**

    roslaunch p_g06_camera_and_lidar multi_robot_camera_lidar.launch

**Launch the script on one robot for testing**

    roslaunch p_g06_camera_and_lidar one_robot_camera_lidar.launch 


################################################

atualmente:

-- não vê nenhum jogador: anda em linha reta até encontrar objetos, pelo lidar, e desvia-se

-- vê um jogador para apanhar: aumenta a velocidade e vai em direção ao jogador

-- vê um jogador para fugir: anda de marcha a trás tentando não perder a visão do jogador e tentando tambem desviar de objetos para enganar o jogador que está a caçar
    este script dá mais valor a fugir que a apanhar, ou seja, se estiver a apanhar e vir um caçador, muda para modo de fuga

PS: ainda tem muitas falhas e precisa de ser calibrado
por uma camera trazeira podia ser uma ideia, desde que não criasse conflitos na publicação no tópico cmd_vel



