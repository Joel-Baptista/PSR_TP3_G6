**ORIENTED_GOAL_CMD_VEL ##################################################**

Demonstration on one robot: https://www.youtube.com/watch?v=APcvfE-Za3w

Multi Robots: https://www.youtube.com/watch?v=g1-Rnxu0qS8

Funcionamento:
1. Robô player obtem as posições de todos os robôs na arena - azuis, vermelhos e verdes
2. Player adatapa o seu movimento para ir em direção do robô a caçar mais próximo (ex: azul vai em direção ao vermelho)
    O metedo utilizado para determinar qual o mais próximo é a calcular a distância entre si e todos os vermelhos e achar a menor
3. Esta posição é atualizada a cada ciclo (20 hz)
4. Pelo caminho o robô recebe os sinais do lidar, caso detete um objecto a menos de 0.9 m, adapta o seu movimento, virando para a esquerda ou para a direita.
    Caso fique muito próximo da parede, recua (0.2 m)
5. Caso detete um verde a menos de 2 m, deixa de ir para o vermelho e recalcula a trajetória para uma posição longe do verde

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

**Launch for all robots:**

    roslaunch oriented_goal multi_robot_cmd_vel.launch

**ORIENTED_GOAL ##################################################**

Video: https://youtu.be/RV6BOIRpye0 
 

Launch arena: 

    roslaunch p_g06_bringup th_arenas.launch arena:=3 

Launch robots: 

    roslaunch p_g06_multi_robot multi_robot.launch 

Launch navigation 

    roslaunch p_g06_navigation p_g06_navigation.launch 

Launch script 

    roslaunch oriented_goal one_robot.launch  

Launch referee: 

    rosrun th_referee th_referee 
