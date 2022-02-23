**ORIENTED_GOAL_CMD_VEL**

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

