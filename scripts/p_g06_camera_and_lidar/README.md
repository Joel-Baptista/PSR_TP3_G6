Este script lê os tópicos scan e camera dos jogadores e toma decisões.

**Launch the script for 3 robots of each team**

    roslaunch p_g06_camera_and_lidar multi_robot_camera_lidar.launch

atualmentes está:

-- não vê nenhum jogador: anda em linha reta até encontrar objetos, pelo lidar, e desvia-se

-- vê um jogador para apanhar: aumenta a velocidade e vai em direção ao jogador

-- vê um jogador para fugir: anda de marcha a trás tentando não perder a visão do jogador e tentando tambem desviar de objetos para enganar o jogador que está a caçar
    este script dá mais valor a fugir que a apanhar, ou seja, se estiver a apanhar e vir um caçador, muda para modo de fuga

PS: ainda tem muitas falhas e precisa de ser calibrado
por uma camera trazeira podia ser uma ideia, desde que não criasse conflitos na publicação no tópico cmd_vel

**Launch the script on one robot for testing**

    roslaunch p_g06_camera_and_lidar one_robot_camera_lidar.launch 

