**Launch player_finder:**

Lançamento do nó:

        rosrun p_g06_sensors player_finder __name:= <input player name>_sensors

Exemplo para o nome: blue1_sensors. Esta alteração foi necessária para não haver conflito com nomes

**Description of message PlayerLocation**

- idx - Vetor com os indices dos objetos (Todas as restantes informações estão associadas a este indice).
- teams - Informa sobre a equipa do objeto encontrado ([Red, Blue, Green]).
- locations - Localização do objeto em 3D (PosedStamp). (Se a localização for [x = 0, y = 0, z = 0)] signifca que _não existe localização 3D do objeto_)
- Xpixel - Informa sobre a coordenada X do centroide do objeto na camara.
- Ypixel - Informa sobre a coordenada Y do centroide do objeto na camara.

**Launch Listener.py:**

Este script apenas serve para verificar se a mensagem está a ser bem transmitida pelo player_finder

    rosrun p_g06_sensors listener.py -tn <input player name>"

