Este script deve ser lançado após lançar o gazebo e todos os jogadores.

O script vai receber as posições dos varios jogadores utilizando o serviço get model state e publicar essas posições com uma ação no tópico move_base.

O tópico move_base é lançado com o launch p_g06_navigation.launch

Assim, como o jogador sabe as posições de todos os jogadores, vai tomar decisões para saber para onde ir.

Atualmente está:

calcula a distância, em linha reta, que esta de todos os outros jogadores e se estiver um jogador para apanhar vai até ele, caso encontre um jogador para fugir vai para nova posição.


PS: este código ainda precisa de bastantes melhorias