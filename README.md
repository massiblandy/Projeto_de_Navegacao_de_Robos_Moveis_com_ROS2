# 🤖 Projeto de Navegação de Robôs Móveis com ROS2

## Descrição
Este é o Projeto Final da disciplina Navegação de Robôs Móveis (CCR210) que visa implementar e simular um algoritmo de planejamento de trajetória para um robô móvel usando ROS2 Humble e Gazebo. Utilizando o algoritmo A*, o robô simulado R2-D2 navega em um ambiente mapeado, desviando de obstáculos fixos.

---
## Objetivos
- Compreender e aplicar conceitos de mapeamento em robótica móvel.
- Implementar algoritmos de planejamento de trajetórias e rotas.
- Integrar e aplicar algoritmos de navegação no ROS2.
- Simular o comportamento do robô no ambiente Gazebo.

---
## Funcionalidades
- Mapeamento do ambiente utilizando ROS2 e Cartographer.
- Planejamento de trajetória utilizando o algoritmo A*.
- Controle do robô para seguir a trajetória planejada, desviando de obstáculos.
- Expansão dos obstáculos no mapa para garantir uma navegação segura.
- Simulação do robô R2-D2 no ambiente Gazebo.

---
## Como Executar
### Executar fora do Docker

1. **Rodando o ambiente ROS**:
No terminal do Visual Studio Code, rode o seguinte comando:
```bash
docker compose up ros-master
```
Quando o port for adicionado, clique em "Open in Browser" para acessar a interface.

---
### Executar dentro do Docker
#### Configurar o Ambiente de Simulação
Os comandos apresentados a partir de aquí são rodados no ambiente simulado do Gazebo.

2. **Configurando o Ambiente de Simulação**:
- Inicie o Gazebo com o ambiente simulado:
```sh
ros2 launch projeto_final simulation.launch.py world_path:=src/projeto_final/simulation/worlds/simple_room_with_fixed_boxes.world
```
- Carregue o robô R2-D2 no Gazebo:
```sh
ros2 launch projeto_final load.launch.py
```

3. **Mapeamento do Ambiente**:
O mapeamento do ambiente já foi feito, não precisa fazer novamente, mas caso queira mudar de mapa, poderia faze-lo e ajustar dentro do código AStar.py o caminho do mapa gerado e salvo (arquivo pgm).
- Inicie o mapeamento do ambiente:
```sh
ros2 launch projeto_final mapping_cartographer.launch.py
```
- Salve o mapa mapeado:
```sh
ros2 run nav2_map_server map_saver_cli
```

4. **Planejamento de Trajetória**:
- Execute o nó do A* para planejar a trajetória:
```sh
ros2 run projeto_final AStar
```

5. **Controle do Robô**:
- Execute o nó de controle para seguir a trajetória:
```sh
ros2 run projeto_final RobotControl
```
---
## Estrutura do Projeto
- **AStar.py**: Implementação do algoritmo A* para planejamento de trajetória.
- **RobotControl.py**: Implementação do controle do robô para seguir a trajetória planejada.
- **map.pgm**: Arquivo de mapa gerado pelo Cartographer.

---
## Ambientes utilizados
- ROS2 Humble
- Gazebo
- Ubuntu 22.04.4 LTS
- Docker
