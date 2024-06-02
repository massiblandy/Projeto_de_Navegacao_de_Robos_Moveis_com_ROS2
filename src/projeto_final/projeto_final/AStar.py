import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import heapq
import numpy as np
from matplotlib import pyplot as plt
import os

class NodeAStar:
    #Inicializa um nó com sua posição, pai, custo g, função heurística h e função de avaliação f
    def __init__(self, position, parent, g=0, h=0, f=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = f

    #Compara nós
    def __eq__(self, other):
        return self.position == other.position

    #Ordena nós
    def __lt__(self, other):
        return self.f < other.f

#Função heurística para calcular h baseada na distância de Manhattan
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

#Função A*
def astar(matrix, start, end, logger):
    #Cria os nós de início e fim
    start_node = NodeAStar(start, None)
    end_node = NodeAStar(end, None)

    #Cria listas aberta (todos os nós que ainda não foram abertos) e fechada (contém os nós que já foram analisados)
    open_list = []
    closed_list = []

    #Adiciona o nó de início à lista aberta
    heapq.heappush(open_list, start_node)

    iteration_count = 0
    #Loop até encontrar o fim
    while len(open_list) > 0:
        iteration_count += 1

        #Pega o nó atual
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        if iteration_count % 100 == 0:
            logger.info(f'Iteration {iteration_count}: Current node position: {current_node.position}')

        #Checa se alcançamos o objetivo, retorna o caminho
        if current_node == end_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            logger.info(f'Path found after {iteration_count} iterations')
            return path[::-1] #Retorna o caminho invertido

        #Gera filhos
        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)] #Nós adjacentes e diagonais

        for next in neighbors:
            #Verifica se está dentro dos limites da matriz
            if next[0] > (len(matrix) - 1) or next[0] < 0 or next[1] > (len(matrix[0]) - 1) or next[1] < 0:
                continue

            #Pega valor do mapa
            map_value = matrix[next[0]][next[1]]
            #Checa se é um obstáculo
            if map_value == 0:
                logger.debug(f'Skipping obstacle at {next}')
                continue

            #Cria um nó filho
            child = NodeAStar(next, current_node)

            #Conferindo se nó filho está na lista já analisada
            if child in closed_list:
                continue

            #Cria os valores f, g e h dos nós filhos
            child.g = current_node.g + 1
            child.h = heuristic(child.position, end_node.position)
            child.f = child.g + child.h

            #Nó filho já está na lista de nós que ainda não foram abertos
            if add_to_open(open_list, child):
                heapq.heappush(open_list, child)
                logger.debug(f'Adding {next} to open list with f={child.f}')

    logger.info('No path found')
    return None

#Checa se um nó filho deve ser adicionado à open_list (lista de nós que ainda não foram abertos)
def add_to_open(open_list, child):
    for i, node in enumerate(open_list):
        if child == node:
            if child.g < node.g:
                open_list[i] = child #Substitui o nó existente pelo novo com menor custo g
                return True
            else:
                return False
    return True

#Função para aumentar o tamanho dos obstáculos na matriz de mapa, garantindo que o robô mantenha uma distância segura dos obstáculos ao planejar sua trajetória. 
#Isso é feito "expandindo" os obstáculos, o que significa marcar células adicionais ao redor de cada obstáculo existente como parte do obstáculo.
def expand_obstacles(matrix, expansion_size):
    expanded_matrix = np.copy(matrix) #Cria uma cópia da matriz original para não modificar a matriz original diretamente
    obstacle_indices = np.argwhere(matrix == 0) #Encontra os índices (coordenadas) de todas as células que são obstáculos (valor 0) na matriz
    for idx in obstacle_indices: #Iterando sobre cada índice de obstáculo encontrado
        for x in range(-expansion_size, expansion_size + 1): #Para cada obstáculo, itera sobre a área ao redor dele dentro do tamanho de expansão especificado
            for y in range(-expansion_size, expansion_size + 1):
                new_x, new_y = idx[0] + x, idx[1] + y #Calcula as novas coordenadas a serem marcadas como obstáculos
                if 0 <= new_x < matrix.shape[0] and 0 <= new_y < matrix.shape[1]: #Verifica se as novas coordenadas estão dentro dos limites da matriz
                    expanded_matrix[new_x, new_y] = 0 #Marca a célula nas novas coordenadas como obstáculo (valor 0) na matriz expandida
    #Retornando a matriz expandida com os obstáculos aumentados
    return expanded_matrix

class PathNode(Node):
    def __init__(self):
        super().__init__('astar_node')
        self.current_pose = None #Inicializando a pose atual do robô
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) #Cria uma subcriber para receber mensagens de Odometry do tópico '/odom'
        self.path_calculated = False #Inicializando a flag path_calculated como False
        self.path_saved = False #Inicializando a flag path_saved como False
        self.imap = 0

    #Função de callback para receber dados de odometria
    def odom_callback(self, msg):
        if self.current_pose is None: #Se a pose atual ainda não foi definida, define a pose atual com base na mensagem recebida
            self.current_pose = msg.pose.pose
            self.get_logger().info(f'Initial pose: x={self.current_pose.position.x}, y={self.current_pose.position.y}')
            self.run_astar() #Executando o algoritmo A*

    #Função para carregar o mapa
    def load_map(self, file_path):
        self.get_logger().info(f'Loading map from {file_path}')
        try:
            #Lê a imagem do mapa e converte os valores para binário (0 ou 1)
            matrix = plt.imread(file_path)
            matrix = 1.0 * (matrix > 250)
            self.get_logger().info(f'Map loaded with shape {matrix.shape}')
            return matrix
        except Exception as e: #Registra um erro se o carregamento do mapa falhar
            self.get_logger().error(f'Failed to load map: {e}')
            return None

    #Função para plotar o mapa
    def plot_map(self, matrix):
        self.get_logger().info('Plotting original map and map with expanded obstacles')
        plt.imshow(matrix, interpolation='nearest', cmap='gray') #Plota a matriz do mapa usando uma escala de cinza
        if self.imap==0:
            plt.title('Original Map')
        else:
            plt.title('Map with Expanded Obstacles')
        plt.show() #Mostra o mapa plotado

    #Função para plotar o caminho sobre o mapa
    def plot_path(self, matrix, path):
        self.get_logger().info('Plotting path on map')
        plt.imshow(matrix, interpolation='nearest', cmap='gray')

        if path:
            for cell in path:
                plt.scatter(x=cell[1], y=cell[0], c='r', s=5) #Plota cada célula no caminho como um ponto vermelho
        plt.title('Path on Map')
        plt.show() #Mostra o mapa plotado

    #Função para converter coordenadas do Gazebo para coordenadas do mapa
    def convert_coordinates(self, x, y, resolution=0.05, origin=(200, 200)):
        #Convertendo as coordenadas do Gazebo para coordenadas do mapa
        map_x = int((x / resolution) + origin[1])
        map_y = int((-y / resolution) + origin[0])
        self.get_logger().info(f'Converting Gazebo Coordinates ({x}, {y}) to Map Coordinates ({map_x}, {map_y})')
        return (map_y, map_x)

    #Função para executar o algoritmo A*
    def run_astar(self):
        #Se a pose atual ainda não foi definida ou se o caminho já foi calculado, retorna
        if self.current_pose is None or self.path_calculated:
            return

        self.path_calculated = True

        matrix = self.load_map('src/projeto_final/projeto_final/map.pgm') #Carrega a matriz do mapa
        if matrix is None:
            rclpy.shutdown() #Se o mapa não foi carregado corretamente, encerra o nó e o programa
            return
        
        self.plot_map(matrix) #Plota a matriz do mapa original
        self.imap = 1

        self.get_logger().info('Expanding obstacles in the map')
        self.get_logger().info('It could take some time. Please wait!!!')
        #Expande os obstáculos na matriz do mapa para garantir uma distância segura
        matrix = expand_obstacles(matrix, expansion_size=12) #Mudar este valor para o valor desejado em dependência da disposição dos obstáculos e do caminho

        self.plot_map(matrix) #Plota a matriz do mapa expandido

        #Converte as coordenadas da pose inicial do robô para coordenadas do mapa
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start = self.convert_coordinates(start_x, start_y)
        self.get_logger().info(f'Starting A* from: {start} (Gazebo: {start_x}, {start_y})')

        #Coordenadas de/ destino no mapa
        end_x = -8
        end_y = -5.5
        end = self.convert_coordinates(end_x, end_y)
        self.get_logger().info(f'Ending A* at: {end} (Gazebo: {end_x}, {end_y})')

        #Verifica se a posição inicial está fora dos limites do mapa
        if start[0] < 0 or start[0] >= matrix.shape[0] or start[1] < 0 or start[1] >= matrix.shape[1]:
            self.get_logger().error(f'Start position {start} is out of map bounds')
            self.destroy_node()
            rclpy.shutdown()
            return
        #Verifica se a posição final está fora dos limites do mapa
        if end[0] < 0 or end[0] >= matrix.shape[0] or end[1] < 0 or end[1] >= matrix.shape[1]:
            self.get_logger().error(f'End position {end} is out of map bounds')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info('Running A* algorithm')
        path = astar(matrix, start, end, self.get_logger()) #Executa o algoritmo A* para encontrar o caminho do início ao fim

        if path:
            #Se o caminho foi encontrado, registra o número de waypoints no caminho
            self.get_logger().info(f'Path found with {len(path)} waypoints')
            #Cria o diretório 'paths' se não existir
            if not os.path.exists('paths'):
                os.makedirs('paths')
            #Salva o caminho encontrado em um arquivo CSV
            path_file = os.path.abspath('paths/path.csv')
            np.savetxt(path_file, path, delimiter=",")
            self.get_logger().info(f'Path saved to {path_file}')
            self.plot_path(matrix, path) #Plota o caminho encontrado no mapa
            self.path_saved = True #Caminho salvo
        else:
            self.get_logger().info('No path found')

        self.destroy_node() #Encerra o nó

def main(args=None):
    rclpy.init(args=args)
    astar_node = PathNode()
    #Criando um executor de thread única e adicionando o nó a ele
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(astar_node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if astar_node.path_saved:
                break
    finally:
        executor.shutdown()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()