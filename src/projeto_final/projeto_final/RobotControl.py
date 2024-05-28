from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10) #Cria um publisher para o tópico /cmd_vel
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10) #Cria um subscriber para o tópico /scan
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) #Cria um subscriber para o tópico /odom
        self.path = self.load_path('paths/path.csv') #Carrega o caminho salvo de um arquivo CSV
        self.current_pose = None #Inicializa a pose atual
        self.current_goal_index = 0 #Índice do ponto atual no caminho
        self.goal_reached = False #Adiciona um estado para checar se o objetivo final foi atingido

    def laser_callback(self, msg):
        self.laser = msg.ranges #Armazena os dados do sensor laser

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose #Atualiza a pose atual com os dados de odometria
        self.follow_path()
    
    #Função para carregar o caminho
    def load_path(self, file_path):
        try:
            path = np.loadtxt(file_path, delimiter=",")
            self.get_logger().info(f'Path loaded from {file_path}')
            return path
        except Exception as e:
            self.get_logger().error(f'Failed to load path: {e}')
            return None
    
    #Função que converte as coordenadas do mapa de volta para as coordenadas do Gazebo, considerando que o mapa está rotacionado 90 graus para a direita em relação ao ambiente de Gazebo
    def convert_coordinates_back(self, map_x, map_y, resolution=0.05, origin=(200, 200)):
        gazebo_x = (map_x - origin[0]) * resolution
        gazebo_y = (origin[1] - map_y) * resolution
        self.get_logger().info(f'Converting Map Coordinates ({map_x}, {map_y}) to Gazebo Coordinates ({gazebo_x}, {gazebo_y})')
        return gazebo_x, gazebo_y

    #Função para fazer o robô seguir o caminho calculado pelo algoritmo A*
    def follow_path(self):
        if self.path is None or self.path.size == 0 or self.current_pose is None or self.current_goal_index >= len(self.path):
            self.get_logger().info('Path is None or current_pose is None or path completed')
            return

        #Extraindo posição e orientação atuais do robô
        position = self.current_pose.position #Pega a posição atual do robô
        orientation = self.current_pose.orientation #Pega a orientação atual do robô
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) #Converte a orientação de quaternion para ângulos de Euler

        goal_map = self.path[self.current_goal_index] #Pega o próximo ponto do caminho
        goal_x, goal_y = self.convert_coordinates_back(goal_map[1], goal_map[0]) #Converte as coordenadas do mapa de volta para as coordenadas do Gazebo
        dx = goal_x - position.x #Calcula a diferença x
        dy = goal_y - position.y #Calcula a diferença y
        distance = np.sqrt(dx ** 2 + dy ** 2) #Calcula a distância até o objetivo
        angle_to_goal = np.arctan2(dy, dx) #Calcula o ângulo até o objetivo

        self.get_logger().info(f'Current Position: x={position.x}, y={position.y}, yaw={yaw}')
        self.get_logger().info(f'Goal Position: x={goal_x}, y={goal_y}')
        self.get_logger().info(f'Distance to Goal: {distance}, Angle to Goal: {angle_to_goal}')

        #Controlando o robô para seguir o caminho
        cmd_vel = Twist() #Inicializa a mensagem Twist para controlar o robô
        angular_error = angle_to_goal - yaw #Calcula o erro angular
        #Ajustando o erro angular
        if angular_error > np.pi:
            angular_error -= 2 * np.pi
        elif angular_error < -np.pi:
            angular_error += 2 * np.pi

        if distance > 0.1:
            if abs(angular_error) > 0.1: #Se o erro angular for significativo -> alinhar o robô
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 1.0 * angular_error #Ajuste da velocidade angular
            else:
                cmd_vel.linear.x = 0.2 #Velocidade linear reduzida para maior precisão
                cmd_vel.angular.z = 0.5 * angular_error #Ajuste da velocidade angular durante o movimento
        else:
            self.current_goal_index += 1 #Incrementa o índice do ponto atual
            if self.current_goal_index >= len(self.path): #Verificando se é o último ponto
                self.get_logger().info('ARRIVED AT FINAL POSITION!!!! :)')
                self.pub_cmd_vel.publish(Twist()) #Parar robô
                self.goal_reached = True

        self.pub_cmd_vel.publish(cmd_vel) #Publica a mensagem de velocidade

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    executor = rclpy.executors.SingleThreadedExecutor() #Executor de thread única
    executor.add_node(robot_control) #Adiciona o nó ao executor
    try:
        while rclpy.ok() and not robot_control.goal_reached:
            executor.spin_once(timeout_sec=1.0) #Executa o nó até que o objetivo seja alcançado ou o ROS2 seja desligado
    except KeyboardInterrupt:
        pass
    finally:
        robot_control.destroy_node() #Destroindo o nó
        rclpy.shutdown() #Encerrando ROS2

if __name__ == '__main__':
    main()
