import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Twist


class Robot:
  def __init__(self, position, velocity):
    self.position = np.array(position, dtype=float)
    self.velocity = np.array(velocity, dtype=float)

  def update(self, delta_time):
    self.position += self.velocity * delta_time

# Função de força atrativa em direção ao objetivo
def attractive_force(robot_position, goal_position, alpha=1):
  return (alpha) * goal_position - robot_position

# Função de força repulsiva em relação a um obstáculo
def repulsive_force(robot_position, obstacle_position, beta=1, gamma=1):
  distance = np.linalg.norm(robot_position - obstacle_position)
  if distance < 40:
    beta *= 180.0
    gamma *= 300.0
  direction = (robot_position - obstacle_position)
  # Componente repulsiva
  repulsive_component = beta * direction/distance**gamma

  # Componente tangencial para contornar o obstáculo
  tangential_component = gamma * (1.0 / distance) * np.array([-direction[1], direction[0]])

  # Ponderação da componente tangencial
  tangential_component *= max(1.0 - np.exp(-distance**2 / (2 * (distance/2)**2)), 1.0)

  # Força total
  total_force = repulsive_component + tangential_component

  return total_force

# Função que calcula a força total considerando forças atrativas e repulsivas
def calculate_total_force(robot_position, goal_position, obstacles):
  total_force = attractive_force(robot_position, goal_position)

  for obstacle in obstacles:
    total_force += repulsive_force(robot_position, obstacle)

  # Normaliza a força total
  total_force /= np.linalg.norm(total_force) + 1e-5

  return total_force

# Função que verifica se a posição é válida, ou seja, se não colide com obstáculos
def is_valid_position(position, obstacles):
  return all(np.linalg.norm(position - obstacle) >= 5 for obstacle in obstacles)


class RobotNavigationNode(Node):
    def __init__(self):
        super().__init__('robot_navigation')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot = Robot(position=[0, 0], velocity=[0, 0])
        self.goal = np.array([50, 50])
        self.obstacles = np.array([[25, 0], [30, 0], [400, 400]])
        self.max_speed = 10.0
        self.max_acceleration = 1.0
        self.timer = self.create_timer(0.1, self.update_callback)

    def update_callback(self):
        force = calculate_total_force(self.robot.position, self.goal, self.obstacles)
        acceleration = force - self.robot.velocity
        acceleration_magnitude = np.linalg.norm(acceleration)
        if acceleration_magnitude > self.max_acceleration:
            acceleration = acceleration * (self.max_acceleration / acceleration_magnitude)
        
        self.robot.velocity += acceleration * 0.1
        speed = np.linalg.norm(self.robot.velocity)
        if speed > self.max_speed:
            self.robot.velocity = self.robot.velocity * (self.max_speed / speed)

        self.robot.update(0.1)

        # Create and publish Twist message
        vel_msg = Twist()
        vel_msg.linear.x = float(self.robot.velocity[0])/10
        vel_msg.linear.y = float(self.robot.velocity[1])/10
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(vel_msg)

        # Debugging Information
        print(f"Position: {self.robot.position}, Velocity: {self.robot.velocity}, Speed: {speed}")

        if np.linalg.norm(self.robot.position - self.goal) < 5 and speed < 1.0:
            self.robot.velocity = np.array([0.0, 0.0])
            

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
