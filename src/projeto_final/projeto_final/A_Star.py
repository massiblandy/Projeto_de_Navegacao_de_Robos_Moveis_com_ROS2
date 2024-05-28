import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import heapq
import numpy as np
from matplotlib import pyplot as plt
import os

class NodeAStar:
    def __init__(self, position, parent, g=0, h=0, f=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = f

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(matrix, start, end, logger):
    start_node = NodeAStar(start, None)
    end_node = NodeAStar(end, None)

    open_list = []
    closed_list = []

    heapq.heappush(open_list, start_node)

    iteration_count = 0
    while open_list:
        iteration_count += 1
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        if iteration_count % 100 == 0:
            logger.info(f'Iteration {iteration_count}: Current node position: {current_node.position}')

        if current_node == end_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            logger.info(f'Path found after {iteration_count} iterations')
            return path[::-1]

        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)]

        for next in neighbors:
            if next[0] > (len(matrix) - 1) or next[0] < 0 or next[1] > (len(matrix[0]) - 1) or next[1] < 0:
                continue

            if matrix[next[0]][next[1]] == 0:
                logger.debug(f'Skipping obstacle at {next}')
                continue

            child = NodeAStar(next, current_node)

            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = heuristic(child.position, end_node.position)
            child.f = child.g + child.h

            if add_to_open(open_list, child):
                heapq.heappush(open_list, child)
                logger.debug(f'Adding {next} to open list with f={child.f}')

    logger.info('No path found')
    return None

def add_to_open(open_list, child):
    for node in open_list:
        if child == node and child.g >= node.g:
            return False
    return True

def expand_obstacles(matrix, expansion_size):
    expanded_matrix = np.copy(matrix)
    obstacle_indices = np.argwhere(matrix == 0)

    for idx in obstacle_indices:
        for x in range(-expansion_size, expansion_size+1):
            for y in range(-expansion_size, expansion_size+1):
                new_x, new_y = idx[0] + x, idx[1] + y
                if 0 <= new_x < matrix.shape[0] and 0 <= new_y < matrix.shape[1]:
                    expanded_matrix[new_x, new_y] = 0

    return expanded_matrix

class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_node')
        self.get_logger().info('AStar Node has been started.')
        self.current_pose = None
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_calculated = False
        self.path_saved = False

    def odom_callback(self, msg):
        if self.current_pose is None:
            self.current_pose = msg.pose.pose
            self.get_logger().info(f'Received initial pose: x={self.current_pose.position.x}, y={self.current_pose.position.y}')
            self.run_astar()

    def load_map(self, file_path):
        self.get_logger().info(f'Loading map from {file_path}')
        try:
            pgmf = open(file_path, 'rb')
            matrix = plt.imread(pgmf)
            matrix = 1.0 * (matrix > 250)
            self.get_logger().info(f'Map loaded with shape {matrix.shape}')
            return matrix
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            return None

    def plot_map(self, matrix):
        self.get_logger().info('Plotting map')
        plt.imshow(matrix, interpolation='nearest', cmap='gray')
        plt.show()

    def plot_path(self, matrix, path):
        self.get_logger().info('Plotting path on map')
        plt.imshow(matrix, interpolation='nearest', cmap='gray')
        if path:
            for cell in path:
                plt.scatter(x=cell[1], y=cell[0], c='r', s=5)
        plt.show()

    def convert_coordinates(self, x, y, resolution=0.05, origin=(200, 200)):
        map_x = int((x / resolution) + origin[1])
        map_y = int((-y / resolution) + origin[0])
        self.get_logger().info(f'Converting Gazebo Coordinates ({x}, {y}) to Map Coordinates ({map_x}, {map_y})')
        return (map_y, map_x)

    def run_astar(self):
        if self.current_pose is None or self.path_calculated:
            return

        self.path_calculated = True

        matrix = self.load_map('src/map.pgm')
        if matrix is None:
            rclpy.shutdown()
            return

        matrix = expand_obstacles(matrix, expansion_size=11)  # Ajuste para garantir uma distância segura dos obstáculos

        self.plot_map(matrix)

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start = self.convert_coordinates(start_x, start_y)
        self.get_logger().info(f'Starting A* from: {start} (Gazebo: {start_x}, {start_y})')

        end_x = 7
        end_y = 6.5
        end = self.convert_coordinates(end_x, end_y)
        self.get_logger().info(f'Ending A* at: {end} (Gazebo: {end_x}, {end_y})')

        if start[0] < 0 or start[0] >= matrix.shape[0] or start[1] < 0 or start[1] >= matrix.shape[1]:
            self.get_logger().error(f'Start position {start} is out of map bounds')
            self.destroy_node()
            rclpy.shutdown()
            return
        if end[0] < 0 or end[0] >= matrix.shape[0] or end[1] < 0 or end[1] >= matrix.shape[1]:
            self.get_logger().error(f'End position {end} is out of map bounds')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info('Running A* algorithm')
        path = astar(matrix, start, end, self.get_logger())

        if path:
            self.get_logger().info(f'Path found with {len(path)} waypoints')
            if not os.path.exists('paths'):
                os.makedirs('paths')
            path_file = os.path.abspath('paths/path.csv')
            np.savetxt(path_file, path, delimiter=",")
            self.get_logger().info(f'Path saved to {path_file}')
            self.plot_path(matrix, path)
            self.path_saved = True
        else:
            self.get_logger().info('No path found')

        self.get_logger().info('Shutting down node')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    astar_node = AStarNode()

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
