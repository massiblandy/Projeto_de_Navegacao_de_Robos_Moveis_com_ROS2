#De robotcontrol.py
def follow_path(self):
        if self.path is None or self.path.size == 0 or self.current_pose is None or self.current_goal_index >= len(self.path):
            self.get_logger().info('Path is None or current_pose is None or path completed')
            return

        # Extraindo posição e orientação atuais
        position = self.current_pose.position
        orientation = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        goal_map = self.path[self.current_goal_index]
        goal_x, goal_y = self.convert_coordinates_back(goal_map[1], goal_map[0])
        dx = goal_x - position.x
        dy = goal_y - position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = np.arctan2(dy, dx)

        self.get_logger().info(f'Current Position: x={position.x}, y={position.y}, yaw={yaw}')
        self.get_logger().info(f'Goal Position: x={goal_x}, y={goal_y}')
        self.get_logger().info(f'Distance to Goal: {distance}, Angle to Goal: {angle_to_goal}')

        # Controlando o robô para seguir o caminho
        cmd_vel = Twist()

        # Se um obstáculo for detectado diretamente à frente, o robô deve parar e ajustar a direção
        if self.closest_distance_front < self.obstacle_distance:
            self.get_logger().info('Obstacle detected ahead!')
            if self.closest_distance_right < self.closest_distance_left:
                cmd_vel = self.curva_esquerda
            else:
                cmd_vel = self.curva_direita
        else:
            if distance > 1.5:
                angular_error = angle_to_goal - yaw
                if abs(angular_error) > 0.1:  # Se o erro angular for significativo, primeiro alinhe o robô
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 1.0 * angular_error  # Ajuste da velocidade angular
                else:
                    cmd_vel.linear.x = 0.2  # Velocidade linear reduzida para maior precisão
                    cmd_vel.angular.z = 0.5 * angular_error  # Ajuste da velocidade angular durante o movimento
            else:
                self.current_goal_index += 1
                self.get_logger().info(f'Moving to next waypoint: {self.current_goal_index}')

        self.pub_cmd_vel.publish(cmd_vel)
