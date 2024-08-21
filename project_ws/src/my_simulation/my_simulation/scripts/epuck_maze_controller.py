import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math
import os
from ament_index_python.packages import get_package_share_directory
import sys

# Get the scripts directory
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from maze_path import astar

class EPuckMazeController(Node):
    '''Moves an Epuck from the start point to the goal trough a maze.'''
    def __init__(self):
        super().__init__('epuck_maze_controller')
        self.publisher = self.create_publisher(Twist, 'epuck/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'epuck/odom', self.odom_callback, 10)
        
        # Robot state
        self.x = None
        self.y = None
        self.theta = None
        
        # Path planning
        self.goal = (-0.55, -1.2)
        self.path = None
        self.world_path = None
        self.current_target = 0
        
        self.initial_position_received = False
        self.grid_resolution = 0.01

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation
        (_, _, self.theta) = quat2euler([orient.w, orient.x, orient.y, orient.z])

        if not self.initial_position_received:
            self.initial_position_received = True
            self.plan_path()
            self.timer = self.create_timer(0.1, self.move_robot)

    def world_to_grid(self, world_x, world_y):
        '''Converts world coordinates to grid coordinates.'''
        grid_x = int((world_x + 0.8) / self.grid_resolution)
        grid_y = int((world_y + 1.49) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, path):
        '''Converts grid coordinates to world coordinates.'''
        world_path = []
        for point in path:
            world_x = point[0] * self.grid_resolution - 0.8
            world_y = point[1] * self.grid_resolution - 1.49 - 0.05
            world_path.append((world_x, world_y))
        return world_path

    def plan_path(self):
        '''Converts the grid path to the world path.'''
        start_grid = self.world_to_grid(self.x, self.y)
        goal_grid = self.world_to_grid(self.goal[0], self.goal[1])
        self.path = astar(start_grid, goal_grid)
        if self.path:
            self.world_path = self.grid_to_world(self.path)
        else:
            self.get_logger().error('No path found')

    def normalize_angle(self, angle):
        '''Normalize the angle to be within [-pi, pi].'''
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        '''Sends a command to stop the Epuck.'''
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

    def move_robot(self):
        '''Moves the Epuck.'''
        if not self.world_path:
            self.get_logger().warn('No path available')
            return

        if self.current_target >= len(self.world_path):
            self.stop_robot()
            self.get_logger().info('Goal reached!')
            return

        target = self.world_path[self.current_target]
        dx = target[0] - self.x
        dy = target[1] - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        self.get_logger().info(f'Current: ({self.x}, {self.y})')
        self.get_logger().info(f'Target: {target}, Distance: {distance}')

        if distance < 0.05:
            self.current_target += 1
            return

        desired_angle = math.atan2(dx, -dy)
        angle_diff = desired_angle - self.theta
        angle_diff = self.normalize_angle(angle_diff)

        msg = Twist()
        if abs(angle_diff) > 0.05:
            msg.angular.z = 0.05 if angle_diff > 0 else -0.05
        else:
            msg.linear.x = 0.10
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    epuck_maze_controller = EPuckMazeController()
    rclpy.spin(epuck_maze_controller)
    epuck_maze_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()