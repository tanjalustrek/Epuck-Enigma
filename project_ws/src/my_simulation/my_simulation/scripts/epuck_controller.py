import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class EpuckController(Node):
    def __init__(self):
        super().__init__('epuck_controller')

        self.publisher_ = self.create_publisher(Twist, '/epuck/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/epuck/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.target_x = -0.2
        self.target_y = 0.3

        self.distance_tolerance = 0.04
        self.angle_tolerance = 0.01
        self.angular_speed = 0.05

        self.state = 'TURN'
        self.target_angle = None
        self.pause_timer = None
        self.pause_complete = False
        self.first_pause_duration = 10.0
        self.second_pause_duration = 5.0

    def odom_callback(self, msg):
        """Update current position and orientation from Odometry data."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to Euler angles."""
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return 0, 0, yaw_z

    def get_angle_to_target(self):
        """Calculate the correct angle to the target position."""
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        return math.atan2(dx, -dy)

    def normalize_angle(self, angle):
        """Normalize the angle to be within [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def first_pause_done_callback(self):
        """Callback function after the first pause is done."""
        if not self.pause_complete:
            self.pause_complete = True
            self.state = 'YAW_CORRECTION'
            self.get_logger().info('Pause complete. Proceeding to yaw correction.')
            if self.pause_timer is not None:
                self.pause_timer.cancel()

    def second_pause_done_callback(self):
        """Callback function after the second pause is done."""
        self.state = 'MOVE'
        self.get_logger().info('Second pause complete. Proceeding to move.')
        if self.pause_timer is not None:
            self.pause_timer.cancel()

    def stop_robot(self):
        """Send a command to stop the robot."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)

    def timer_callback(self):
        """Main loop controlling the robot's movement."""
        msg = Twist()

        # Calculate distance to the target
        distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)

        if self.state == 'TURN':
            # Calculate the angle to the target
            self.target_angle = self.get_angle_to_target()

            # Calculate the difference between current yaw and target angle
            angle_diff = self.normalize_angle(self.target_angle - self.current_yaw)

            if abs(angle_diff) > self.angle_tolerance:
                # Rotate towards the target angle with reduced angular speed
                msg.angular.z = self.angular_speed * angle_diff / abs(angle_diff)
                self.pause_complete = False
            else:
                # Stop rotating
                self.stop_robot()
                # Start pause timer if not already paused
                if not self.pause_complete and self.pause_timer is None:
                    self.get_logger().info(f'Reached target orientation. Pausing for {self.first_pause_duration} seconds...')
                    self.pause_timer = self.create_timer(self.first_pause_duration, self.first_pause_done_callback)
                    self.state = 'PAUSE'
                    return

        elif self.state == 'YAW_CORRECTION':
            # Recalculate the angle to the target
            self.target_angle = self.get_angle_to_target()
            angle_diff = self.normalize_angle(self.target_angle - self.current_yaw)

            if abs(angle_diff) > self.angle_tolerance:
                # Rotate towards the target angle with reduced angular speed
                msg.angular.z = self.angular_speed * angle_diff / abs(angle_diff)
            else:
                # Stop rotating and start the second pause
                self.stop_robot()
                self.get_logger().info(f'Yaw correction complete. Pausing for {self.second_pause_duration} seconds...')
                self.pause_timer = self.create_timer(self.second_pause_duration, self.second_pause_done_callback)
                self.state = 'SECOND_PAUSE'
                return

        elif self.state == 'MOVE':
            if distance > self.distance_tolerance:
                # Move straight towards the target
                msg.linear.x = 0.2  # Move forward with a constant speed
                msg.angular.z = 0.0  # Ensure no rotation while moving
            else:
                # Stop the robot when the target is reached
                self.state = 'STOP'
                self.get_logger().info('Target reached. Stopping robot.')

        elif self.state == 'STOP':
            self.stop_robot()

        self.publisher_.publish(msg)
        self.get_logger().info(f'State: {self.state}, Position: ({self.current_x:.2f}, {self.current_y:.2f}), '
                               f'Target: ({self.target_x:.2f}, {self.target_y:.2f}), Distance: {distance:.2f}, '
                               f'Yaw: {self.current_yaw:.2f}, Target Angle: {self.target_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EpuckController()
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()


#import rclpy
#from rclpy.node import Node
#from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
#from sensor_msgs.msg import LaserScan
#import math
#
#class EpuckController(Node):
#    def __init__(self):
#        super().__init__('epuck_controller')
#
#        self.publisher_ = self.create_publisher(Twist, '/epuck/cmd_vel', 10)
#        self.subscription = self.create_subscription(Odometry, '/epuck/odom', self.odom_callback, 10)
#        self.laser_subscription = self.create_subscription(LaserScan, '/bocbot/scan', self.laser_callback, 10)
#        self.timer = self.create_timer(0.1, self.timer_callback)
#
#        self.current_x = 0.0
#        self.current_y = 0.0
#        self.current_yaw = 0.0
#
#        self.target_x = -0.2
#        self.target_y = 0.3
#
#        self.distance_tolerance = 0.04
#        self.angle_tolerance = 0.005
#        self.angular_speed = 0.05
#        self.obstacle_distance_threshold = 0.4  # Threshold distance for obstacle detection
#        self.front_angle_range = 40  # Consider obstacles within a +/-30 degree range in front
#
#        self.state = 'TURN'
#        self.target_angle = None
#        self.pause_timer = None
#        self.pause_complete = False
#        self.first_pause_duration = 10.0
#        self.second_pause_duration = 5.0
#
#        self.obstacle_detected = False
#        self.obstacle_avoidance_state = None
#
#    def odom_callback(self, msg):
#        """Update current position and orientation from Odometry data."""
#        self.current_x = msg.pose.pose.position.x
#        self.current_y = msg.pose.pose.position.y
#
#        orientation_q = msg.pose.pose.orientation
#        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
#
#    def laser_callback(self, msg):
#        """Process LaserScan data to detect obstacles."""
#        front_min_index = len(msg.ranges) // 2 - self.front_angle_range // 2
#        front_max_index = len(msg.ranges) // 2 + self.front_angle_range // 2
#        front_distances = msg.ranges[front_min_index:front_max_index]
#
#        min_distance = min(front_distances)  # Get the minimum distance in the front laser scan range
#
#        # Debugging: Log laser data and distance check
#        self.get_logger().info(f'Laser min distance in front: {min_distance:.2f} meters.')
#
#        if min_distance < self.obstacle_distance_threshold:
#            self.obstacle_detected = True
#            self.get_logger().info(f'Obstacle detected at {min_distance:.2f} meters.')
#        else:
#            self.obstacle_detected = False
#
#    def euler_from_quaternion(self, x, y, z, w):
#        """Convert quaternion to Euler angles."""
#        t3 = 2.0 * (w * z + x * y)
#        t4 = 1.0 - 2.0 * (y * y + z * z)
#        yaw_z = math.atan2(t3, t4)
#        return 0, 0, yaw_z
#
#    def get_angle_to_target(self):
#        """Calculate the correct angle to the target position."""
#        dx = self.target_x - self.current_x
#        dy = self.target_y - self.current_y
#        return math.atan2(dx, -dy)
#
#    def normalize_angle(self, angle):
#        """Normalize the angle to be within [-pi, pi]."""
#        while angle > math.pi:
#            angle -= 2 * math.pi
#        while angle < -math.pi:
#            angle += 2 * math.pi
#        return angle
#
#    def first_pause_done_callback(self):
#        """Callback function after the first pause is done."""
#        if not self.pause_complete:
#            self.pause_complete = True
#            self.state = 'YAW_CORRECTION'
#            self.get_logger().info('Pause complete. Proceeding to yaw correction.')
#            if self.pause_timer is not None:
#                self.pause_timer.cancel()
#
#    def second_pause_done_callback(self):
#        """Callback function after the second pause is done."""
#        self.state = 'MOVE'
#        self.get_logger().info('Second pause complete. Proceeding to move.')
#        if self.pause_timer is not None:
#            self.pause_timer.cancel()
#
#    def stop_robot(self):
#        """Send a command to stop the robot."""
#        stop_msg = Twist()
#        stop_msg.linear.x = 0.0
#        stop_msg.angular.z = 0.0
#        self.publisher_.publish(stop_msg)
#
#    def timer_callback(self):
#        """Main loop controlling the robot's movement."""
#        msg = Twist()
#
#        # Calculate distance to the target
#        distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
#
#        if self.state == 'MOVE' and self.obstacle_detected:
#            if self.obstacle_avoidance_state is None:
#                self.get_logger().info('Obstacle avoidance initiated.')
#                self.obstacle_avoidance_state = 'TURN'
#
#            if self.obstacle_avoidance_state == 'TURN':
#                # Rotate to avoid the obstacle
#                msg.angular.z = 0.3  # Turn to avoid obstacle
#                msg.linear.x = 0.0
#                self.obstacle_avoidance_state = 'MOVING_AWAY'
#
#            elif self.obstacle_avoidance_state == 'MOVING_AWAY':
#                # After turning, move forward for a short time
#                msg.angular.z = 0.0
#                msg.linear.x = 0.2
#
#                # If the obstacle is cleared, resume normal movement
#                if not self.obstacle_detected:
#                    self.obstacle_avoidance_state = None  # Reset obstacle avoidance state
#
#        else:
#            self.obstacle_avoidance_state = None  # Reset obstacle avoidance state
#
#            if self.state == 'TURN':
#                # Calculate the angle to the target
#                self.target_angle = self.get_angle_to_target()
#               
#                # Calculate the difference between current yaw and target angle
#                angle_diff = self.normalize_angle(self.target_angle - self.current_yaw)
#               
#                if abs(angle_diff) > self.angle_tolerance:
#                    # Rotate towards the target angle with reduced angular speed
#                    msg.angular.z = self.angular_speed * angle_diff / abs(angle_diff)
#                    self.pause_complete = False
#                else:
#                    # Stop rotating
#                    self.stop_robot()
#                    # Start pause timer if not already paused
#                    if not self.pause_complete and self.pause_timer is None:
#                        self.get_logger().info(f'Reached target orientation. Pausing for {self.first_pause_duration} seconds...')
#                        self.pause_timer = self.create_timer(self.first_pause_duration, self.first_pause_done_callback)
#                        self.state = 'PAUSE'
#                        return
#
#            elif self.state == 'YAW_CORRECTION':
#                # Recalculate the angle to the target
#                self.target_angle = self.get_angle_to_target()
#                angle_diff = self.normalize_angle(self.target_angle - self.current_yaw)
#               
#                if abs(angle_diff) > self.angle_tolerance:
#                    # Rotate towards the target angle with reduced angular speed
#                    msg.angular.z = self.angular_speed * angle_diff / abs(angle_diff)
#                else:
#                    # Stop rotating and start the second pause
#                    self.stop_robot()
#                    self.get_logger().info(f'Yaw correction complete. Pausing for {self.second_pause_duration} seconds...')
#                    self.pause_timer = self.create_timer(self.second_pause_duration, self.second_pause_done_callback)
#                    self.state = 'SECOND_PAUSE'
#                    return
#
#            elif self.state == 'MOVE':
#                if distance > self.distance_tolerance:
#                    # Move straight towards the target
#                    msg.linear.x = 0.2  # Move forward with a constant speed
#                    msg.angular.z = 0.0  # Ensure no rotation while moving
#                else:
#                    # Stop the robot when the target is reached
#                    self.state = 'STOP'
#                    self.get_logger().info('Target reached. Stopping robot.')
#
#            elif self.state == 'STOP':
#                self.stop_robot()
#
#        self.publisher_.publish(msg)
#        # Safe handling if target_angle is None
#        target_angle_str = f'{self.target_angle:.2f}' if self.target_angle is not None else 'None'
#        self.get_logger().info(f'State: {self.state}, Position: ({self.current_x:.2f}, {self.current_y:.2f}), '
#                               f'Target: ({self.target_x:.2f}, {self.target_y:.2f}), Distance: {distance:.2f}, '
#                               f'Yaw: {self.current_yaw:.2f}, Target Angle: {target_angle_str}')
#
#def main(args=None):
#    rclpy.init(args=args)
#    epuck_controller = EpuckController()
#    rclpy.spin(epuck_controller)
#    epuck_controller.destroy_node()
#    rclpy.shutdown()
#
#if __name__ == '__main__':
#    main()