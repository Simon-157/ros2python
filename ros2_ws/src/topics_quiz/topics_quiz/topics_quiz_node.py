import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np


class TopicsQuizNode(Node):

    def __init__(self):
        super().__init__('topics_quiz_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.is_parallel = False
        self.is_rotated = False
        self.initial_x = None 

    def listener_callback(self, msg):
            if self.initial_x is None:
                self.initial_x = msg.pose.pose.position.x

            current_distance = abs(msg.pose.pose.position.x - self.initial_x)

            if current_distance <  2.8:
                # Move the robot forward for a certain distance
                twist_msg = Twist()
                twist_msg.linear.x = 0.5  # Adjust the linear velocity as needed
                self.publisher_.publish(twist_msg)
            else:
                if not self.is_parallel:
                    # Perform the parallel logic here
                    if self.check_parallel_condition(msg):
                        self.is_parallel = True

                elif self.is_parallel and not self.is_rotated:
                    # Rotate the robot 90 degrees in the direction of the opening
                    twist_msg = Twist()
                    twist_msg.angular.z = 0.25  # Adjust the angular velocity as needed
                    self.publisher_.publish(twist_msg)

                    if self.check_rotation_condition(msg):
                        self.is_rotated = True

                elif self.is_rotated:
                    # Move the robot forward until it goes past the opening
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.5  # Adjust the linear velocity as needed
                    self.publisher_.publish(twist_msg)

                    if self.check_past_opening_condition(msg):
                        twist_msg.linear.x = 0.0  # Stop the robot
                        self.publisher_.publish(twist_msg)


    def check_parallel_condition(self, msg):
        roll, pitch, yaw = self.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        # Check if the robot is parallel with the opening
        return abs(yaw) < 0.1  # Adjust the threshold as needed

    def check_rotation_condition(self, msg):
        roll, pitch, yaw = self.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        # Check if the robot has rotated 90 degrees
        return abs(abs(yaw) - np.pi/2) < 0.1  # Adjust the threshold as needed

    def check_past_opening_condition(self, msg):
        opening_x_coordinate = 5.0  # Adjust this value to the x-coordinate of the opening
        opening_y_coordinate = 0.0  # Adjust this value to the y-coordinate of the opening
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Check if the robot has gone past the opening
        return current_x > opening_x_coordinate and current_y == opening_y_coordinate


    def euler_from_quaternion(self, quaternion):
        # Implement the provided euler_from_quaternion function here
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)

    topics_quiz_node = TopicsQuizNode()

    rclpy.spin(topics_quiz_node)

    topics_quiz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
