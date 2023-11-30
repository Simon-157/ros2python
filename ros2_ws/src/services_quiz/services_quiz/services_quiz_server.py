import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn  # Change 'services_quiz_srv' to the actual package name
import time

class SpinningServiceServer(Node):

    def __init__(self):
        super().__init__('services_quiz_server')
        self.srv = self.create_service(Turn, 'turn', self.spin_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # Adjust the topic 'cmd_vel' according to your robot's motion control topic

    def spin_callback(self, request, response):
        direction = request.direction
        angular_velocity = request.angular_velocity
        time_duration = request.time

        twist = Twist()
        if direction.lower() == 'right':
            # Spin right
            print(f"Spinning to the right at {angular_velocity} rad/s for {time_duration} seconds.")
            twist.angular.z = -abs(angular_velocity)  # Set the angular velocity to rotate to the right

        elif direction.lower() == 'left':
            # Spin left
            print(f"Spinning to the left at {angular_velocity} rad/s for {time_duration} seconds.")
            twist.angular.z = abs(angular_velocity)  # Set the angular velocity to rotate to the left

        else:
            # Invalid direction provided
            print("Invalid direction provided. Please provide 'right' or 'left'.")
            response.success = False
            return response

        self.publisher_.publish(twist)  # Publish the Twist message to initiate the robot's spinning movement

        time.sleep(time_duration)  # Wait for the specified duration

        stop_twist = Twist()  # Create a Twist message to stop the robot's movement
        self.publisher_.publish(stop_twist)  # Publish the stop command to halt the robot's movement

        response.success = True  # Set the success variable to True if everything went okay
        return response

def main(args=None):
    rclpy.init(args=args)
    services_quiz_server = SpinningServiceServer()
    rclpy.spin(services_quiz_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
