import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('services_quiz_client')

    client = node.create_client(Turn, 'turn')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = Turn.Request()
    request.direction = 'right'  # Set the direction to 'right'
    request.angular_velocity = 0.2  # Set the angular velocity to 0.2 rad/s
    request.time = 10  # Set the time duration to 10 seconds

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Response: %r' % future.result().success)
    else:
        node.get_logger().error('Service call failed %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
