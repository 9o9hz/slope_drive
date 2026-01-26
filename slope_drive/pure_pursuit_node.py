import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
# from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32 # For steering
from std_msgs.msg import Float64 # For velocity


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.get_logger().info('Pure Pursuit Node has been started.')

        # This node is optional if the path_planner already outputs commands.
        # If used, it would subscribe to a path and output velocity/steering commands.

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)
        
        # This could also subscribe to a target point directly
        self.target_sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.target_callback,
            10)

        # Publishers for vehicle commands
        self.steer_pub = self.create_publisher(Float32, '/cmd_steer', 10)
        self.vel_pub = self.create_publisher(Float64, '/cmd_vel', 10)
        # Alternatively, a single message type can be used:
        # self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)


    def path_callback(self, msg):
        self.get_logger().info(f'Received Path with {len(msg.poses)} poses.')
        # Pure pursuit logic would go here:
        # 1. Find the current vehicle pose (e.g., from /tf or /odom)
        # 2. Find the lookahead point on the received path
        # 3. Calculate the required steering angle to intercept that point
        # 4. Publish steering and velocity commands

    def target_callback(self, msg):
        self.get_logger().info(f'Received Target Point: {msg.point.x}, {msg.point.y}')
        # Simplified control logic if a target point is provided directly
        # Calculate steering angle based on the target point's position
        # relative to the vehicle's frame
        steer_cmd = Float32()
        steer_cmd.data = 0.0 # Placeholder
        self.steer_pub.publish(steer_cmd)

        vel_cmd = Float64()
        vel_cmd.data = 0.5 # m/s, placeholder
        self.vel_pub.publish(vel_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
