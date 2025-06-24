import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class StartSignalReal(Node):
    def __init__(self):
        super().__init__('start_signal_real')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.safe_distance = 0.3         # distance to stop for wall
        self.obstacle_distance = 0.1     # distance to wait for obstacle removal

        self.state = "WAITING"  # Initial state

    def lidar_callback(self, msg):
        front_index = 0
        front_distance = msg.ranges[front_index]

        self.get_logger().info(f"Current state: {self.state}")
        self.get_logger().info(f"LIDAR distance: {front_distance:.2f} m")

        if self.state == "WAITING":
            if front_distance >= self.obstacle_distance and front_distance >= self.safe_distance:
                self.get_logger().info("Obstacle removed. Starting to drive.")
                self.drive_forward()
                self.state = "DRIVING"
            else:
                self.get_logger().info("Waiting for obstacle to be removed.")
                self.stop()

        elif self.state == "DRIVING":
            if front_distance < self.safe_distance:
                self.get_logger().info("Wall detected. Stopping.")
                self.stop()
                self.state = "STOPPED"

        elif self.state == "STOPPED":
            if front_distance >= self.safe_distance:
                self.get_logger().info("Wall cleared. Driving forward again.")
                self.drive_forward()
                self.state = "DRIVING"
            else:
                self.get_logger().info("Still too close to wall. Staying stopped.")
                self.stop()

    def drive_forward(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.publisher_.publish(msg)

    def stop(self):
        msg = Twist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StartSignalReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
