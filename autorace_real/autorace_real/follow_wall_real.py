import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import threading


class FollowWallReal(Node):
    def __init__(self):
        super().__init__('follow_wall_real')

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.front = 0.0
        self.ur = 0.0
        self.mr = 0.0
        self.lr = 0.0

        self.state = 'FORWARD'
        self.lock = threading.Lock()

        self.create_timer(0.1, self.control_loop)

    def get_range(self, scan, angle_deg):
        angle_rad = angle_deg * 3.14159 / 180.0
        index = int((angle_rad - scan.angle_min) / scan.angle_increment)
        index = max(0, min(index, len(scan.ranges) - 1))
        dist = scan.ranges[index]
        return dist if dist > 0.01 else 0.0  # Treat 0.0 as no detection (∞)

    def lidar_callback(self, msg):
        with self.lock:
            self.front = self.get_range(msg, 0)
            self.ur = self.get_range(msg, 315)
            self.mr = self.get_range(msg, 270)
            self.lr = self.get_range(msg, 225)

        self.get_logger().info(f"FRONT: {self.front:.2f}, UR: {self.ur:.2f}, MR: {self.mr:.2f}, LR: {self.lr:.2f}")

    def control_loop(self):
        with self.lock:
            front = self.front
            ur = self.ur
            mr = self.mr
            lr = self.lr

        twist = Twist()

        # === State Transitions ===
        if self.state != 'TURN_LEFT' and front < 0.3 and front != 0.0:
            self.state = 'TURN_LEFT'
        elif self.state == 'TURN_LEFT':
            # Exit TURN_LEFT only when wall is no longer in front and MR sees a wall
            if front > 0.4 or front == 0.0:
                if mr != 0.0 and mr < 0.5:
                    self.state = 'FOLLOW_RIGHT'
        elif ur == 0.0 and mr == 0.0:
            self.state = 'SEARCH_RIGHT'
        elif abs(ur - lr) > 0.3 and ur != 0.0 and lr != 0.0:
            self.state = 'FOLLOW_RIGHT'
        else:
            self.state = 'FORWARD'

        self.get_logger().info(f"State: {self.state}")

        # === Actions ===
        if self.state == 'TURN_LEFT':
            twist.angular.z = 0.5
        elif self.state == 'SEARCH_RIGHT':
            twist.linear.x = 0.05
        elif self.state == 'FOLLOW_RIGHT':
            error = lr - ur
            twist.linear.x = 0.1
            twist.angular.z = error * 0.8
        elif self.state == 'FORWARD':
            twist.linear.x = 0.15

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowWallReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
