# Node that subscribes to /amcl_pose and prints x, y, z of the robot in the map frame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class AMCLSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_subscriber')

        # QoS to match AMCL (TRANSIENT_LOCAL + RELIABLE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribe to /amcl_pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,  # Message type
            '/amcl_pose',               # Topic name
            self.listener_callback,     # Callback
            qos
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z  # usually 0 for ground robots
        self.get_logger().info(f"Robot Pose -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AMCLSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

