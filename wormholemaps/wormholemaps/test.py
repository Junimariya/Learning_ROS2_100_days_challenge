import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

maps = [
    "/home/juni/wormhole_ws/src/wormholemaps/map/junimap.yaml",
    "/home/juni/wormhole_ws/src/wormholemaps/map/maps.yaml"
]


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(LoadMap, '/map_server/load_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = LoadMap.Request()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            qos
        )

    def send_request(self, map_url):
        self.req.map_url = map_url
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Map load response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def listener_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        self.get_logger().info(f"Robot Pose -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")

        if x > 3:
            self.send_request(maps[1])
        else:
            self.send_request(maps[0])


def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
