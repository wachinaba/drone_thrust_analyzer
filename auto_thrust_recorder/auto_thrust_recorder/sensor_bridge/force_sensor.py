import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty

class ForceSensor(Node):
    def __init__(self):
        super().__init__('force_sensor_subscriber')
        self.force_subscriber = self.create_subscription(
            WrenchStamped, "/force_sensor_node/data", self.force_callback, 10)
        self.set_offset_client_ = self.create_client(Empty, '/force_sensor_node/set_offset')
        self.force = WrenchStamped()

        self.on_sensor_update_callback = None

    def force_callback(self, msg: WrenchStamped):
        self.force = msg
        if self.on_sensor_update_callback:
            self.on_sensor_update_callback(msg)

    def set_on_sensor_update(self, callback):
        self.on_sensor_update_callback = callback

    def get_force(self) -> WrenchStamped:
        return self.force

    def set_sensor_offset(self):
        """Service callback to set the force sensor offset."""
        self.get_logger().info('Setting force sensor offset')
        self.set_offset_client_.call_async(Empty.Request())
        return