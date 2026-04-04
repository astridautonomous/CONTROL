import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
# Not: Gerçek tipiniz farklı olsa bile 'generic_subscription' 
# mesaj geldiğini anlamak için yeterlidir. 
# Ancak test için String kullanalım.

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher_node')
        
        # Test etmek istediğimiz bir kaç topic seçelim
        self.topics_to_publish = [
            "/astrid/perception/traffic_sign",
            "/astrid/slam/local_map",
            "/astrid/navigation/global_path",
            "/scan"
        ]
        
        self.publishers_ = {}
        for topic in self.topics_to_publish:
            # Watchdog'un discovery mekanizması için tip önemli değil, 
            # sadece yayın olması yeterli.
            self.publishers_[topic] = self.create_publisher(String, topic, 10)
            
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Mock Publisher başladı. 4 topic yayınlanıyor...")

    def timer_callback(self):
        msg = String()
        for topic, pub in self.publishers_.items():
            msg.data = f"Test verisi: {self.get_clock().now().to_msg().nanosec}"
            pub.publish(msg)
            # self.get_logger().info(f"Yayınlandı: {topic}")

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()