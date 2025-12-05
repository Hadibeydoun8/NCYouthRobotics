import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ultralytics import YOLO
from cv_bridge import CvBridge


from ament_index_python.packages import get_package_share_directory
import os

from sensor_msgs.msg import Image

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscriber_ = self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)


        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info(os.getcwd())

        pkg_share = get_package_share_directory('robot_perception')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')

        self.model = YOLO(model_path)
        self.bridge = CvBridge()


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def image_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.header.stamp)
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ObjectDetection()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()