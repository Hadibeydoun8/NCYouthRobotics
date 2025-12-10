import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import cv2
import os


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        # Publisher for annotated images
        self.annotated_pub = self.create_publisher(Image, 'camera/image_annotated', 10)
        # Optional publisher for detected class names
        self.class_pub = self.create_publisher(String, 'detected_classes', 10)

        # Subscriber to raw camera images
        self.subscriber_ = self.create_subscription(Image, "robot/D415/color/image_raw", self.image_callback, 10)

        # YOLO model
        pkg_share = get_package_share_directory('robot_perception')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')
        self.model = YOLO(model_path)

        # CvBridge instance
        self.bridge = CvBridge()

        self.get_logger().info(f"ObjectDetection node initialized. Using model: {model_path}")

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # FIX: Rotate camera feed 180 degrees (invert image)
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        # Run YOLO detection
        results = self.model(cv_image)

        # Annotate image
        annotated_image = results[0].plot()

        # Publish annotated image
        try:
            ros_annotated = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            ros_annotated.header = msg.header
            self.annotated_pub.publish(ros_annotated)
        except Exception as e:
            self.get_logger().error(f"Could not publish annotated image: {e}")

        # Publish detected classes
        detected_classes = [self.model.names[int(cls)] for cls in results[0].boxes.cls] if results[0].boxes is not None else []
        if detected_classes:
            msg_out = String()
            msg_out.data = ','.join(detected_classes)
            self.class_pub.publish(msg_out)
            self.get_logger().info(f"Detected: {msg_out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
