import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')

        self.publisher_ = self.create_publisher(Image, 'webcam_output', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.webcam = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
   
    def timer_callback(self):
        ret, frame = self.webcam.read()
          
        if ret == True:
            # Convert the frame to a ROS Image message with the correct encoding
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
 
            self.get_logger().info('Publishing video frame')
  
def main():
    try:
        rclpy.init(args=None)
        webcam_publisher = WebcamPublisher()
        rclpy.spin(webcam_publisher)
  
    except KeyboardInterrupt:
        webcam_publisher.destroy_node()
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()
