import rclpy
from rclpy.node import Node
from std_msgs.msg import Image
from std_msgs.msg import ByteMultiArray
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CVExample(Node):
    def __init__(self):
        super().__init__('cv_node')

        self.img = np.ndarray((720,1280,3))
        self.valid_img = False
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image,'/video_source/raw',self.camera_callback,10)
        self.pub = self.create_publisher(Image,'/img_processing/compressed',10)
        self.publisher = self.create_publisher(CompressedImage, '/image', 10)

        self.msg = ByteMultiArray()

        self.msg2 = CompressedImage()

        dt = 0.01
        self.timer = self.create_timer(dt, self.timer_callback)
        self.logger().info("CVExample node has been created")

    def camera_callback(self):
        try:
            self.img = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")
            print("Image received")
            self.valid_img = True
        except:
            self.get_logger().info("Error converting image")

    def timer_callback(self):
        try:
            if self.valid_img:
                imgCompr = np.array(cv2.imencode('.jpg', self.img)[1]).tostring()
                #print("Memory Size Original in bytes: ",self.img.size*self)
                print("Memory Size Compressed in bytes: ",len(imgCompr))

                self.msg2.header.stamp = self.get_clock().now().to_msg()
                self.msg2.format = "jpeg"
                self.msg2.data = imgCompr
                self.publisher.publish(self.msg2)
                self.valid_img = False
            else:
                self.get_logger().info("No image available")    
        except:
            self.get_logger().info("Error publishing image")

def main(args=None):
    rclpy.init(args=args)
    cv_node = CVExample()
    rclpy.spin(cv_node)
    cv_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()

