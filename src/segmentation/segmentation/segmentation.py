import numpy as np
import rclpy
import cv2 as cv
from rclpy.node import Node
import sensor_msgs
from perception_msgs.msg import Zed as ZedTopic

class Segmentation(Node):
    def __init__(self):
        super().__init__("segmentation")
        self.subscription = self.create_subscription(ZedTopic, "feed", self.callback, 10)

    def callback(self, zed_stuff):
        image = zed_stuff.color
        data = np.array(image.data).reshape(image.height, image.width, 3);
        cv.imshow("ROS", data)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    seg = Segmentation()
    rclpy.spin(seg)
    seg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
