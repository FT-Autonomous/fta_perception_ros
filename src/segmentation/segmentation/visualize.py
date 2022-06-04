import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2 as cv
import os
import sys
from ament_index_python.packages import get_package_prefix

class VisualizeSegmentation(Node):
    def __init__(self):
        super().__init__("visualize_segmentation")
        self.subscription = self.create_subscription(Image, "segmentation_mask", self.callback, 1)
        sys.path.append(os.path.join(get_package_prefix('fta'), 'lib', 'fta'))
        import fta
        self.fta = fta

    def callback(self, image):
        prediction = np.asarray(image.data, dtype=np.uint8).reshape(image.height, image.width)
        cv.imshow("Segmentation Preview", self.fta.live.colorise(prediction))
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    visualize_segmentation = VisualizeSegmentation()
    rclpy.spin(visualize_segmentation)
    visualize_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
