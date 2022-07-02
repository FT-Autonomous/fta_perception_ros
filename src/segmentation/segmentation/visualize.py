import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from perception_msgs.msg import Zed
import numpy as np
import cv2 as cv
import os
import sys
from ament_index_python.packages import get_package_prefix

class VisualizeSegmentation(Node):
    def __init__(self):
        super().__init__("visualize_segmentation")
        self.mask_subscription = self.create_subscription(Image, "segmentation_mask", self.mask_callback, 1)
        self.image_subscription = self.create_subscription(Zed, "zed", self.image_callback, 1)
        self.show_timer = self.create_timer(0.1, self.show_callback)
        sys.path.append(os.path.join(get_package_prefix('ft_semantic_segmentation'), 'lib'))
        import ft_semantic_segmentation
        self.ft_semantic_segmentation = ft_semantic_segmentation
        self.image = None
        self.mask = None

    def show_callback(self):
        if self.image is not None and self.mask is not None:
            image_to_show = self.ft_semantic_segmentation.live.merge(self.image, self.mask)
            cv.imshow("Visualisation", image_to_show)
            cv.waitKey(1)
        
    def image_callback(self, zed):
        self.image = np.asarray(zed.color.data, dtype=np.uint8).reshape(zed.color.height, zed.color.width, 3);

    def mask_callback(self, mask):
        prediction = np.asarray(mask.data, dtype=np.uint8).reshape(mask.height, mask.width)
        self.mask = self.ft_semantic_segmentation.live.colorise(prediction)

def main(args=None):
    rclpy.init(args=args)
    visualize_segmentation = VisualizeSegmentation()
    rclpy.spin(visualize_segmentation)
    visualize_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
