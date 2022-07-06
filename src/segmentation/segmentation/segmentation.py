import sys, os
import array
import torch
import torchvision.transforms.functional as F
import torchvision.transforms as T
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from ament_index_python.packages import get_package_prefix
from eufs_msgs.srv import ForceSegment
from eufs_msgs.msg import Zed

class Segmentation(Node):
    def __init__(self):
        super().__init__("segmentation")
        self.declare_parameter("publish_force_segment_results", True)
        self.declare_parameter("subscribe_to_zed", False);
        self.declare_parameter("model", "cgnet")
        self.declare_parameter("weights", os.path.join(get_package_prefix('segmentation'), 'share/segmentation/', 'cgnet.ts'))
        sys.path.append(os.path.join(get_package_prefix('ft_semantic_segmentation'), 'lib'))
        import ft_semantic_segmentation
        self.ft_semantic_segmentation = ft_semantic_segmentation
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.jit.load(self.get_parameter('weights').get_parameter_value().string_value, map_location=self.device)
        self.get_logger().info('Loaded model')
        self.publisher = self.create_publisher(Image, "segmentation_mask", 1)
        if self.get_parameter("subscribe_to_zed").get_parameter_value().bool_value:
            self.subscription = self.create_subscription(Zed, "zed", self.callback, 1)
        self.force_segment = self.create_service(ForceSegment, "force_segment", self.force_segment);
        self.get_logger().info('Initialised Segmentation Node')
        
    def segment(self, image):
        self.get_logger().info("Segmenting...")
        cv_image = np.array(image.data).reshape(image.height, image.width, 3)
        output = np.uint8(self.ft_semantic_segmentation.live.predict(cv_image, model=self.model, device=self.device))
        return Image(
            header=Header(frame_id=image.header.frame_id),
            height=output.shape[0],
            width=output.shape[1],
            data=array.array('B', output.flatten().tolist())
        )                    
    
    def force_segment(self, request, response):
        response.segmentation_mask = self.segment(request.input)
        if self.get_parameter("publish_force_segment_results").get_parameter_value().bool_value:
            self.publisher.publish(response.segmentation_mask)
        return response
        
    def callback(self, zed):
        self.publisher.publish(self.segment(zed.color))

def main(args=None):
    rclpy.init(args=args)
    seg = Segmentation()
    rclpy.spin(seg)
    seg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
