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
from perception_msgs.srv import ForceSegment

class Segmentation(Node):
    def __init__(self):
        super().__init__("segmentation")
        self.declare_parameter("model", "cgnet")
        self.declare_parameter("weights", os.path.join(os.environ['HOME'], "downloads", self.get_parameter('model').get_parameter_value().string_value + ".ts"))
        sys.path.append(os.path.join(get_package_prefix('fta'), 'lib', 'fta'))
        import fta
        self.subscription = self.create_subscription(Image, "color", self.callback, 1)
        self.force_segment = self.create_service(ForceSegment, "force_segment", self.force_segment);
        self.fta = fta
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.jit.load(self.get_parameter('weights').get_parameter_value().string_value, map_location=self.device)
        self.get_logger().info('Loaded model')
        self.publisher = self.create_publisher(Image, "segmentation_mask", 1)
        
    def segment(self, image):
        cv_image = np.array(image.data).reshape(image.height, image.width, 3)
        cv_image = self.fta.live.half(self.fta.live.scale(cv_image, 512))
        output = np.uint8(self.fta.live.predict(cv_image, model=self.model, device=self.device))
        return Image(
            header=Header(frame_id=image.header.frame_id),
            height=output.shape[0],
            width=output.shape[1],
            data=array.array('B', output.flatten().tolist())
        )                    
    
    def force_segment(self, request, response):
        response.segmentation_mask = self.segment(request.input)
        return response
        
    def callback(self, image):
        self.publisher.publish(self.segment(image))

def main(args=None):
    rclpy.init(args=args)
    seg = Segmentation()
    rclpy.spin(seg)
    seg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
