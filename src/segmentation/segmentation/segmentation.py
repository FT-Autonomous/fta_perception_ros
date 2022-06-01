import torchvision.transforms.functional as F
import torchvision.transforms as T
import numpy as np
import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from perception_msgs.srv import ForceSegment
from std_msgs.msg import String
import sys, os
import torch

def mask_to_color(prediction):
    # This really could be an array
    colors = np.array([
        [0, 0, 0], # Background
        [0, 0, 255], # Blue
        [100, 100, 20], # Yellow
        [100, 50, 0], # Large Orange
        [120, 30, 0]  # Orange 
    ], dtype=np.uint8)
    
    width = prediction.shape[0]
    height = prediction.shape[1]
    return colors[prediction.cpu().numpy()]

class Segmentation(Node):
    def __init__(self):
        super().__init__("segmentation")
        self.declare_parameter("fta_path", os.path.join(os.environ['HOME'], "projects", "python", "ft_semantic_segmentation"))
        self.declare_parameter("model", "cgnet")
        self.declare_parameter("weights", os.path.join(os.environ['HOME'], "downloads", self.get_parameter('model').get_parameter_value().string_value + ".pth"))
        sys.path.append(self.get_parameter("fta_path").get_parameter_value().string_value)
        import fta
        self.subscription = self.create_subscription(Image, "color", self.segment, 10)
        self.force_segment = self.create_service(ForceSegment, "force_segment", self.force_segment);
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = fta.models.model_zoo.get_segmentation_model(
            dataset="overfit",
            pretrained_base=False,
            backbone="resnet50",
            model=self.get_parameter("model").get_parameter_value().string_value,
            norm_layer=torch.nn.BatchNorm2d
        ).to(self.device)
        self.model.load_state_dict(torch.load(
            self.get_parameter("weights").get_parameter_value().string_value,
            map_location=self.device
        ))
        self.model.train()
        
    def segment(self, image):
        data = np.array(image.data).reshape(image.height, image.width, 3)
        self.predict_and_show(data)
        cv.waitKey(1)
        return image
    
    def force_segment(self, request, response):
        response.segmentation_mask = self.segment(request.input)
        return response
        
    def callback(self, image):
        data = np.array(image.data).reshape(image.height, image.width, 3);
        cv.imshow("ROS", data)
        cv.waitKey(1)

    def predict_and_show(self, original_image, interval=25):
        '''Takes a numpy HWC, BGR image, makes predictions
        And shows them on the screen'''
        new_height = int(512 * original_image.shape[1] / original_image.shape[0])
        resized_image = cv.resize(original_image, (new_height - new_height % 64, 512), interpolation=cv.INTER_NEAREST)
        image = cv.cvtColor(resized_image, cv.COLOR_BGR2RGB)
        normal = T.Normalize([.485, .456, .406], [.229, .224, .225])
        tensor = normal(torch.from_numpy(image.transpose(2, 0, 1)).to(self.device).float())
        #TODO: Publish both argmax and softmax versions
        pred = torch.nn.Softmax(0)(self.model(tensor[None, ...])[0][0]).argmax(0)
        #pred[0, pred[0, ...] > 0.2] = 1
        mask = cv.cvtColor(mask_to_color(pred), cv.COLOR_RGB2BGR)
        #cv.imshow("Image only", resized_image)
        result = np.uint8(resized_image / 2 + mask / 2.0)
        cv.imshow("Mask and Image", result)
        #cv.imshow("Mask only", mask)
        #cv.imwrite(f"test/{random.randint(0, 1000000)}.png", result)
        return pred

def main(args=None):
    rclpy.init(args=args)
    seg = Segmentation()
    rclpy.spin(seg)
    seg.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
