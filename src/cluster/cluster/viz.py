import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt

import rclpy
from math import isnan, isinf
from rclpy.node import Node
from perception_msgs.msg import ConeArray

def accumulate_with_color(cones, color, x, y, c):
    for cone in cones:
        hmmm = cone.x + cone.y + cone.z
        if not (isnan(hmmm) or isinf(hmmm)):
            x.append(cone.x)
            y.append(cone.z)
            c.append(color)

class VisualizeCones(Node):
    def __init__(self):
        super().__init__("visualize_cones")
        self.subscriber = self.create_subscription(ConeArray, "cones", self.callback, 1)
        plt.ion()
        plt.show()
        print("we done out here brah")

    def callback(self, cone_array):
        print("here brah")
        plt.clf()
        scatter_args = dict(x=[], y=[], c=[])
        accumulate_with_color(cone_array.blue_cones, [0,0,1], **scatter_args)
        accumulate_with_color(cone_array.yellow_cones, [0.4,0.4,0.1], **scatter_args)
        accumulate_with_color(cone_array.orange_cones, [0.5, 0.1, 0], **scatter_args)
        accumulate_with_color(cone_array.big_orange_cones, [0.4,0.05, 0.0], **scatter_args)
        plt.scatter(**scatter_args, s=30)
        plt.draw()
        plt.pause(0.001)
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisualizeCones())
    rclpy.shutdown()

if "__main__" == __name__:
    main()
