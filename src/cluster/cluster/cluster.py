import rclpy
from rclpy.node import Node
import cv2 as cv
from sklearn.cluster import OPTICS
import numpy as np
from math import nan
from perception_msgs.srv import Cluster
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import array
import time

class ClusterNode(Node):
    def __init__(self):
        super().__init__("cluster")
        self.declare_parameter("eps", 300)
        self.declare_parameter("min_samples", 100)
        self.declare_parameter("n_jobs", 6)
        self.cluster_service = self.create_service(Cluster, "cluster", self.cluster)
        
    def cluster(self, request, response):
        print("here")
        start = time.perf_counter()
        segmentation_mask = np.frombuffer(request.segmentation_mask.data, dtype=np.uint8).reshape(request.segmentation_mask.height, request.segmentation_mask.width)
        depth_map = np.frombuffer(request.points.data, dtype=np.float32).reshape(request.points.height, request.points.width, 3)
        depth_map[np.isnan(depth_map)] = 100
        depth_map[np.isinf(depth_map)] = 100

        cv.imshow("Depth", depth_map / depth_map.max())
        cv.waitKey(1);

        cluster_map = np.zeros((segmentation_mask.shape[0], segmentation_mask.shape[1]), dtype=np.uint8)
        non_background_indices = np.nonzero(segmentation_mask) # Maps pred -> real
        min_samples = self.get_parameter("min_samples").get_parameter_value().integer_value
        
        if len(non_background_indices[0]) > min_samples:
            point_cloud = depth_map[non_background_indices]
            cluster_map[non_background_indices] = 1

            # cluster the filtered point cloud
            model = OPTICS(
                cluster_method='dbscan',
                eps=self.get_parameter("eps").get_parameter_value().integer_value,
                min_samples=min_samples,
                n_jobs=self.get_parameter("n_jobs").get_parameter_value().integer_value
            )
            
            # eps is how close the samples should be to each other to be considered as part of a same cluster. Set to max 30cm but need to optimize!
            # min_samples means that a cluster is made of at least 7 data points
            pred = model.fit_predict(point_cloud)
            self.get_logger().info(f"Found {max(pred)} clusters in image '{request.segmentation_mask.header.frame_id}'")

            #loop through to assign each non background pixel to its cluster
            cluster_map[non_background_indices] = pred + 1

        response.clusters = Image(
            header=Header(
                frame_id=request.segmentation_mask.header.frame_id
            ),
            width=segmentation_mask.shape[1],
            height=segmentation_mask.shape[0],
            data=cluster_map.tobytes()
        )
        
        self.get_logger().info(f"Cluster took {time.perf_counter() - start}s")

        return response


def main(args=None):
    rclpy.init(args=args)
    cluster_node = ClusterNode()
    rclpy.spin(cluster_node)
    cluster_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
