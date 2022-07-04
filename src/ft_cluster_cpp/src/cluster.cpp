#include "rclcpp/logging.hpp"
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <functional>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/srv/cluster.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "cluster_gl.hpp"

using namespace std;

using uchar = unsigned char;
using Point = std::array<float, 3>;

std::vector<Point> bytes_to_points(const std::vector<uchar> & bytes) {
    std::vector<Point> points;
    points.resize(bytes.size() / sizeof(Point));
    memcpy(points.data(), bytes.data(), bytes.size());
    return points;
}

std::vector<unsigned char> points_to_bytes(const std::vector<Point> & points) {
    std::vector<uchar> bytes;
    bytes.resize(points.size() * sizeof(Point));
    memcpy(bytes.data(), points.data(), bytes.size());
    return bytes;
}

class ClusterNode
    : public rclcpp::Node {
private:
    using Cluster = perception_msgs::srv::Cluster;
    GLCluster gl_cluster;
    rclcpp::Service<Cluster>::SharedPtr service;
    int unsafe = 0;

    bool traverse(int start,
		  uchar cluster,
		  std::vector<uchar> & cluster_map,
		  const std::vector<uchar> & class_map,
		  std::vector<std::vector<int>> & neighbors_map,
		  int min_samples)
    {
	if (not cluster_map[start]) {
	    cluster_map[start] = cluster;
        if (neighbors_map.size() > min_samples) {
	    for (auto neighbor : neighbors_map[start]) {
		traverse(neighbor,
			 cluster,
			 cluster_map,
			 class_map,
			 neighbors_map,
			 min_samples);
	    }

	}
        
        return true;
    } else {
	    return false;
	}
    }

    int traverse2(int index, int cluster_id, std::vector<std::vector<int>> & neighbors, std::vector<uchar> & cluster_map, int min_samples) {
        if (cluster_map[index] == 0) {
            if (neighbors[index].size() < min_samples) {
                cluster_map[index] = 255;
                return 0;
            } else {
                cluster_map[index] = cluster_id;
                int valid_neighbors = 0;
                for (auto neighbor : neighbors[index])
                    valid_neighbors += traverse2(neighbor, cluster_id, neighbors, cluster_map, min_samples);
                return 1 + valid_neighbors;
            }
        }
    }

    std::vector<uchar> cluster(const std::vector<Point> & points,
			       const std::vector<uchar> & class_map,
			       float epsilon,
			       int min_samples)
    {
	std::vector<std::vector<int>> neighbors_map(points.size());
        
        this->gl_cluster.setEps(epsilon);
        
        for (int c = 1; c <= 3; c++) {
            // When we collect high quality points of a particular class, we need to remember
            // their origins in the full array.
            std::vector<int> map_back;
            std::vector<Point> class_points;
            
            for (int i = 0; i < points.size(); i++) {
                float sum = points[i][0] + points[i][1] + points[i][2];
                if (sum and not std::isnan(sum) and not std::isinf(sum)
                    and (class_map[i] == c or c == 3 && class_map[i] == 4)) {
                    class_points.push_back(points[i]);
                    map_back.push_back(i);
                }
            }
            std::vector<std::vector<int>> class_neighbors_map(class_points.size());
            
            this->gl_cluster.setMaximumPoints(class_points.size());
            this->gl_cluster.setInputData((float*)class_points.data(), class_points.size(), false);
            
            for (int i = 0; i < class_points.size(); i += this->gl_cluster.getBatchSize()) {
                this->gl_cluster.generateOutput(i, class_neighbors_map);
            }
            // Remember to Zero initialize your buffers kids

            for (int i = 0; i < class_points.size(); i++) {
                int actual_root_index = map_back[i];
                assert(actual_root_index < points.size());
                for (int j = 0; j < class_neighbors_map[i].size(); j++) {
                    assert(map_back.size() == class_points.size());
                    int susIndex = class_neighbors_map[i][j];
                    int actual_child_index = map_back[susIndex];
                    assert(actual_child_index < points.size());
                    neighbors_map[actual_root_index].push_back(actual_child_index);
                }
            }
        }
    
	std::vector<uchar> cluster_map(points.size());

        // We use 1 as a background index and subtract 1 after the fact.
        int cluster_id = 2;
        int cumulative = 0;
        for (int i = 0; i < points.size(); i++) {
            int visited = traverse2(i, cluster_id, neighbors_map, cluster_map, min_samples);
            if (visited - 1 > 0) {
                cout << "Cluster #" << i << " is related to " << visited << " points" << endl;
                cumulative += visited;
                cluster_id++;
            }
        }
        cout << "Cumulatively, " << cumulative << " points were visited " << endl;
        
        
        for (int i = 0; i < points.size(); i++) {
            if (cluster_map[i] == 1 || cluster_map[i] == 255) {
                cluster_map[i] = 0;
            }
        }

        return cluster_map;
    }

    void cluster_callback(Cluster::Request::SharedPtr request, Cluster::Response::SharedPtr response) {
	auto & image = response->clusters;
	image.header.frame_id = request->points.header.frame_id;
	image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	image.height = request->points.height;
	image.width = request->points.width;
	image.data = cluster(bytes_to_points(request->points.data),
			     request->segmentation_mask.data,
			     this->get_parameter("eps").get_parameter_value().get<float>(),
			     this->get_parameter("min_samples").get_parameter_value().get<int>());
    }
    
public:
    ClusterNode()
	: rclcpp::Node("cluster_node") {
	using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "HERE");
	this->declare_parameter<int>("min_samples", 10);
	this->declare_parameter<float>("eps", 300);
        this->declare_parameter<int>("batch_size", 1000);
        this->gl_cluster.setBatchSize(this->get_parameter("batch_size").get_parameter_value().get<int>());
        this->gl_cluster.setEps(this->get_parameter("eps").get_parameter_value().get<float>());
	this->service = this->create_service<Cluster>("cluster",
						      std::bind(&ClusterNode::cluster_callback, this, _1, _2));
	RCLCPP_INFO(this->get_logger(), "Initialised c++ cluster node");
    }
};

int
main(int argc, char *argv[])
{
    initializeEGL();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterNode>());
    rclcpp::shutdown();
}
