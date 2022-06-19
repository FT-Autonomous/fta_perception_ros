#include "rclcpp/logging.hpp"
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <limits>
#include <map>
#include <memory>
#include <functional>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/srv/cluster.hpp>
#include <sensor_msgs/image_encodings.hpp>

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

template <typename T> std::vector<T> pool(const std::vector<T> & input, int n = 4) {
    std::vector<T> output;

    for (int i = 0; i < input.size(); i+=n) {
	output.push_back(input[i]);
    }
    
    return output;
}

template <typename T> std::vector<T> unpool(const std::vector<T> & input, int n = 4) {
    std::vector<T> output;

    for (int i = 0; i < input.size(); i++) {
	for (int j = 0; j < n; j++) {
	    output.push_back(input[i]);
	}
    }
    
    return output;
}

class ClusterNode
    : public rclcpp::Node {
private:
    using Cluster = perception_msgs::srv::Cluster;
    rclcpp::Service<Cluster>::SharedPtr service;
    int unsafe = 0;

    float sqeuclidean(const Point & a,
		      const Point & b)
    {
	auto d0 = (a[0] - b[0]),
	    d1 = (a[1] - b[1]),
	    d2 = (a[2] - b[2]);
	auto d = d0 * d0 + d1 * d1 + d2 * d2;
	return d >= 0 && d <= 10000 ? d : 10000;
    }

    bool traverse(int start,
		  uchar cluster,
		  std::vector<uchar> & cluster_map,
		  const std::vector<uchar> & class_map,
		  std::vector<std::vector<int>> & neighbors_map,
		  int min_samples)
    {
        if (unsafe++ > 100000000) {

            return false;
        }
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

    std::vector<uchar> cluster(const std::vector<Point> & source_points,
			       const std::vector<uchar> & source_class_map,
			       float epsilon,
			       int min_samples)
    {
	
	auto points = pool(source_points);
	auto class_map = pool(source_class_map);
	
	const float sqepsilon = epsilon * epsilon;
	std::vector<std::vector<int>> neighbors_map;
	neighbors_map.resize(points.size());
    
	for (auto i = 0; i < points.size() - 1; i++) {
	    if (i % 50000 == 0) {
		RCLCPP_INFO_STREAM(this->get_logger(), "At " << i);
	    }
	    
	    if (class_map[i]) {
		for (auto j = i + 1; j < points.size(); j++) {
		    if (class_map[i] == class_map[j]
			&& sqeuclidean(points[i], points[j]) < sqepsilon) {
			neighbors_map[i].push_back(j);
		    }
		}
	    }
	}

    RCLCPP_INFO(this->get_logger(), "ALIVE");
    
	std::vector<uchar> cluster_map;
	cluster_map.resize(points.size(), 0);

	int cluster = 1;
    unsafe = 0;
	for (int i = 0; i < neighbors_map.size(); i++) {
	    if (traverse(i,
			 cluster,
			 cluster_map,
			 class_map,
			 neighbors_map,
			 min_samples)) {
		cluster++;
	    }
	}

    RCLCPP_INFO(this->get_logger(), "STILL ALIVE");

	return unpool(cluster_map);
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
			     this->get_parameter("min_samples").get_parameter_value().get<float>());
    }
    
public:
    ClusterNode()
	: rclcpp::Node("cluster_node") {
	using namespace std::placeholders;
	this->declare_parameter<float>("min_samples", 100);
	this->declare_parameter<float>("eps", 100);
	this->service = this->create_service<Cluster>("cluster",
						      std::bind(&ClusterNode::cluster_callback, this, _1, _2));
	RCLCPP_INFO(this->get_logger(), "Initialised c++ cluster node");
    }
};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterNode>());
    rclcpp::shutdown();
}
