#include <cstring>
#include <vector>
#include <array>
#include <map>
#include <memory>
#include <functional>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/srv/cluster.hpp>
#include <sensor_msgs/image_encodings.hpp>

using uchar = unsigned char;
using Point = std::array<float, 3>;

float sqeuclidean(const Point & a,
	    const Point & b)
{
    auto d0 = (a[0] - b[0]),
	 d1 = (a[1] - b[1]),
	 d2 = (a[2] - b[2]);
    return d0 * d0 + d1 * d1 + d2 * d2;
}

void traverse(int start,
	 uchar cluster,
	 std::vector<uchar> & cluster_map,
	 const std::vector<uchar> & class_map,
	 const std::map<int, std::vector<int>> & neighbors_map)
{
    if (not cluster_map[start]) {
	cluster_map[start] = cluster;
	for (auto pair : neighbors_map) {
	    if (class_map[start] == class_map[pair.first]) {
		traverse(pair.first,
			 cluster,
			 cluster_map,
			 class_map,
			 neighbors_map);
	    }
	}
    }
}

std::vector<uchar> cluster(const std::vector<Point> & points,
	const std::vector<uchar> & class_map,
	float epsilon,
	int min_samples)
{
    const float sqepsilon = epsilon * epsilon;
    std::map<int, std::vector<int>> neighbors_map;
    
    for (auto i = 0; i < points.size(); i++) {
	if (not class_map[i]) {
	    for (auto j = 0; j < points.size(); j++) {
		if (sqeuclidean(points[i], points[j]) < sqepsilon) {
		    neighbors_map[i].push_back(j);
		}
	    }

	    if (neighbors_map[i].size() < min_samples) {
		neighbors_map.erase(i);
	    }
	}
    }

    std::vector<uchar> cluster_map;
    cluster_map.resize(points.size(), 0);

    int cluster = 1;
    for (auto pair : neighbors_map) {
	traverse(pair.first,
		 cluster,
		 cluster_map,
		 class_map,
		 neighbors_map);
    }

    return cluster_map;
}

std::vector<Point> bytes_to_points(const std::vector<uchar> & bytes) {
    std::vector<Point> points;
    points.resize(bytes.size() / sizeof(Point));
    memcpy(points.data(), bytes.data(), bytes.size());
    return points;
}

std::vector<unsigned char> points_to_bytes(const std::vector<Point> & points) {
    std::vector<uchar> bytes;
    bytes.resize(points.size() * sizeof(Point));
    memcpy(bytes.data(), points.data(), points.size() * sizeof(Point));
    return bytes;
}

class ClusterNode
    : public rclcpp::Node {
private:
    using Cluster = perception_msgs::srv::Cluster;
    rclcpp::Service<Cluster>::SharedPtr service;
public:
    ClusterNode()
	: rclcpp::Node("cluster_node") {
	using namespace std::placeholders;
	this->declare_parameter<float>("min_samples", 100);
	this->declare_parameter<float>("eps", 100);
	this->service = this->create_service<Cluster>("cluster",
						      std::bind(&ClusterNode::cluster_callback, this, _1, _2));
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
};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterNode>());
    rclcpp::shutdown();
}
