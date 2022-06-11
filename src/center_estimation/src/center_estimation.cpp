
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "opencv2/core/types.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/service.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/srv/get_centers.hpp>
#include <geometry_msgs/msg/point.h>
#include <vector>
#include <opencv2/core.hpp>

class CenterEstimationNode : public rclcpp::Node {
private:
    using GetCenters = perception_msgs::srv::GetCenters;
    using Point = geometry_msgs::msg::Point;
    rclcpp::Service<GetCenters>::SharedPtr get_centers_service;
public:
    CenterEstimationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: rclcpp::Node("center_estimation") {
	using namespace std::placeholders;
	this->get_centers_service = this->create_service<GetCenters>("estimate_centers", std::bind(&CenterEstimationNode::get_centers, this, _1, _2));
    }

    void get_centers(GetCenters::Request::SharedPtr request, GetCenters::Response::SharedPtr response) {

	// Problems:
	// Floating Point Percision?
	// Efficiency?
	
	std::vector<cv::Point3f> points;
	auto number_of_points = request->depth.height * request->depth.width;
	points.reserve(number_of_points);
	std::memcpy(points.data(), request->depth.data.data(), sizeof(cv::Point3f) * number_of_points);
	RCLCPP_INFO_STREAM(this->get_logger(), "Size of point is " << sizeof(Point));
	auto clusters = request->clusters.data;
	auto segmentation_mask = request->segmentation_mask.data;
	auto number_of_clusters = *std::max_element(clusters.begin(), clusters.end());
	
	for (auto i = 1; i < number_of_clusters + 1; i++) {
	    int points_collected = 0;
	    Point estimated_center;

	    for (auto j = 0; j < clusters.size() && points_collected < 50; j++) {
		if (clusters[j] == i) {
		    points_collected++;
		    estimated_center.x += points[j].x;
		    estimated_center.y += points[j].y;
		    estimated_center.z += points[j].z;
		}
	    }

	    estimated_center.x /= (float) points_collected;
	    estimated_center.y /= (float) points_collected;
	    estimated_center.z /= (float) points_collected;
	    
	    response->centers.push_back(estimated_center);
	}
    }
};

int main (int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CenterEstimationNode>());
    rclcpp::shutdown();
}
