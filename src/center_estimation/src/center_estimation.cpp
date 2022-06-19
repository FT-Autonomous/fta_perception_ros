
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
	auto points = (cv::Point3f*) request->depth.data.data();
	
	auto clusters = request->clusters.data;
	auto segmentation_mask = request->segmentation_mask.data;
	auto number_of_clusters = *std::max_element(clusters.begin(), clusters.end());
		
	for (auto i = 1; i < number_of_clusters + 1; i++) {
	    
	    int points_collected = 0;
	    auto color_index = 0;
	    Point estimated_center;

	    for (auto j = 0; j < clusters.size() && points_collected < 50; j++) {
		if (clusters[j] == i) {
		    points_collected++;
            if (points[j].x == points[j].x &&
                    points[j].y == points[j].y &&
                    points[j].z == points[j].z) {
		    estimated_center.x += points[j].x;
		    estimated_center.y += points[j].y;
		    estimated_center.z += points[j].z;
            }
		    if (segmentation_mask[j] != 0) {
                color_index = j;
            }
		}
        }

	   

	    estimated_center.x /= (float) points_collected;
	    estimated_center.y /= (float) points_collected;
	    estimated_center.z /= (float) points_collected;

	    auto color = segmentation_mask[color_index];
        RCLCPP_INFO_STREAM(this->get_logger(), "Found cone");
	    
	    if (color == 1) response->cones.blue_cones.push_back(estimated_center);
	    else if (color == 2) response->cones.yellow_cones.push_back(estimated_center);
	    else if (color == 3) response->cones.orange_cones.push_back(estimated_center);
	    else if (color == 4) response->cones.big_orange_cones.push_back(estimated_center);
	}
    }
};

int main (int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CenterEstimationNode>());
    rclcpp::shutdown();
}
