#include "perception_msgs/srv/detail/force_segment__struct.hpp"
#include "perception_msgs/srv/detail/get_centers__struct.hpp"
#include "sensor_msgs/msg/detail/image__struct.hpp"
#include <functional>

#include <mutex>
#include <future>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/msg/zed.hpp>
#include <perception_msgs/srv/get_centers.hpp>
#include <perception_msgs/srv/cluster.hpp>
#include <perception_msgs/srv/force_segment.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;    

using namespace sensor_msgs::msg;
using namespace perception_msgs::msg;
using namespace perception_msgs::srv;
    
class PerceptionNode
    : public rclcpp::Node {
private:
    using CenterClient = rclcpp::Client<GetCenters>;
    using SegmentClient = rclcpp::Client<ForceSegment>;
    using ClusterClient = rclcpp::Client<Cluster>;
    using ConeArrayPublisher = rclcpp::Publisher<ConeArray>;
    
    rclcpp::Subscription<Zed>::SharedPtr zed_subscription;
    
    CenterClient::SharedPtr center_estimation_client;
    SegmentClient::SharedPtr segment_client;
    ClusterClient::SharedPtr cluster_client;
    ConeArrayPublisher::SharedPtr cone_publisher;

    Image last_depth_map;
    Image last_segmentation_mask;

    std::mutex processing;
public:
    PerceptionNode()
	: rclcpp::Node("perception_node") {
	using namespace std::placeholders;
	this->declare_parameter<int>("service_wait_time_ms", 1000);
	this->cluster_client = this->create_client<Cluster>("cluster");
	this->segment_client = this->create_client<ForceSegment>("force_segment");
	this->center_estimation_client = this->create_client<GetCenters>("estimate_centers");
	this->cone_publisher = this->create_publisher<ConeArray>("cones", 1);
	this->zed_subscription = this->create_subscription<Zed>("zed", 1, std::bind(&PerceptionNode::segment, this, _1));
    }

    template <typename T>
    void make_sure_service_is_ready(std::shared_ptr<rclcpp::Client<T>> service) {
	if (not service->wait_for_service(0s)) {
	    do {
		RCLCPP_INFO_STREAM(this->get_logger(), "Service \"" << service->get_service_name() << "\" is not available. Waiting...");
	    } while (not service->wait_for_service(std::chrono::milliseconds(this->get_parameter("service_wait_time_ms").get_parameter_value().get<int>())));
	}
    }

    void segment(Zed zed_msg) {
	if (this->processing.try_lock()) {
	    auto segment_request = std::make_shared<ForceSegment::Request>();
	    this->last_depth_map = zed_msg.depth;
	    segment_request->input = zed_msg.color;
	    this->make_sure_service_is_ready<ForceSegment>(this->segment_client);
	    this->segment_client->async_send_request(segment_request, std::bind(&PerceptionNode::cluster, this, _1));
	}
    }

    void cluster(std::shared_future<ForceSegment::Response::SharedPtr> future) {
	auto cluster_request = std::make_shared<Cluster::Request>();
	this->last_segmentation_mask = future.get()->segmentation_mask;
	cluster_request->segmentation_mask = this->last_segmentation_mask;
	cluster_request->points = this->last_depth_map;
	this->make_sure_service_is_ready<Cluster>(this->cluster_client);
	this->cluster_client->async_send_request(cluster_request, std::bind(&PerceptionNode::estimate_centers, this, _1));
    }

    void estimate_centers(std::shared_future<Cluster::Response::SharedPtr> future) {
	auto center_request = std::make_shared<GetCenters::Request>();
	center_request->clusters = future.get()->clusters;
	center_request->segmentation_mask = this->last_segmentation_mask;
	center_request->depth = this->last_depth_map;
	this->make_sure_service_is_ready<GetCenters>(this->center_estimation_client);
	this->center_estimation_client->async_send_request(center_request, std::bind(&PerceptionNode::publish, this, _1));
    }

    void publish(std::shared_future<GetCenters::Response::SharedPtr> future) {
	this->cone_publisher->publish(future.get()->cones);
	this->processing.unlock();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
}
