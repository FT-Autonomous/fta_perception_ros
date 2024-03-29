#include <functional>
#include <filesystem>

#include <mutex>
#include <future>
#include <memory>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ft_msgs/msg/zed.hpp>
#include <ft_msgs/srv/get_centers.hpp>
#include <ft_msgs/srv/cluster.hpp>
#include <ft_msgs/srv/force_segment.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;    

using namespace sensor_msgs::msg;
using namespace ft_msgs::msg;
using namespace ft_msgs::srv;
    
class PerceptionNode
    : public rclcpp::Node {
private:
    using CenterClient = rclcpp::Client<GetCenters>;
    using SegmentClient = rclcpp::Client<ForceSegment>;
    using ClusterClient = rclcpp::Client<Cluster>;
    
    rclcpp::Subscription<Zed>::SharedPtr zed_subscription;
    
    CenterClient::SharedPtr center_estimation_client;
    SegmentClient::SharedPtr segment_client;
    ClusterClient::SharedPtr cluster_client;

    Image last_depth_map;
    Image last_segmentation_mask;

    int ad_hoc_interval;
    std::mutex lock;
    int ad_hoc_timer;
public:
    PerceptionNode()
	: rclcpp::Node("perception_node") {
	using namespace std::placeholders;
	this->declare_parameter<int>("service_wait_time_ms", 1000);
	this->declare_parameter<int>("downsample_factor", 1);
	this->cluster_client = this->create_client<Cluster>("cluster");
        RCLCPP_INFO(this->get_logger(), "cluster available");
	this->segment_client = this->create_client<ForceSegment>("force_segment");
        RCLCPP_INFO(this->get_logger(), "force segment available");
	this->zed_subscription = this->create_subscription<Zed>("zed", 1, std::bind(&PerceptionNode::segment, this, _1));
        RCLCPP_INFO(this->get_logger(), "zed subscription made");
    }

    template <typename T>
    void make_sure_service_is_ready(std::shared_ptr<rclcpp::Client<T>> service) {
	if (not service->wait_for_service(0s)) {
	    do {
		RCLCPP_INFO_STREAM(this->get_logger(), "Service \"" << service->get_service_name() << "\" is not available. Waiting...");
	    } while (not service->wait_for_service(std::chrono::milliseconds(this->get_parameter("service_wait_time_ms").get_parameter_value().get<int>())));
	}
    }

    void segment(Zed::SharedPtr zed_msg) {
        if (lock.try_lock()) {
            auto segment_request = std::make_shared<ForceSegment::Request>();
            this->last_depth_map = zed_msg->depth;
            segment_request->input = zed_msg->color;
            this->make_sure_service_is_ready<ForceSegment>(this->segment_client);
            this->segment_client->async_send_request(segment_request, std::bind(&PerceptionNode::cluster, this, _1));
        }
    }

    void cluster(std::shared_future<ForceSegment::Response::SharedPtr> future) {

	int factor = this->get_parameter("downsample_factor").get_parameter_value().get<int>();

        Image & depth = last_depth_map;
        Image new_depth;
        new_depth.width = depth.width / factor;
        new_depth.height = depth.height / factor;
	
        cv::Mat floats = cv::Mat(depth.height, depth.width, CV_32FC3);
	memcpy(floats.data, depth.data.data(), depth.data.size());
	
        cv::Mat out_depth;
        cv::resize(floats, out_depth, cv::Size(new_depth.width, new_depth.height), cv::INTER_NEAREST);
	
        new_depth.data.resize(4 * new_depth.width * new_depth.height * 3);
        memcpy(new_depth.data.data(), out_depth.data, new_depth.data.size());

        Image & seg = future.get()->segmentation_mask;
        Image new_segmentation_mask;
        new_segmentation_mask.width = seg.width / factor;
        new_segmentation_mask.height = seg.height / factor;
        cv::Mat chars = cv::Mat(seg.height, seg.width, CV_8UC1);
	memcpy(chars.data, seg.data.data(), seg.data.size());
	cv::Mat downsampled_segmentation_mask;
        cv::resize(chars, downsampled_segmentation_mask, cv::Size(new_segmentation_mask.width, new_segmentation_mask.height), cv::INTER_NEAREST);
        new_segmentation_mask.data.resize(new_segmentation_mask.width * new_segmentation_mask.height);
        memcpy(new_segmentation_mask.data.data(), downsampled_segmentation_mask.data, new_segmentation_mask.data.size());

	auto cluster_request = std::make_shared<Cluster::Request>();
	this->last_segmentation_mask = new_segmentation_mask;
	cluster_request->segmentation_mask = new_segmentation_mask;
	this->last_depth_map = new_depth;
	cluster_request->points = new_depth;
	this->make_sure_service_is_ready<Cluster>(this->cluster_client);
	this->cluster_client->async_send_request(cluster_request, std::bind(&PerceptionNode::estimate_centers, this, _1));
    }

    void estimate_centers(std::shared_future<Cluster::Response::SharedPtr>) {
        lock.unlock(); // Hacky way of doing things
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
}
