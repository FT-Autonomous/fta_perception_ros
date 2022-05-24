#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include <future>
#include <ostream>
#include <vector>
#include <memory>
#include <functional>
#include <cstdlib>
#include <string>
#include <iostream>
#include <filesystem>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <perception_msgs/srv/force_segment.hpp>

using namespace std::chrono_literals;

class VideoCaptureNode
    : public rclcpp::Node {
private:
    using Image = sensor_msgs::msg::Image;
    using ForceSegment = perception_msgs::srv::ForceSegment;
    using ForceSegmentClient = rclcpp::Client<ForceSegment>;
    using ImagePublisher = rclcpp::Publisher<Image>;

    unsigned int frame_id;
    cv::VideoCapture camera;
    Image current_ros_image;
    cv::Mat image_buffer;
    ImagePublisher::SharedPtr color_publisher;
    ImagePublisher::SharedPtr depth_publisher;
    ForceSegmentClient::SharedPtr force_segment_client;
    rclcpp::TimerBase::SharedPtr shot_clock;
    rclcpp::TimerBase::SharedPtr force_segment_future_timer;
    std::shared_future<ForceSegmentClient::SharedResponse> force_segment_future;
    bool waiting;

private:
    void capture_image() {
	
	camera.read(this->image_buffer);

	current_ros_image.header.frame_id = std::to_string(this->frame_id++);
	current_ros_image.width = this->image_buffer.cols;
	current_ros_image.height = this->image_buffer.rows;
	current_ros_image.encoding = sensor_msgs::image_encodings::BGR8;
	current_ros_image.step = this->image_buffer.cols * 3;
	current_ros_image.data = this->image_buffer.reshape(1, image_buffer.rows * image_buffer.cols * image_buffer.channels());
	
	auto request = std::make_shared<ForceSegment::Request>();
	request->input = current_ros_image;
	waiting = true;
	this->force_segment_future = this->force_segment_client->async_send_request(request);
	this->force_segment_future_timer = this->create_wall_timer(5ms, std::bind(&VideoCaptureNode::try_finish, this));

    }

    void try_finish() {
	if (this->force_segment_future.wait_for(0s) == std::future_status::ready) {
	    this->force_segment_future_timer->cancel();
	    this->current_ros_image.data = this->image_buffer.reshape(1, image_buffer.rows * image_buffer.cols * image_buffer.channels());
	    this->color_publisher->publish(this->current_ros_image);
	    this->depth_publisher->publish(this->force_segment_future.get()->segmentation_mask);
	    waiting = false;
	}
    }
public:
    VideoCaptureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: rclcpp::Node("video_capture", options),
	  frame_id(0),
	  waiting(false)
    {
	using namespace std::placeholders;
	this->declare_parameter<int>("camera", 1);
	this->camera = cv::VideoCapture(this->get_parameter("camera").get_parameter_value().get<int>());
	this->color_publisher = this->create_publisher<Image>("color", 1);
	this->depth_publisher = this->create_publisher<Image>("depth", 1);
	this->shot_clock = this->create_wall_timer(0.5s, std::bind(&VideoCaptureNode::capture_image, this));
	this->force_segment_client = this->create_client<ForceSegment>("force_segment");
	while (not this->force_segment_client->wait_for_service(1s)) {
	    RCLCPP_INFO(this->get_logger(), "Force Segment Server not available! Waiting...");
	}
    }
};

int main (int ac, char *av[]) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<VideoCaptureNode>());
    rclcpp::shutdown();
}
