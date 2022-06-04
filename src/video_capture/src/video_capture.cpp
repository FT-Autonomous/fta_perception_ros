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
    using ImagePublisher = rclcpp::Publisher<Image>;

    unsigned int frame_id;
    cv::VideoCapture camera;
    Image current_ros_image;
    cv::Mat image_buffer;
    ImagePublisher::SharedPtr color_publisher;
    ImagePublisher::SharedPtr depth_publisher;
    rclcpp::TimerBase::SharedPtr shot_clock;
    
private:
    void capture_image() {
	camera.read(this->image_buffer);

	Image image;
	image.header.frame_id = std::to_string(this->frame_id++);
	image.width = this->image_buffer.cols;
	image.height = this->image_buffer.rows;
	image.encoding = sensor_msgs::image_encodings::BGR8;
	image.step = this->image_buffer.cols * 3;
	image.data = this->image_buffer.reshape(1, image_buffer.rows * image_buffer.cols * image_buffer.channels());

	this->color_publisher->publish(image);
    }
    
public:
    VideoCaptureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: rclcpp::Node("video_capture", options),
	  frame_id(0)
    {
	using namespace std::placeholders;
	this->declare_parameter<int>("camera", 1);
	this->camera = cv::VideoCapture(this->get_parameter("camera").get_parameter_value().get<int>());
	this->color_publisher = this->create_publisher<Image>("color", 1);
	this->depth_publisher = this->create_publisher<Image>("depth", 1);
	this->shot_clock = this->create_wall_timer(0s, std::bind(&VideoCaptureNode::capture_image, this));
    }
};

int main (int ac, char *av[]) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<VideoCaptureNode>());
    rclcpp::shutdown();
}
