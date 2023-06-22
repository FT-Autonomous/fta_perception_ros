#include "rclcpp/time.hpp"
#include <ctime>
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

#include <ft_msgs/msg/zed.hpp>

using namespace std::chrono_literals;

class VideoCaptureNode
    : public rclcpp::Node {
private:
    using Image = sensor_msgs::msg::Image;
    using Zed = ft_msgs::msg::Zed;
    using ZedPublihser = rclcpp::Publisher<Zed>;

    unsigned int frame_id;
    cv::VideoCapture camera;
    cv::Mat image_buffer;
    ZedPublihser::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr shot_clock;
    
private:
    void capture_image() {
	camera.read(this->image_buffer);

	Image color, depth;
	color.header.frame_id = std::to_string(this->frame_id++);
	color.width = this->image_buffer.cols;
	color.height = this->image_buffer.rows;
	color.encoding = sensor_msgs::image_encodings::BGR8;
	color.step = this->image_buffer.cols * 3;
	color.data = this->image_buffer.reshape(1, image_buffer.rows * image_buffer.cols * image_buffer.channels());

	depth.header.frame_id = color.header.frame_id;
	depth.width = color.width;
	depth.height = color.height;
	depth.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
	depth.data = cv::Mat3f(color.height, color.width);

	Zed zed_msg;
	zed_msg.color = color;
	zed_msg.depth = depth;

	this->publisher->publish(zed_msg);
    }
    
public:
    VideoCaptureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: rclcpp::Node("video_capture", options),
	  frame_id(0)
    {
	using namespace std::placeholders;
	this->declare_parameter<int>("camera", 1);
	this->camera = cv::VideoCapture(this->get_parameter("camera").get_parameter_value().get<int>());
	this->publisher = this->create_publisher<Zed>("zed", 1);
	this->shot_clock = this->create_wall_timer(0s, std::bind(&VideoCaptureNode::capture_image, this));
    }
};

int main (int ac, char *av[]) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<VideoCaptureNode>());
    rclcpp::shutdown();
}
