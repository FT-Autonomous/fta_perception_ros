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
#include <perception_msgs/msg/zed.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

class VideoCaptureNode
    : public rclcpp::Node {
private:
    using ZedTopic = perception_msgs::msg::Zed;
    
    cv::VideoCapture camera;
    cv::Mat image_buffer;
    rclcpp::Publisher<ZedTopic>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr shot_clock;

private:
    void capture_image() {
	camera.read(this->image_buffer);
	ZedTopic zed_topic;
	sensor_msgs::msg::Image im;
	im.header.stamp = rclcpp::Time();
	im.width = image_buffer.cols;
	im.height = image_buffer.rows;
	im.encoding = sensor_msgs::image_encodings::BGR8;
	im.is_bigendian = false;
	im.step = image_buffer.cols * 3;
	im.data = image_buffer.reshape(1, image_buffer.rows * image_buffer.cols * image_buffer.channels());
	zed_topic.color = im;
	this->publisher->publish(zed_topic);
    }
public:
    VideoCaptureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: rclcpp::Node("video_capture", options)
    {
	using namespace std::placeholders;
	this->declare_parameter<int>("camera", 1);
	this->camera = cv::VideoCapture(this->get_parameter("camera").get_parameter_value().get<int>());
	this->publisher = this->create_publisher<ZedTopic>("feed", 1);
	this->shot_clock = this->create_wall_timer(0.5s, std::bind(&VideoCaptureNode::capture_image, this));
    }
};

int main (int ac, char *av[]) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<VideoCaptureNode>());
    rclcpp::shutdown();
}
