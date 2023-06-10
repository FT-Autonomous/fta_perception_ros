#include "rclcpp/utilities.hpp"
#include <mutex>
#include <iostream>
#include <opencv2/core.hpp>
#include <functional>
#include <chrono>
#include <memory>
#include <sl/Camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <eufs_msgs/msg/zed.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using namespace std;

class ZedCapture
    : public rclcpp::Node {
private:
    using Image = sensor_msgs::msg::Image;
    using Zed = eufs_msgs::msg::Zed;
    bool enabled;
    mutex mtx;
    sl::Camera camera;
    sl::CameraConfiguration camera_configuration;
    sl::RuntimeParameters runtime_parameters;
    sl::Mat xyz;
    sl::Mat rgb;
    rclcpp::TimerBase::SharedPtr zed_timer;
    rclcpp::Publisher<Zed>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub;
    
public:
    ZedCapture()
	: rclcpp::Node("zed_capture")
    {
	using namespace std::placeholders;
	this->publisher = this->create_publisher<Zed>("zed", 1);
	this->start_zed();
	this->enabled_sub = this->create_subscription<std_msgs::msg::Bool>("enabled", 1, std::bind(&ZedCapture::enabled_cb, this, _1));
	RCLCPP_INFO(this->get_logger(), "Initialised ZEDCapture node");
    }

    void enabled_cb(std_msgs::msg::Bool::SharedPtr enabled) {
	if (enabled->data and !this->enabled) start_zed();
	else if (!enabled->data and this->enabled) stop_zed();
    }

    void start_zed() {
	this->enabled = true;
	sl::InitParameters init_parameters;
	init_parameters.camera_resolution = sl::RESOLUTION::HD720;
	init_parameters.depth_maximum_distance = 10000.0f;
	init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	auto returned_state = camera.open(init_parameters);

	if (returned_state != sl::ERROR_CODE::SUCCESS) {
	    RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to open ZED Camera" << returned_state);
	    rclcpp::shutdown();
	}

	RCLCPP_INFO(this->get_logger(), "Opened ZED Camera");
	this->runtime_parameters.confidence_threshold = 50;
	this->camera_configuration = this->camera.getCameraInformation().camera_configuration;
	this->zed_timer = this->create_wall_timer(100ms, std::bind(&ZedCapture::capture, this));
    }

    void stop_zed() {
	this->enabled = false;
	this->zed_timer->cancel();
	camera.close();
    }

    void capture() {
	    if (!mtx.try_lock()) return;
	auto code = this->camera.grab(this->runtime_parameters);
	if (sl::ERROR_CODE::SUCCESS == code) {
	    this->camera.retrieveMeasure(this->xyz, sl::MEASURE::XYZ);
	    this->camera.retrieveImage(this->rgb, sl::VIEW::LEFT);

	    Image depth, color;
	    auto frame_id = std::to_string(camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getSeconds());
	    
	    color.header.frame_id = frame_id;
	    color.width = this->rgb.getWidth();
	    color.height = this->rgb.getHeight();
	    color.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	    color.data.reserve(color.width * color.height);
	    for (int i = 0; i < color.width * color.height; i++) {
		auto color_vector = this->rgb.getPtr<sl::uchar4>()[i];
		color.data.push_back(color_vector[0]);
		color.data.push_back(color_vector[1]);
		color.data.push_back(color_vector[2]);
	    }

	    depth.header.frame_id = frame_id;
	    depth.width = color.width;
	    depth.height = color.height;
	    depth.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
	    depth.data.resize(4 * depth.width * depth.height * 3, 0);
	    auto depth_data = (float*) depth.data.data();
	    for (int i = 0; i < depth.width * depth.height; i++) {
		auto xyz_ = this->xyz.getPtr<sl::float4>()[i];
		depth_data[3*i] = xyz_[0];
		depth_data[3*i+1] = xyz_[1];
		depth_data[3*i+2] = xyz_[2];
	    }

	    Zed zed_msg;

	    zed_msg.color = color;
	    zed_msg.depth = depth;

	    this->publisher->publish(zed_msg);
	} else {
	    RCLCPP_WARN(this->get_logger(), "Could not grab image from ZED");
	}
	mtx.unlock();
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedCapture>());
    rclcpp::shutdown();
}
