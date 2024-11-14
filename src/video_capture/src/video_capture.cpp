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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <ft_msgs/msg/zed.hpp>


        

using namespace std::chrono_literals;

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Zed = ft_msgs::msg::Zed;
using ZedPublihser = rclcpp::Publisher<Zed>;
using ImagePublisher = rclcpp::Publisher<Image>;
using CameraInfoPublisher = rclcpp::Publisher<CameraInfo>;

Image matToImage(const cv::Mat& mat) {
  Image image;
  image.width = mat.cols;
  image.height = mat.rows;
  image.encoding = sensor_msgs::image_encodings::BGR8;
  image.step = mat.cols * 3;
  image.data = mat.reshape(1, mat.rows * mat.cols * mat.channels());
  return image;
}

class VideoCaptureNode
    : public rclcpp::Node {
private:
    
    unsigned int frame_id;
    cv::VideoCapture camera;
    cv::Mat image_buffer;
    ZedPublihser::SharedPtr publisher;
    ImagePublisher::SharedPtr left_publisher;
    CameraInfoPublisher::SharedPtr left_info_publisher;
    ImagePublisher::SharedPtr right_publisher;
    CameraInfoPublisher::SharedPtr right_info_publisher;
    rclcpp::TimerBase::SharedPtr shot_clock;
    
private:
    void capture_image() {
      camera.read(this->image_buffer);

      // Check if the stereo image has the correct dimensions
    int width = image_buffer.cols / 2;

    // Define the region of interest for the left image
    cv::Rect left_roi(0, 0, width, image_buffer.rows);
    cv::Rect right_roi(width, 0, width, image_buffer.rows);

    cv::Mat left = image_buffer(left_roi).clone();
    Image left_image = matToImage(left);
    left_image.header.frame_id = "zed_camera_center";

    cv::Mat right = image_buffer(left_roi).clone();
    Image right_image = matToImage(right);
    right_image.header.frame_id = "zed_camera_center";

	Image color, depth;

        this->left_publisher->publish(left_image);
        this->right_publisher->publish(right_image);

        CameraInfo camera_info;
        camera_info.header.frame_id = "zed_camera_center";
        camera_info.width = width;
        camera_info.height = image_buffer.rows;
        double fx=700.819;
        double fy=700.819;
        double cx=665.465;
        double cy=371.953;
        double k1=-0.174318;
        double k2=0.0261121;
        std::array<double, 9> k = {
          fx, 0.0, cx,
          0.0, fy, cy,
          0.0, 0.0, 1.0
        };
        camera_info.k = k;
        std::array<double, 12> p = { fx, 0.0, cx, 0.0,
                                     0.0, fy, cy, 0.0,
                                     0.0, 0.0, 1.0, 0.0
        };
        camera_info.p = p;
        // camera_info.p = inputs["camera_info"]["P"];
        // camera_info.k = inputs["camera_info"]["K"];
        camera_info.distortion_model = "rational_polynomial";
        this->left_info_publisher->publish(camera_info);
        this->right_info_publisher->publish(camera_info);
        

	// depth.header.frame_id = color.header.frame_id;
	// depth.width = color.width;
	// depth.height = color.height;
	// depth.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
	// depth.data = cv::Mat3f(color.height, color.width);

	Zed zed_msg;
	zed_msg.color = color;
	// zed_msg.depth = depth;

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
	this->left_publisher = this->create_publisher<Image>("zed/zed_node/left/image_rect_color", 1);
	this->right_publisher = this->create_publisher<Image>("zed/zed_node/right/image_rect_color", 1);
        this->right_info_publisher = this->create_publisher<CameraInfo>("zed/zed_node/right/camera_info", 1);
        this->left_info_publisher = this->create_publisher<CameraInfo>("zed/zed_node/left/camera_info", 1);
	this->shot_clock = this->create_wall_timer(0s, std::bind(&VideoCaptureNode::capture_image, this));
    }
};

int main (int ac, char *av[]) {
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<VideoCaptureNode>());
    rclcpp::shutdown();
}
