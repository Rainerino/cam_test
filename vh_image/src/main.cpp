#include <cv_bridge/cv_bridge.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "gst/gstelement.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "vh_appsink.hpp"

using namespace std::chrono_literals;

class GStreamer2ROS : public rclcpp::Node {
 public:
  GStreamer2ROS() : Node("awa") {
    _publisher = this->create_publisher<sensor_msgs::msg::Image>("awa", 1);
    _subscriber =
        this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&GStreamer2ROS::image_callback, this, std::placeholders::_1));
    
    _timer = this->create_wall_timer(
        30ms, std::bind(&GStreamer2ROS::timer_callback, this));
    
  }
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "DO SOMETHING WITH ROS IMAGE HERE");
    // 1. translate ROS message to CV2
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image = cv_ptr->image;
    // 2. save cv2 to gstreamer something something magic
  }
  void timer_callback() {
    if (test_image.data == NULL) {
      RCLCPP_INFO(this->get_logger(), "?");
      return;
    }
    cv_bridge::CvImage cv_image;
    try {
      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();

      // copy image from global variable. 
      // TODO: remove this evil from the world
      cv_image = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    test_image);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
    // message.data.c_str());
    auto message = cv_image.toImageMsg();
    // message->encoding = "rgb8";
    // message->width = 1280;
    // message->height = 720;

    // RCLCPP_INFO(this->get_logger(), "%d", test_image.data[0]);
    _publisher->publish(*(message));
  }

 private:
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _publisher;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscriber;
};

int main(int argc, char *argv[]) {
  auto *app = new TestApp(argc, argv);
  app->exec();

  std::cout << "idle" << std::endl;
  std::cout << "HHHH" << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GStreamer2ROS>());
  rclcpp::shutdown();
  return 0;
}