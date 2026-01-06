#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("Received Image", img);
    cv::waitKey(1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("subscriber"), "cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_subscriber");
  auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image", 10, imageCallback);
  RCLCPP_INFO(rclcpp::get_logger("subscriber"), "Subscribed to /camera/image topic");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}