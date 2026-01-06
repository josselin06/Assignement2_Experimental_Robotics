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
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  // TransportHints does not actually declare the parameter
  node->declare_parameter<std::string>("image_transport", "raw");
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::TransportHints hints(node.get());
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback, &hints);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  return 0;
}