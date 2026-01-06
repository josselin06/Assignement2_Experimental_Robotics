#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <set>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <sstream>

struct MarkerInfo
{
  cv::Point2f center;      
  bool processed = false;
};

enum class State
{
  SCAN,         
  GOTO_MARKER,  
  DONE
};

class ArucoExplorer : public rclcpp::Node
{
public:
  ArucoExplorer() : Node("aruco_explorer")
  {
    this->declare_parameter<std::string>("image_topic", "camera/image");
    this->declare_parameter<std::string>("image_transport", "raw");
    this->declare_parameter<int>("expected_markers", 5);          // found 5 IDs
    this->declare_parameter<double>("center_tolerance_px", 5.0); //we can reduce this parameter to really center the marker as much as possible

    image_topic_ = this->get_parameter("image_topic").as_string();
    std::string transport   = this->get_parameter("image_transport").as_string();
    expected_markers_       = this->get_parameter("expected_markers").as_int();
    center_tolerance_px_    = this->get_parameter("center_tolerance_px").as_double();

    RCLCPP_INFO(this->get_logger(),
                "ArucoExplorer started. image_topic='%s', image_transport='%s', expected_markers=%d",
                image_topic_.c_str(), transport.c_str(), expected_markers_);

    sub_ = image_transport::create_subscription(
      this,
      image_topic_,
      std::bind(&ArucoExplorer::imageCallback, this, std::placeholders::_1),
      transport
    );

    // Image debug 
    pub_image_ = image_transport::create_publisher(this, "camera/aruco_debug");

    // Image to see the marker with the red circle 
    selected_pub_ = image_transport::create_publisher(this, "camera/marker_selected");

    // Command for robot speed
    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "ArucoExplorer initialized, waiting for images...");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // Conversion ROS to OpenCV
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    auto params     = cv::aruco::DetectorParameters::create();

    // Detection settings 
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    params->adaptiveThreshConstant = 7;
    params->minMarkerPerimeterRate = 0.01;
    params->maxMarkerPerimeterRate = 4.0;
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 30;
    params->cornerRefinementMinAccuracy = 0.1;

    //Detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, params);
    std::map<int, cv::Point2f> frame_centers;

    //Security 
    if (!ids.empty()) {
      for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        const auto & pts = corners[i];
        if (pts.size() < 4) {
          continue;
        }

        float cx = 0.0f;
        float cy = 0.0f;
        for (int k = 0; k < 4; ++k) {
          cx += pts[k].x;
          cy += pts[k].y;
        }
        cx /= 4.0f;
        cy /= 4.0f;

        frame_centers[id] = cv::Point2f(cx, cy);

        MarkerInfo & info = markers_[id];
        info.center = frame_centers[id];

        if (seen_ids_.insert(id).second) {
          RCLCPP_INFO(this->get_logger(),
                      "New marker stored: ID = %d, center = (%.1f, %.1f)",
                      id, cx, cy);
        }
      }

      int remaining = countRemainingMarkers();
      if (state_ == State::SCAN) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[SCAN] Currently stored %zu unique marker IDs",
          markers_.size());
      } else if (state_ == State::GOTO_MARKER) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[GOTO_MARKER] Markers remaining to center: %d",
          remaining);
      }

      cv::aruco::drawDetectedMarkers(frame, corners, ids);
    } 
    

    // SCAN / GOTO_MARKER / DONE
    geometry_msgs::msg::Twist cmd;  

    switch (state_) {
      case State::SCAN:
        handleScanState(cmd);
        break;

      case State::GOTO_MARKER:
        handleGotoMarkerState(cmd, frame, frame_centers, msg->header);
        break;

      case State::DONE:
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        break;
    }

    cmd_pub_->publish(cmd);

    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    pub_image_.publish(out_msg);

    if (has_selected_image_ && last_selected_msg_) {
      selected_pub_.publish(last_selected_msg_);
    }
  }

  // State management

  void handleScanState(geometry_msgs::msg::Twist & cmd)
  {
    // Until we see all the expected markers, continue to go around in circles
    if (static_cast<int>(markers_.size()) < expected_markers_) {
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.5;  //constant rotation

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[SCAN] markers_seen=%zu / %d -> rotating...",
        markers_.size(), expected_markers_);
    } else {
      // 5/5 markers found 
      int next_id = findNextUnprocessedId();
      if (next_id < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[SCAN] markers_ full mais pas d'ID dispo ? Passage en DONE.");
        state_ = State::DONE;
        return;
      }

      // Before proceeding to GOTO_MARKER, we display the sorted list of detected IDs
      std::vector<int> ids_sorted;
      ids_sorted.reserve(markers_.size());
      for (const auto & kv : markers_) {
        ids_sorted.push_back(kv.first);
      }
      std::sort(ids_sorted.begin(), ids_sorted.end());

      std::stringstream ss;
      for (size_t i = 0; i < ids_sorted.size(); ++i) {
        if (i > 0) {
          ss << ", ";
        }
        ss << ids_sorted[i];
      }

      RCLCPP_INFO(this->get_logger(),
                  "[SCAN] Markers detected (sorted IDs): %s",
                  ss.str().c_str());

      current_target_id_ = next_id;
      int remaining = countRemainingMarkers();

      RCLCPP_INFO(this->get_logger(),
                  "[SCAN] Found %d markers. First target ID = %d. Remaining to center: %d. Switching to GOTO_MARKER.",
                  expected_markers_, current_target_id_, remaining);

      // stop Robot and go to GOTO_MARKER
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      state_ = State::GOTO_MARKER;
    }
  }

  void handleGotoMarkerState(geometry_msgs::msg::Twist & cmd,
                             cv::Mat & frame,
                             const std::map<int, cv::Point2f> & frame_centers,
                             const std_msgs::msg::Header & header)
  {
    if (current_target_id_ < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[GOTO_MARKER] current_target_id_ invalide, passage en DONE");
      state_ = State::DONE;
      return;
    }

    auto it_center = frame_centers.find(current_target_id_);
    if (it_center == frame_centers.end()) {
      // Marker not visible so constant rotation 
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.5;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[GOTO_MARKER] Target ID %d not visible, rotating to search...", current_target_id_);
      return;
    }

    // Marker visible so look the horizontal position 
    const cv::Point2f & c = it_center->second;
    double image_center_x = frame.cols / 2.0;
    double error_x = c.x - image_center_x;  // >0 : marker on the right 

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[GOTO_MARKER] ID %d center=(%.1f, %.1f), error_x=%.1f px",
      current_target_id_, c.x, c.y, error_x);

    if (std::fabs(error_x) <= center_tolerance_px_) {
      // Marker sufficiently centered -> we stop the robot
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(),
                  "[GOTO_MARKER] Target ID %d centered within %.1f px -> mark as processed",
                  current_target_id_, center_tolerance_px_);

      // draw a red circle 
      int radius = 80;     
      int thickness = 4;
      cv::circle(frame, c, radius, cv::Scalar(0, 0, 255), thickness); 

      auto selected_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      last_selected_msg_ = selected_msg;
      has_selected_image_ = true;

      selected_pub_.publish(last_selected_msg_);

      // ID done 
      markers_[current_target_id_].processed = true;

      int remaining_after = countRemainingMarkers();
      RCLCPP_INFO(this->get_logger(),
                  "[GOTO_MARKER] Markers remaining to center after ID %d: %d",
                  current_target_id_, remaining_after);

      // looking for the next ID
      int next_id = findNextUnprocessedId();
      if (next_id < 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[GOTO_MARKER] All markers processed. Switching to DONE.");
        state_ = State::DONE;
      } else {
        current_target_id_ = next_id;
        RCLCPP_INFO(this->get_logger(),
                    "[GOTO_MARKER] Next target ID = %d. Staying in GOTO_MARKER.",
                    current_target_id_);
      }
    } else {
      // Rotational control proportional to error
      double image_half_width = frame.cols / 2.0;
      double norm_error = error_x / image_half_width;
      double Kp = -0.5; 
      cmd.angular.z = Kp * norm_error;
      cmd.linear.x  = 0.0;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[GOTO_MARKER] Rotating to center ID %d (cmd.angular.z=%.3f)",
        current_target_id_, cmd.angular.z);
    }
  }

  int findNextUnprocessedId() const
  {
    int best_id = -1;
    for (const auto & kv : markers_) {
      int id = kv.first;
      const MarkerInfo & info = kv.second;
      if (!info.processed) {
        if (best_id < 0 || id < best_id) {
          best_id = id;
        }
      }
    }
    return best_id;
  }

  int countRemainingMarkers() const
  {
    int count = 0;
    for (const auto & kv : markers_) {
      if (!kv.second.processed) {
        ++count;
      }
    }
    return count;
  }

  image_transport::Subscriber sub_;
  image_transport::Publisher  pub_image_;      // /camera/aruco_debug
  image_transport::Publisher  selected_pub_;   // /camera/marker_selected
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  std::set<int> seen_ids_;
  std::map<int, MarkerInfo> markers_;

  std::string image_topic_;
  int expected_markers_ = 5;
  double center_tolerance_px_ = 20.0;

  State state_ = State::SCAN;
  int current_target_id_ = -1;

  sensor_msgs::msg::Image::SharedPtr last_selected_msg_;
  bool has_selected_image_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoExplorer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}