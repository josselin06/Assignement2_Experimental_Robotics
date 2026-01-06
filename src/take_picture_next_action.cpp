#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;


static std::set<int> read_ids_file(const std::string & path)
{
  std::set<int> ids;
  std::ifstream f(path);
  if (!f.is_open()) return ids;
  int x;
  while (f >> x) ids.insert(x);
  return ids;
}

static void write_ids_file(const std::string & path, const std::set<int> & ids)
{
  std::ofstream f(path, std::ios::trunc);
  for (int id : ids) f << id << "\n";
}

static std::unordered_map<int, std::pair<double,double>> read_id2pose_file(const std::string & path)
{
  std::unordered_map<int, std::pair<double,double>> m;
  std::ifstream f(path);
  if (!f.is_open()) return m;

  int id; double x, y;
  while (f >> id >> x >> y) {
    m[id] = {x, y};
  }
  return m;
}

static int pick_next_id(const std::set<int> & all, const std::set<int> & done)
{
  for (int id : all) {
    if (done.find(id) == done.end()) return id;
  }
  return -1;
}

static std::string join_args(const std::vector<std::string> & args)
{
  std::ostringstream oss;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i) oss << " ";
    oss << args[i];
  }
  return oss.str();
}

class TakePictureNextAction : public plansys2::ActionExecutorClient
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  TakePictureNextAction()
  : plansys2::ActionExecutorClient("take_picture_next", 100ms)
  {
    this->declare_parameter<std::string>("action", "take_picture_next");

    this->declare_parameter<std::string>("image_topic", "/camera/image");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    this->declare_parameter<std::string>("nav_action_name", "/navigate_to_pose");
    this->declare_parameter<std::string>("nav_goal_frame", "map");

    
    this->declare_parameter<double>("center_tolerance_px", 6.0);
    this->declare_parameter<double>("search_ang_vel", 0.5);
    this->declare_parameter<double>("kp", -0.5);

    
    this->declare_parameter<double>("wait_nav_server_timeout_s", 10.0);
    this->declare_parameter<double>("overall_timeout_s", 120.0);

    
    this->declare_parameter<bool>("reset_done_file_on_start", true);
    this->declare_parameter<std::string>("ids_file", "/tmp/aruco_ids.txt");
    this->declare_parameter<std::string>("done_file", "/tmp/aruco_done.txt");
    this->declare_parameter<std::string>("id2pose_file", "/tmp/aruco_id2pose.txt");

    image_topic_ = this->get_parameter("image_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();

    nav_action_name_ = this->get_parameter("nav_action_name").as_string();
    nav_goal_frame_  = this->get_parameter("nav_goal_frame").as_string();

    center_tol_ = this->get_parameter("center_tolerance_px").as_double();
    search_wz_  = this->get_parameter("search_ang_vel").as_double();
    kp_         = this->get_parameter("kp").as_double();

    wait_nav_server_timeout_s_ = this->get_parameter("wait_nav_server_timeout_s").as_double();
    overall_timeout_s_ = this->get_parameter("overall_timeout_s").as_double();

    reset_done_file_on_start_ = this->get_parameter("reset_done_file_on_start").as_bool();

    ids_file_     = this->get_parameter("ids_file").as_string();
    done_file_    = this->get_parameter("done_file").as_string();
    id2pose_file_ = this->get_parameter("id2pose_file").as_string();

    if (reset_done_file_on_start_) {
      std::ofstream f(done_file_, std::ios::trunc);
      (void)f;
    }

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    selected_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/marker_selected", rclcpp::SensorDataQoS());

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TakePictureNextAction::image_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "[take_picture_next] ready. image=%s cmd=%s nav=%s frame=%s tol=%.1f ids=%s done=%s id2pose=%s",
      image_topic_.c_str(), cmd_vel_topic_.c_str(),
      nav_action_name_.c_str(), nav_goal_frame_.c_str(), center_tol_,
      ids_file_.c_str(), done_file_.c_str(), id2pose_file_.c_str());
  }

private:
  enum class Phase {
    IDLE,
    WAIT_NAV_SERVER,
    PICK_TARGET,
    SEND_NAV,
    WAIT_NAV_RESULT,
    CENTERING
  };

  void reset_for_new_goal(const std::vector<std::string> & args)
  {
    phase_ = Phase::WAIT_NAV_SERVER;

    target_id_ = -1;
    target_x_ = 0.0;
    target_y_ = 0.0;

    have_frame_ = false;
    last_centers_.clear();

    nav_goal_active_ = false;
    have_nav_result_ = false;
    current_nav_goal_.reset();

    goal_start_steady_ = std::chrono::steady_clock::now();
    nav_wait_start_steady_ = std::chrono::steady_clock::now();
    last_goal_signature_ = join_args(args);

    stop_robot();

    
    id2pose_ = read_id2pose_file(id2pose_file_);

    RCLCPP_INFO(get_logger(), "[take_picture_next] NEW GOAL args=[%s]", last_goal_signature_.c_str());
  }

  void do_work() override
  {
    const auto args = get_arguments();

    if (args.empty()) {
      if (had_goal_) {
        had_goal_ = false;
        phase_ = Phase::IDLE;
        stop_robot();
      }
      return;
    }

    if (!had_goal_) {
      had_goal_ = true;
      reset_for_new_goal(args);
    }

    
    if (args.size() != 4) {
      stop_robot();
      finish(false, 1.0, "take_picture_next(): invalid number of arguments (expected 4)");
      had_goal_ = false;
      phase_ = Phase::IDLE;
      return;
    }

    const std::string sig = join_args(args);
    if (sig != last_goal_signature_) {
      reset_for_new_goal(args);
    }

    const double overall_elapsed =
      std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - goal_start_steady_).count();

    if (overall_elapsed > overall_timeout_s_) {
      stop_robot();
      finish(false, 1.0, "take_picture_next(): overall timeout");
      had_goal_ = false;
      phase_ = Phase::IDLE;
      return;
    }

    
    if (phase_ == Phase::WAIT_NAV_SERVER) {
      if (nav_client_->wait_for_action_server(200ms)) {
        phase_ = Phase::PICK_TARGET;
      } else {
        const double waited =
          std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - nav_wait_start_steady_).count();

        send_feedback(0.0, "Waiting for " + nav_action_name_ + "...");
        if (waited > wait_nav_server_timeout_s_) {
          finish(false, 1.0, "take_picture_next(): nav server not available: " + nav_action_name_);
          had_goal_ = false;
          phase_ = Phase::IDLE;
        }
        return;
      }
    }

    
    if (phase_ == Phase::PICK_TARGET) {
      auto all  = read_ids_file(ids_file_);
      auto done = read_ids_file(done_file_);
      target_id_ = pick_next_id(all, done);

      if (target_id_ < 0) {
        stop_robot();
        finish(true, 1.0, "take_picture_next(): all marker IDs already processed");
        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      auto it = id2pose_.find(target_id_);
      if (it == id2pose_.end()) {
        stop_robot();
        finish(false, 1.0,
               "take_picture_next(): missing id->pose mapping for ID " + std::to_string(target_id_) +
               " (check " + id2pose_file_ + ")");
        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      target_x_ = it->second.first;
      target_y_ = it->second.second;

      RCLCPP_INFO(get_logger(), "[take_picture_next] target ID=%d -> go to (%.2f, %.2f)",
                  target_id_, target_x_, target_y_);

      phase_ = Phase::SEND_NAV;
      return;
    }

    
    if (phase_ == Phase::SEND_NAV) {
      NavigateToPose::Goal goal;
      goal.pose.header.frame_id = nav_goal_frame_;
      goal.pose.header.stamp = this->now();
      goal.pose.pose.position.x = target_x_;
      goal.pose.pose.position.y = target_y_;
      goal.pose.pose.orientation.w = 1.0;

      have_nav_result_ = false;
      nav_goal_active_ = false;
      current_nav_goal_.reset();

      auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

      opts.goal_response_callback =
        [this](GoalHandleNav::SharedPtr handle) {
          if (!handle) {
            RCLCPP_ERROR(get_logger(), "[take_picture_next] Nav2 goal rejected");
            have_nav_result_ = true;
            finish(false, 1.0, "take_picture_next(): Nav2 goal rejected");
            had_goal_ = false;
            phase_ = Phase::IDLE;
            return;
          }
          current_nav_goal_ = handle;
          nav_goal_active_ = true;
        };

      opts.result_callback =
        [this](const GoalHandleNav::WrappedResult & result) {
          have_nav_result_ = true;
          nav_goal_active_ = false;

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(),
              "[take_picture_next] reached target pose -> start CENTERING (ID=%d)", target_id_);
            phase_ = Phase::CENTERING;
            return;
          }

          const int code = static_cast<int>(result.code);
          RCLCPP_ERROR(get_logger(), "[take_picture_next] nav failed (code=%d)", code);
          finish(false, 1.0, "take_picture_next(): Nav2 failed (code=" + std::to_string(code) + ")");
          had_goal_ = false;
          phase_ = Phase::IDLE;
        };

      (void)nav_client_->async_send_goal(goal, opts);
      phase_ = Phase::WAIT_NAV_RESULT;
      send_feedback(0.1, "Navigating to marker area...");
      return;
    }

    if (phase_ == Phase::WAIT_NAV_RESULT) {
      if (!have_nav_result_) {
        send_feedback(0.2, "Navigating...");
        return;
      }
      return;
    }

    
    if (phase_ == Phase::CENTERING) {
      geometry_msgs::msg::Twist cmd;

      if (!have_frame_) {
        cmd.angular.z = search_wz_;
        cmd_pub_->publish(cmd);
        send_feedback(0.3, "Waiting for camera frames...");
        return;
      }

      auto it = last_centers_.find(target_id_);
      if (it == last_centers_.end()) {
        cmd.angular.z = search_wz_;
        cmd_pub_->publish(cmd);
        send_feedback(0.4, "Target not visible, rotating...");
        return;
      }

      const cv::Point2f c = it->second;

      const double image_center_x = last_frame_width_ / 2.0;
      const double error_x = (double)c.x - image_center_x;

      if (std::fabs(error_x) <= center_tol_) {
        stop_robot();

        
        cv::Mat pic = last_frame_.clone();

        
        cv::aruco::drawDetectedMarkers(pic, last_corners_, last_ids_);

        
        cv::circle(pic, c, 80, cv::Scalar(0, 0, 255), 4);

        
        cv::putText(pic,
                    "ID " + std::to_string(target_id_),
                    cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 255),
                    2);

        auto out = cv_bridge::CvImage(last_header_, "bgr8", pic).toImageMsg();
        selected_pub_->publish(*out);

        
        auto done = read_ids_file(done_file_);
        done.insert(target_id_);
        write_ids_file(done_file_, done);

        finish(true, 1.0, "Centered ID " + std::to_string(target_id_) + " (published /camera/marker_selected, saved done)");
        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      
      const double half = last_frame_width_ / 2.0;
      const double norm_err = (half > 1e-6) ? (error_x / half) : 0.0;

      cmd.angular.z = kp_ * norm_err;
      cmd.linear.x  = 0.0;
      cmd_pub_->publish(cmd);

      send_feedback(0.6, "Centering... err=" + std::to_string((int)error_x) + "px");
      return;
    }
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist z;
    cmd_pub_->publish(z);
  }


  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      return;
    }

    last_frame_ = frame;
    last_header_ = msg->header;
    last_frame_width_ = frame.cols;
    have_frame_ = true;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    auto params     = cv::aruco::DetectorParameters::create();


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

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, params);

    last_ids_ = ids;
    last_corners_ = corners;

    std::map<int, cv::Point2f> centers;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (corners[i].size() < 4) continue;
      float cx = 0.f, cy = 0.f;
      for (int k = 0; k < 4; k++) { cx += corners[i][k].x; cy += corners[i][k].y; }
      cx /= 4.f; cy /= 4.f;
      centers[ids[i]] = cv::Point2f(cx, cy);
    }

    last_centers_ = centers;
  }


  std::string image_topic_;
  std::string cmd_vel_topic_;
  std::string nav_action_name_;
  std::string nav_goal_frame_;

  double center_tol_{6.0};
  double search_wz_{0.5};
  double kp_{-0.5};

  double wait_nav_server_timeout_s_{10.0};
  double overall_timeout_s_{120.0};
  bool reset_done_file_on_start_{true};


  std::string ids_file_{"/tmp/aruco_ids.txt"};
  std::string done_file_{"/tmp/aruco_done.txt"};
  std::string id2pose_file_{"/tmp/aruco_id2pose.txt"};


  Phase phase_{Phase::IDLE};
  bool had_goal_{false};

  int target_id_{-1};
  double target_x_{0.0}, target_y_{0.0};

  std::chrono::steady_clock::time_point goal_start_steady_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point nav_wait_start_steady_{std::chrono::steady_clock::now()};
  std::string last_goal_signature_;


  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr current_nav_goal_;
  bool nav_goal_active_{false};
  bool have_nav_result_{false};

  bool have_frame_{false};
  cv::Mat last_frame_;
  std_msgs::msg::Header last_header_;
  int last_frame_width_{0};

  std::map<int, cv::Point2f> last_centers_;
  std::vector<int> last_ids_;
  std::vector<std::vector<cv::Point2f>> last_corners_;

  std::unordered_map<int, std::pair<double,double>> id2pose_;


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr selected_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakePictureNextAction>();

  node->set_parameter(rclcpp::Parameter("action", "take_picture_next"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
