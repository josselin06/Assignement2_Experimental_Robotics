#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <plansys2_executor/ActionExecutorClient.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <chrono>
#include <fstream>
#include <set>
#include <string>
#include <unordered_map>
#include <sstream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <optional>

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

  int id;
  double x, y;
  while (f >> id >> x >> y) {
    m[id] = {x, y};
  }
  return m;
}

static void write_id2pose_file(const std::string & path,
                               const std::unordered_map<int, std::pair<double,double>> & m)
{
  std::vector<int> ids;
  ids.reserve(m.size());
  for (const auto & kv : m) ids.push_back(kv.first);
  std::sort(ids.begin(), ids.end());

  std::ofstream f(path, std::ios::trunc);
  for (int id : ids) {
    const auto & p = m.at(id);
    f << id << " " << p.first << " " << p.second << "\n";
  }
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

static std::string join_ids_sorted(const std::set<int> & ids)
{
  std::ostringstream oss;
  bool first = true;
  for (int id : ids) {
    if (!first) oss << ", ";
    first = false;
    oss << id;
  }
  return oss.str();
}

class ExploreAction : public plansys2::ActionExecutorClient
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ExploreAction()
  : plansys2::ActionExecutorClient("explore", 250ms)
  {
    
    this->declare_parameter<std::string>("action", "explore");

    this->declare_parameter<std::string>("image_topic", "/camera/image");
    this->declare_parameter<std::string>("map_topic", "/map");

    this->declare_parameter<std::string>("nav_action_name", "/navigate_to_pose");
    this->declare_parameter<std::string>("nav_goal_frame", "map");

    
    this->declare_parameter<double>("scan_duration_s", 4.0);

    
    this->declare_parameter<double>("detect_near_dist_m", 1.0);
    this->declare_parameter<bool>("detect_only_near_during_nav", true);

    
    this->declare_parameter<bool>("enable_approach_search", true);
    this->declare_parameter<double>("approach_radius_m", 0.60);
    this->declare_parameter<int>("approach_max_poses", 6);
    this->declare_parameter<double>("approach_min_free_cell", 50.0);

    
    this->declare_parameter<double>("wait_nav_server_timeout_s", 20.0);
    this->declare_parameter<double>("overall_timeout_s", 1000.0);

    
    this->declare_parameter<double>("clip_margin_m", 0.50);
    this->declare_parameter<int>("max_clip_steps", 40);

    
    this->declare_parameter<bool>("reset_ids_file_on_start", true);

    
    this->declare_parameter<bool>("cancel_nav_on_new_marker", true);

    
    this->declare_parameter<int>("log_image_every_n", 30);

    image_topic_ = this->get_parameter("image_topic").as_string();
    map_topic_   = this->get_parameter("map_topic").as_string();

    nav_action_name_ = this->get_parameter("nav_action_name").as_string();
    nav_goal_frame_  = this->get_parameter("nav_goal_frame").as_string();

    scan_duration_s_ = this->get_parameter("scan_duration_s").as_double();

    detect_near_dist_m_ = this->get_parameter("detect_near_dist_m").as_double();
    detect_only_near_during_nav_ = this->get_parameter("detect_only_near_during_nav").as_bool();

    enable_approach_search_ = this->get_parameter("enable_approach_search").as_bool();
    approach_radius_m_ = this->get_parameter("approach_radius_m").as_double();
    approach_max_poses_ = this->get_parameter("approach_max_poses").as_int();
    approach_min_free_cell_ = this->get_parameter("approach_min_free_cell").as_double();

    wait_nav_server_timeout_s_ = this->get_parameter("wait_nav_server_timeout_s").as_double();
    overall_timeout_s_ = this->get_parameter("overall_timeout_s").as_double();

    clip_margin_m_ = this->get_parameter("clip_margin_m").as_double();
    max_clip_steps_ = this->get_parameter("max_clip_steps").as_int();

    reset_ids_file_on_start_ = this->get_parameter("reset_ids_file_on_start").as_bool();
    cancel_nav_on_new_marker_ = this->get_parameter("cancel_nav_on_new_marker").as_bool();

    log_image_every_n_ = this->get_parameter("log_image_every_n").as_int();

    
    waypoints_["wp1"] = {  -6.0,  -6.0};
    waypoints_["wp2"] = {  -6.0, 6.0};
    waypoints_["wp3"] = { 6.0, -6.0};
    waypoints_["wp4"] = { 6.0,  6.0};

    
    if (reset_ids_file_on_start_) {
      {
        std::ofstream f(ids_file_, std::ios::trunc);
        (void)f;
      }
      {
        std::ofstream f(id2pose_file_, std::ios::trunc);
        (void)f;
      }
    }

    global_found_ids_ = read_ids_file(ids_file_);
    id_to_pose_ = read_id2pose_file(id2pose_file_);

    nav_client_  = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);

    
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&ExploreAction::map_cb, this, std::placeholders::_1));

    
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ExploreAction::image_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "[explore] ready. image=%s map=%s nav=%s frame=%s scan=%.1fs detect_near=%.2fm detect_only_near_nav=%s cancel_nav_on_new_marker=%s approach_search=%s radius=%.2f poses=%d id2pose=%s",
      image_topic_.c_str(), map_topic_.c_str(),
      nav_action_name_.c_str(),
      nav_goal_frame_.c_str(),
      scan_duration_s_,
      detect_near_dist_m_,
      detect_only_near_during_nav_ ? "true" : "false",
      cancel_nav_on_new_marker_ ? "true" : "false",
      enable_approach_search_ ? "true" : "false",
      approach_radius_m_, approach_max_poses_,
      id2pose_file_.c_str());
  }

private:
  enum class Phase {
    IDLE,
    WAIT_NAV_SERVER,
    WAIT_MAP,
    SEND_NAV_GOAL,
    WAIT_NAV_RESULT,
    WAIT_SCAN,
    DECIDE_AFTER_SCAN,
    SEND_APPROACH_GOAL,
    WAIT_APPROACH_NAV,
    WAIT_APPROACH_SCAN
  };

  
  bool compute_map_bounds(double & min_x, double & max_x, double & min_y, double & max_y) const
  {
    if (!have_map_) return false;
    const auto & info = last_map_.info;
    const double res = info.resolution;
    min_x = info.origin.position.x;
    min_y = info.origin.position.y;
    max_x = min_x + (double)info.width  * res;
    max_y = min_y + (double)info.height * res;
    return true;
  }

  bool in_map_bounds(double x, double y) const
  {
    double min_x, max_x, min_y, max_y;
    if (!compute_map_bounds(min_x, max_x, min_y, max_y)) return true; 
    return (x >= min_x + clip_margin_m_ &&
            x <= max_x - clip_margin_m_ &&
            y >= min_y + clip_margin_m_ &&
            y <= max_y - clip_margin_m_);
  }

  std::pair<double,double> clip_to_map(double x, double y) const
  {
    if (!have_map_) return {x, y};
    double min_x=0, max_x=0, min_y=0, max_y=0;
    (void)compute_map_bounds(min_x, max_x, min_y, max_y);
    const double cx = std::min(std::max(x, min_x + clip_margin_m_), max_x - clip_margin_m_);
    const double cy = std::min(std::max(y, min_y + clip_margin_m_), max_y - clip_margin_m_);
    return {cx, cy};
  }

  bool map_cell_is_free(double x, double y) const
  {
    if (!have_map_) return true;

    const auto & info = last_map_.info;
    const double res = info.resolution;
    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;

    const int mx = (int)std::floor((x - ox) / res);
    const int my = (int)std::floor((y - oy) / res);

    if (mx < 0 || my < 0 || mx >= (int)info.width || my >= (int)info.height) return false;

    const int idx = my * (int)info.width + mx;
    if (idx < 0 || idx >= (int)last_map_.data.size()) return false;

    const int8_t v = last_map_.data[idx];
    if (v < 0) return true; 
    return (double)v < approach_min_free_cell_;
  }

  
  double dist_robot_to(double tx, double ty) const
  {
    if (!have_robot_pose_) return 1e9;
    const double dx = tx - robot_x_;
    const double dy = ty - robot_y_;
    return std::sqrt(dx*dx + dy*dy);
  }

  
  bool get_detection_target(double & tx, double & ty) const
  {
    if (current_wp_.empty()) return false;

    auto it = waypoints_.find(current_wp_);
    if (it == waypoints_.end()) return false;

    tx = it->second.first;
    ty = it->second.second;

    if ((phase_ == Phase::WAIT_APPROACH_NAV || phase_ == Phase::WAIT_APPROACH_SCAN) &&
        approach_index_ >= 0 && approach_index_ < (int)approach_poses_.size())
    {
      tx = approach_poses_[approach_index_].first;
      ty = approach_poses_[approach_index_].second;
    }

    return true;
  }

  void reset_for_new_goal(const std::vector<std::string> & args)
  {
    phase_ = Phase::IDLE;
    current_wp_.clear();

    ids_in_this_wp_.clear();
    saw_new_marker_this_goal_ = false;

    have_nav_result_ = false;
    nav_goal_active_ = false;
    requested_nav_cancel_ = false;
    current_nav_goal_.reset();

    clip_steps_ = 0;
    goal_is_intermediate_ = false;

    
    approach_index_ = 0;
    approach_poses_.clear();

    
    global_found_ids_ = read_ids_file(ids_file_);
    id_to_pose_ = read_id2pose_file(id2pose_file_);

    goal_start_steady_ = std::chrono::steady_clock::now();
    nav_wait_start_steady_ = std::chrono::steady_clock::now();
    scan_start_steady_ = std::chrono::steady_clock::time_point{};
    last_goal_signature_ = join_args(args);

    image_count_ = 0;

    have_robot_pose_ = false;

    RCLCPP_INFO(get_logger(), "[explore] NEW GOAL args=[%s] global_ids=[%s]",
      last_goal_signature_.c_str(),
      join_ids_sorted(global_found_ids_).c_str());
  }

  void build_approach_poses()
  {
    approach_poses_.clear();
    approach_index_ = 0;

    auto it = waypoints_.find(current_wp_);
    const double wx = it->second.first;
    const double wy = it->second.second;

    const int N = std::max(0, approach_max_poses_);
    if (N == 0) return;

    for (int i = 0; i < N; ++i) {
      const double a = (2.0 * M_PI) * ((double)i / (double)N);
      double gx = wx + approach_radius_m_ * std::cos(a);
      double gy = wy + approach_radius_m_ * std::sin(a);

      if (!in_map_bounds(gx, gy)) {
        auto clipped = clip_to_map(gx, gy);
        gx = clipped.first;
        gy = clipped.second;
      }

      if (!map_cell_is_free(gx, gy)) {
        continue;
      }

      approach_poses_.push_back({gx, gy});
    }

    RCLCPP_INFO(get_logger(), "[explore] approach poses built: %zu (radius=%.2f)",
                approach_poses_.size(), approach_radius_m_);
  }

  void maybe_print_recap()
  {
    if (visited_wps_.size() < 4) return;

    std::vector<std::string> order = {"wp1","wp2","wp3","wp4"};

    std::ostringstream oss;
    oss << "\n========== ARUCO RECAP (4 waypoints visited) ==========\n";
    for (const auto & wp : order) {
      oss << " - " << wp << " : ";
      auto it = ids_by_wp_.find(wp);
      if (it == ids_by_wp_.end() || it->second.empty()) {
        oss << "(none)";
      } else {
        oss << join_ids_sorted(it->second);
      }
      oss << "\n";
    }
    oss << " GLOBAL IDs : ";
    if (global_found_ids_.empty()) oss << "(none)";
    else oss << join_ids_sorted(global_found_ids_);
    oss << "\n======================================================";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  
  void do_work() override
  {
    const auto args = get_arguments();

    if (args.empty()) {
      if (had_goal_) {
        had_goal_ = false;
        phase_ = Phase::IDLE;
      }
      return;
    }

    if (!had_goal_) {
      had_goal_ = true;
      reset_for_new_goal(args);
    }

    if (args.size() != 4) {
      finish(false, 1.0, "explore(): invalid number of arguments (expected 4)");
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
      finish(false, 1.0, "explore(): overall timeout");
      had_goal_ = false;
      phase_ = Phase::IDLE;
      return;
    }

    if (phase_ == Phase::IDLE) {
      const std::string wp = args[1];
      if (waypoints_.find(wp) == waypoints_.end()) {
        finish(false, 1.0, "explore(): invalid waypoint '" + wp + "'");
        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      current_wp_ = wp;
      ids_in_this_wp_.clear();
      saw_new_marker_this_goal_ = false;

      nav_wait_start_steady_ = std::chrono::steady_clock::now();
      goal_start_steady_ = std::chrono::steady_clock::now();

      send_feedback(0.0, "Starting explore to " + current_wp_);
      phase_ = Phase::WAIT_NAV_SERVER;
    }

    if (phase_ == Phase::WAIT_NAV_SERVER) {
      if (nav_client_->wait_for_action_server(200ms)) {
        phase_ = Phase::WAIT_MAP;
      } else {
        const double waited =
          std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - nav_wait_start_steady_).count();
        send_feedback(0.0, "Waiting for " + nav_action_name_ + "...");
        if (waited > wait_nav_server_timeout_s_) {
          finish(false, 1.0, "explore(): nav server not available: " + nav_action_name_);
          had_goal_ = false;
          phase_ = Phase::IDLE;
        }
        return;
      }
    }

    if (phase_ == Phase::WAIT_MAP) {
      if (!have_map_) {
        send_feedback(0.05, "Waiting for /map...");
        return;
      }
      phase_ = Phase::SEND_NAV_GOAL;
    }

    if (phase_ == Phase::SEND_NAV_GOAL) {
      auto it = waypoints_.find(current_wp_);
      const double target_x = it->second.first;
      const double target_y = it->second.second;

      double goal_x = target_x;
      double goal_y = target_y;

      if (!in_map_bounds(target_x, target_y)) {
        if (clip_steps_ >= max_clip_steps_) {
          finish(false, 1.0, "explore(): waypoint stays outside map bounds after many steps");
          had_goal_ = false;
          phase_ = Phase::IDLE;
          return;
        }

        auto clipped = clip_to_map(target_x, target_y);
        goal_x = clipped.first;
        goal_y = clipped.second;

        goal_is_intermediate_ = true;
        clip_steps_++;

        std::ostringstream msg;
        msg << "Waypoint (" << target_x << "," << target_y << ") out of map -> clipped ("
            << goal_x << "," << goal_y << ") step " << clip_steps_ << "/" << max_clip_steps_;
        send_feedback(0.10, msg.str());
      } else {
        goal_is_intermediate_ = false;
        send_feedback(0.10, "Going to final waypoint " + current_wp_);
      }

      NavigateToPose::Goal goal;
      goal.pose.header.frame_id = nav_goal_frame_;
      goal.pose.header.stamp = this->now();
      goal.pose.pose.position.x = goal_x;
      goal.pose.pose.position.y = goal_y;
      goal.pose.pose.orientation.w = 1.0;

      have_nav_result_ = false;
      nav_goal_active_ = false;
      requested_nav_cancel_ = false;
      current_nav_goal_.reset();

      auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

      opts.goal_response_callback =
        [this](GoalHandleNav::SharedPtr handle) {
          if (!handle) {
            RCLCPP_ERROR(get_logger(), "[explore] Nav2 goal rejected");
            have_nav_result_ = true;
            finish(false, 1.0, "explore(): Nav2 goal rejected");
            had_goal_ = false;
            phase_ = Phase::IDLE;
            return;
          }
          current_nav_goal_ = handle;
          nav_goal_active_ = true;
        };

      opts.feedback_callback =
        [this](GoalHandleNav::SharedPtr,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
          if (!feedback) return;
          robot_x_ = feedback->current_pose.pose.position.x;
          robot_y_ = feedback->current_pose.pose.position.y;
          have_robot_pose_ = true;
        };

      opts.result_callback = std::bind(&ExploreAction::nav_result_cb, this, std::placeholders::_1);

      (void)nav_client_->async_send_goal(goal, opts);
      phase_ = Phase::WAIT_NAV_RESULT;
      return;
    }

    if (phase_ == Phase::WAIT_NAV_RESULT) {
      if (cancel_nav_on_new_marker_ && saw_new_marker_this_goal_ &&
          nav_goal_active_ && current_nav_goal_ && !requested_nav_cancel_)
      {
        requested_nav_cancel_ = true;
        RCLCPP_INFO(get_logger(), "[explore] NEW marker seen during NAV (near) -> cancel nav and scan (wp=%s)",
                    current_wp_.c_str());
        (void)nav_client_->async_cancel_goal(current_nav_goal_);
      }

      if (!have_nav_result_) {
        send_feedback(0.2, "Navigating...");
        return;
      }
      return;
    }

    if (phase_ == Phase::WAIT_SCAN) {
      const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - scan_start_steady_).count();

      const double prog = std::min(1.0, elapsed / scan_duration_s_);
      send_feedback(0.2 + 0.7 * prog, "Scanning " + current_wp_);

      if (elapsed >= scan_duration_s_) {
        phase_ = Phase::DECIDE_AFTER_SCAN;
      }
      return;
    }

    if (phase_ == Phase::DECIDE_AFTER_SCAN) {
      ids_by_wp_[current_wp_].insert(ids_in_this_wp_.begin(), ids_in_this_wp_.end());
      visited_wps_.insert(current_wp_);

      if (!ids_in_this_wp_.empty()) {
        global_found_ids_ = read_ids_file(ids_file_);
        global_found_ids_.insert(ids_in_this_wp_.begin(), ids_in_this_wp_.end());
        write_ids_file(ids_file_, global_found_ids_);

        std::ostringstream msg;
        msg << "Explore finished at " << current_wp_
            << " (found " << ids_in_this_wp_.size() << " ids)"
            << " ids_wp=[" << join_ids_sorted(ids_in_this_wp_) << "]"
            << " global_ids=[" << join_ids_sorted(global_found_ids_) << "]";
        finish(true, 1.0, msg.str());

        maybe_print_recap();

        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      if (enable_approach_search_) {
        build_approach_poses();
        if (!approach_poses_.empty()) {
          RCLCPP_WARN(get_logger(), "[explore] 0 marker at waypoint %s -> approach search", current_wp_.c_str());
          phase_ = Phase::SEND_APPROACH_GOAL;
          return;
        }
      }

      global_found_ids_ = read_ids_file(ids_file_);
      std::ostringstream msg;
      msg << "Explore finished at " << current_wp_ << " (0 marker detected)";
      finish(true, 1.0, msg.str());

      maybe_print_recap();

      had_goal_ = false;
      phase_ = Phase::IDLE;
      return;
    }

    if (phase_ == Phase::SEND_APPROACH_GOAL) {
      if (approach_index_ >= (int)approach_poses_.size()) {
        global_found_ids_ = read_ids_file(ids_file_);
        std::ostringstream msg;
        msg << "Explore finished at " << current_wp_
            << " (0 marker after approach search)";
        finish(true, 1.0, msg.str());

        maybe_print_recap();

        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      const auto [gx0, gy0] = approach_poses_[approach_index_];
      double gx = gx0, gy = gy0;

      if (!in_map_bounds(gx, gy)) {
        auto clipped = clip_to_map(gx, gy);
        gx = clipped.first;
        gy = clipped.second;
      }

      NavigateToPose::Goal goal;
      goal.pose.header.frame_id = nav_goal_frame_;
      goal.pose.header.stamp = this->now();
      goal.pose.pose.position.x = gx;
      goal.pose.pose.position.y = gy;
      goal.pose.pose.orientation.w = 1.0;

      have_nav_result_ = false;
      nav_goal_active_ = false;
      requested_nav_cancel_ = false;
      current_nav_goal_.reset();
      saw_new_marker_this_goal_ = false;

      auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

      opts.goal_response_callback =
        [this](GoalHandleNav::SharedPtr handle) {
          if (!handle) {
            RCLCPP_WARN(get_logger(), "[explore] Approach nav goal rejected -> skip");
            have_nav_result_ = true;
            return;
          }
          current_nav_goal_ = handle;
          nav_goal_active_ = true;
        };

      opts.feedback_callback =
        [this](GoalHandleNav::SharedPtr ,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
          if (!feedback) return;
          robot_x_ = feedback->current_pose.pose.position.x;
          robot_y_ = feedback->current_pose.pose.position.y;
          have_robot_pose_ = true;
        };

      opts.result_callback = std::bind(&ExploreAction::approach_nav_result_cb, this, std::placeholders::_1);

      (void)nav_client_->async_send_goal(goal, opts);

      RCLCPP_INFO(get_logger(), "[explore] approach %d/%zu -> go (%.2f, %.2f)",
                  approach_index_+1, approach_poses_.size(), gx, gy);

      phase_ = Phase::WAIT_APPROACH_NAV;
      return;
    }

    if (phase_ == Phase::WAIT_APPROACH_NAV) {
      if (cancel_nav_on_new_marker_ && saw_new_marker_this_goal_ &&
          nav_goal_active_ && current_nav_goal_ && !requested_nav_cancel_)
      {
        requested_nav_cancel_ = true;
        RCLCPP_INFO(get_logger(), "[explore] NEW marker seen during APPROACH NAV (near) -> cancel and scan");
        (void)nav_client_->async_cancel_goal(current_nav_goal_);
      }

      if (!have_nav_result_) {
        send_feedback(0.4, "Approaching around waypoint...");
        return;
      }
      return;
    }

    if (phase_ == Phase::WAIT_APPROACH_SCAN) {
      const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - scan_start_steady_).count();

      const double prog = std::min(1.0, elapsed / scan_duration_s_);
      send_feedback(0.4 + 0.5 * prog, "Approach scanning...");

      if (!ids_in_this_wp_.empty()) {
        ids_by_wp_[current_wp_].insert(ids_in_this_wp_.begin(), ids_in_this_wp_.end());
        visited_wps_.insert(current_wp_);

        global_found_ids_ = read_ids_file(ids_file_);
        global_found_ids_.insert(ids_in_this_wp_.begin(), ids_in_this_wp_.end());
        write_ids_file(ids_file_, global_found_ids_);

        std::ostringstream msg;
        msg << "Explore finished at " << current_wp_
            << " (found after approach) ids_wp=[" << join_ids_sorted(ids_in_this_wp_) << "]"
            << " global_ids=[" << join_ids_sorted(global_found_ids_) << "]";
        finish(true, 1.0, msg.str());

        maybe_print_recap();

        had_goal_ = false;
        phase_ = Phase::IDLE;
        return;
      }

      if (elapsed >= scan_duration_s_) {
        approach_index_++;
        phase_ = Phase::SEND_APPROACH_GOAL;
      }
      return;
    }
  }

  
  void nav_result_cb(const GoalHandleNav::WrappedResult & result)
  {
    have_nav_result_ = true;
    nav_goal_active_ = false;

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (goal_is_intermediate_) {
        phase_ = Phase::SEND_NAV_GOAL;
        return;
      }

      RCLCPP_INFO(get_logger(), "[explore] reached %s -> start scan window %.1fs", current_wp_.c_str(), scan_duration_s_);
      scan_start_steady_ = std::chrono::steady_clock::now();
      phase_ = Phase::WAIT_SCAN;
      return;
    }

    if (cancel_nav_on_new_marker_ && saw_new_marker_this_goal_ &&
        result.code == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_INFO(get_logger(), "[explore] nav canceled after detecting marker (near) -> scan window %.1fs", scan_duration_s_);
      scan_start_steady_ = std::chrono::steady_clock::now();
      phase_ = Phase::WAIT_SCAN;
      return;
    }

    const int code = static_cast<int>(result.code);
    RCLCPP_ERROR(get_logger(), "[explore] nav2 failed for %s (code=%d)", current_wp_.c_str(), code);
    finish(false, 1.0, "explore(): Nav2 failed (code=" + std::to_string(code) + ")");
    had_goal_ = false;
    phase_ = Phase::IDLE;
  }

  void approach_nav_result_cb(const GoalHandleNav::WrappedResult & result)
  {
    have_nav_result_ = true;
    nav_goal_active_ = false;

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      scan_start_steady_ = std::chrono::steady_clock::now();
      phase_ = Phase::WAIT_APPROACH_SCAN;
      return;
    }

    if (cancel_nav_on_new_marker_ && saw_new_marker_this_goal_ &&
        result.code == rclcpp_action::ResultCode::CANCELED)
    {
      scan_start_steady_ = std::chrono::steady_clock::now();
      phase_ = Phase::WAIT_APPROACH_SCAN;
      return;
    }

    approach_index_++;
    phase_ = Phase::SEND_APPROACH_GOAL;
  }

  
  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    last_map_ = *msg;
    have_map_ = true;
  }

  
  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!had_goal_) return;

    const bool phase_allows_detection =
      (phase_ == Phase::WAIT_NAV_RESULT) ||
      (phase_ == Phase::WAIT_SCAN) ||
      (phase_ == Phase::WAIT_APPROACH_NAV) ||
      (phase_ == Phase::WAIT_APPROACH_SCAN);

    if (!phase_allows_detection) return;

    
    if (detect_only_near_during_nav_) {
      const bool is_nav_phase = (phase_ == Phase::WAIT_NAV_RESULT) || (phase_ == Phase::WAIT_APPROACH_NAV);
      if (is_nav_phase) {
        double tx, ty;
        if (get_detection_target(tx, ty) && have_robot_pose_) {
          const double d = dist_robot_to(tx, ty);
          if (d > detect_near_dist_m_) {
            return;
          }
        } else {
          return;
        }
      }
    }

    image_count_++;
    if (log_image_every_n_ > 0 && (image_count_ % log_image_every_n_ == 0)) {
      double tx=0, ty=0;
      double d = -1.0;
      if (get_detection_target(tx, ty) && have_robot_pose_) d = dist_robot_to(tx, ty);

      RCLCPP_INFO(get_logger(),
        "[explore] image rx (%dx%d) enc='%s' phase=%d wp=%s ids_wp=%zu dist_to_target=%.2f",
        (int)msg->width, (int)msg->height, msg->encoding.c_str(),
        (int)phase_, current_wp_.c_str(), ids_in_this_wp_.size(), d);
    }

    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "[explore] cv_bridge exception: %s (enc='%s')",
                   e.what(), msg->encoding.c_str());
      return;
    }

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

    if (ids.empty()) return;

    std::set<int> new_ids_this_frame;
    bool mapping_changed = false;

    
    auto it_wp = waypoints_.find(current_wp_);
    const bool have_wp_pose = (it_wp != waypoints_.end());
    const double wp_x = have_wp_pose ? it_wp->second.first  : 0.0;
    const double wp_y = have_wp_pose ? it_wp->second.second : 0.0;

    for (int id : ids) {
      ids_in_this_wp_.insert(id);
      ids_by_wp_[current_wp_].insert(id);

      
      if (global_found_ids_.insert(id).second) {
        new_ids_this_frame.insert(id);
        saw_new_marker_this_goal_ = true;
      }

      
      if (have_wp_pose && id_to_pose_.find(id) == id_to_pose_.end()) {
        id_to_pose_[id] = {wp_x, wp_y};
        mapping_changed = true;
      }
    }

    if (mapping_changed) {
      write_id2pose_file(id2pose_file_, id_to_pose_);
      RCLCPP_INFO(get_logger(),
        "[explore] 🧠 id->pose updated (wp=%s -> %.2f %.2f) -> wrote %s",
        current_wp_.c_str(), wp_x, wp_y, id2pose_file_.c_str());
    }

    if (!new_ids_this_frame.empty()) {
      write_ids_file(ids_file_, global_found_ids_);

      const char * tag =
        (phase_ == Phase::WAIT_NAV_RESULT || phase_ == Phase::WAIT_APPROACH_NAV) ? "NAV_NEAR" : "SCAN";

      RCLCPP_INFO(get_logger(),
        "[explore] ✅ NEW ArUco IDs detected (%s, wp=%s): %s  -> wrote %s",
        tag, current_wp_.c_str(),
        join_ids_sorted(new_ids_this_frame).c_str(),
        ids_file_.c_str());
    }
  }

  
  std::string image_topic_;
  std::string map_topic_;
  std::string nav_action_name_;
  std::string nav_goal_frame_;

  double scan_duration_s_{4.0};

  double detect_near_dist_m_{1.0};
  bool detect_only_near_during_nav_{true};

  bool enable_approach_search_{true};
  double approach_radius_m_{0.60};
  int approach_max_poses_{6};
  double approach_min_free_cell_{50.0};

  double wait_nav_server_timeout_s_{20.0};
  double overall_timeout_s_{1000.0};

  double clip_margin_m_{0.5};
  int max_clip_steps_{40};

  bool reset_ids_file_on_start_{true};
  bool cancel_nav_on_new_marker_{true};

  int log_image_every_n_{30};

  
  Phase phase_{Phase::IDLE};
  bool had_goal_{false};
  std::string current_wp_;

  bool have_nav_result_{false};
  GoalHandleNav::SharedPtr current_nav_goal_;
  bool nav_goal_active_{false};
  bool requested_nav_cancel_{false};

  bool have_map_{false};
  nav_msgs::msg::OccupancyGrid last_map_;

  int clip_steps_{0};
  bool goal_is_intermediate_{false};

  std::chrono::steady_clock::time_point goal_start_steady_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point nav_wait_start_steady_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point scan_start_steady_{};

  std::unordered_map<std::string, std::pair<double,double>> waypoints_;

  std::set<int> ids_in_this_wp_;
  std::set<int> global_found_ids_;
  bool saw_new_marker_this_goal_{false};

  std::string last_goal_signature_;
  size_t image_count_{0};

  
  std::vector<std::pair<double,double>> approach_poses_;
  int approach_index_{0};

  
  std::set<std::string> visited_wps_;
  std::unordered_map<std::string, std::set<int>> ids_by_wp_;

  
  bool have_robot_pose_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};

  
  const std::string ids_file_    = "/tmp/aruco_ids.txt";
  const std::string id2pose_file_ = "/tmp/aruco_id2pose.txt";

  
  std::unordered_map<int, std::pair<double,double>> id_to_pose_;

  
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreAction>();

  node->set_parameter(rclcpp::Parameter("action", "explore"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
