#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

static double yaw_from_quaternion(double w, double z) {
  return std::atan2(2.0 * w * z, 1.0 - 2.0 * z * z);
}

static bool lookupTF(tf2_ros::Buffer& tf_buffer,
                     const std::string& target_frame,
                     const std::string& source_frame,
                     geometry_msgs::msg::PoseStamped& out_pose,
                     rclcpp::Logger logger,
                     int max_retry = 20) {
  for (int i = 0; i < max_retry; ++i) {
    try {
      auto t = tf_buffer.lookupTransform(source_frame, target_frame, tf2::TimePointZero);
      out_pose.header.stamp = t.header.stamp;
      out_pose.header.frame_id = target_frame;
      out_pose.pose.position.x = t.transform.translation.x;
      out_pose.pose.position.y = t.transform.translation.y;
      out_pose.pose.position.z = t.transform.translation.z;
      out_pose.pose.orientation = t.transform.rotation;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(logger, "lookupTF try %d/%d failed: %s", i+1, max_retry, ex.what());
      std::this_thread::sleep_for(50ms);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger, "lookupTF generic exception: %s", e.what());
      std::this_thread::sleep_for(50ms);
    }
  }
  return false;
}

static double get_euclidean_distance(const std::pair<double,double>& a, const std::pair<double,double>& b) {
  return std::hypot(a.first - b.first, a.second - b.second);
}

class PurePursuitTracker : public rclcpp::Node {
public:
  PurePursuitTracker()
  : Node("pure_pursuit_tracker"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this, true) {
    
    declare_parameter("control_hz", 30.0);
    declare_parameter("debug", true);
    declare_parameter("lookahead_distance", 0.5);
    declare_parameter("lookahead_time", 1.5);
    declare_parameter("min_lookahead_distance", 0.3);
    declare_parameter("max_lookahead_distance", 0.9);
    declare_parameter("angle_to_first_point_on_path", 0.785);
    declare_parameter("min_distance_to_first_lookahead", 0.7);
    declare_parameter("k_first_lookahead", 1.5);
    declare_parameter("use_velocity_scaled_lookahead", true);
    declare_parameter("desired_linear_vel", 0.3);
    declare_parameter("desired_angular_vel", 0.35);
    declare_parameter("use_regulated_linear_velocity_scaling", true);
    declare_parameter("regulated_linear_scaling_min_radius", 0.90);
    declare_parameter("velocity_scaling_distance", 0.5);
    declare_parameter("k_rot", 1.0);
    declare_parameter("k_linear", 1.0);
    declare_parameter("v_max", 0.5);
    declare_parameter("dv_max", 0.1);
    declare_parameter("w_max", 0.5);
    declare_parameter("dw_max", 0.25);
    declare_parameter("goal_position_tolerance", 0.20);
    declare_parameter("vicinity_to_path", 0.5);
    declare_parameter("use_rotate_to_heading", true);
    declare_parameter("rotate_to_heading_min_angle", 0.785);
    declare_parameter("obstacle_threshold", 0.5);

    control_hz_ = get_parameter("control_hz").as_double();
    debug_ = get_parameter("debug").as_bool();
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    lookahead_time_ = get_parameter("lookahead_time").as_double();
    min_lookahead_distance_ = get_parameter("min_lookahead_distance").as_double();
    max_lookahead_distance_ = get_parameter("max_lookahead_distance").as_double();
    angle_to_first_point_on_path_ = get_parameter("angle_to_first_point_on_path").as_double();
    min_distance_to_first_lookahead_ = get_parameter("min_distance_to_first_lookahead").as_double();
    k_first_lookahead_ = get_parameter("k_first_lookahead").as_double();
    use_velocity_scaled_lookahead_ = get_parameter("use_velocity_scaled_lookahead").as_bool();
    desired_linear_vel_ = get_parameter("desired_linear_vel").as_double();
    desired_angular_vel_ = get_parameter("desired_angular_vel").as_double();
    use_regulated_linear_velocity_scaling_ = get_parameter("use_regulated_linear_velocity_scaling").as_bool();
    regulated_linear_scaling_min_radius_ = get_parameter("regulated_linear_scaling_min_radius").as_double();
    velocity_scaling_distance_ = get_parameter("velocity_scaling_distance").as_double();
    k_rot_ = get_parameter("k_rot").as_double();
    k_linear_ = get_parameter("k_linear").as_double();
    v_max_ = get_parameter("v_max").as_double();
    dv_max_ = get_parameter("dv_max").as_double();
    w_max_ = get_parameter("w_max").as_double();
    dw_max_ = get_parameter("dw_max").as_double();
    goal_position_tolerance_ = get_parameter("goal_position_tolerance").as_double();
    vicinity_to_path_ = get_parameter("vicinity_to_path").as_double();
    use_rotate_to_heading_ = get_parameter("use_rotate_to_heading").as_bool();
    rotate_to_heading_min_angle_ = get_parameter("rotate_to_heading_min_angle").as_double();
    obstacle_threshold_ = get_parameter("obstacle_threshold").as_double();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/pure_pursuit/lookahead_point", 10);
    
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&PurePursuitTracker::scan_callback, this, std::placeholders::_1));
    
    rclcpp::QoS path_qos(rclcpp::KeepLast(1));
    path_qos.reliable().transient_local();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", path_qos, std::bind(&PurePursuitTracker::path_callback, this, std::placeholders::_1));
    
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&PurePursuitTracker::odom_cb, this, std::placeholders::_1), sub_options);

    execution_thread_ = std::thread(&PurePursuitTracker::execution_loop, this);
    x_ = y_ = yaw_ = v_ = w_ = prev_x_ = prev_y_ = 0.0;
  }

  ~PurePursuitTracker() {
    running_.store(false);
    exec_cv_.notify_all();
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

private:
  double x_, y_, yaw_, v_, w_;
  double prev_x_, prev_y_;
  bool oriented_to_path_;
  double angle_between_bot_and_path_;
  std::mutex obstacle_mutex_;
  std::atomic<bool> obstacle_detected_{false};
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::deque<std::pair<double,double>> current_path_;
  std::mutex path_mutex_;
  std::condition_variable exec_cv_;
  std::thread execution_thread_;
  std::atomic<bool> running_{true};
  std::atomic<bool> request_preempt_{false};
  bool reached_goal_ = false;

  double control_hz_;
  bool debug_;
  double lookahead_distance_;
  double lookahead_time_;
  double min_lookahead_distance_;
  double max_lookahead_distance_;
  double angle_to_first_point_on_path_;
  double min_distance_to_first_lookahead_;
  double k_first_lookahead_;
  bool use_velocity_scaled_lookahead_;
  double desired_linear_vel_;
  double desired_angular_vel_;
  bool use_regulated_linear_velocity_scaling_;
  double regulated_linear_scaling_min_radius_;
  double velocity_scaling_distance_;
  double k_rot_;
  double k_linear_;
  double v_max_;
  double dv_max_;
  double w_max_;
  double dw_max_;
  double goal_position_tolerance_;
  double vicinity_to_path_;
  bool use_rotate_to_heading_;
  double rotate_to_heading_min_angle_;
  double obstacle_threshold_;

  double calculate_curvature(const std::pair<double,double>& lookahead_point) {
    double chord_length_2 = lookahead_point.first * lookahead_point.first + 
                           lookahead_point.second * lookahead_point.second;
    if (chord_length_2 < 0.001) return 0.00001;
    
    double curvature = 2.0 * lookahead_point.second / chord_length_2;
    return (curvature > 0.0 ? 1.0 : -1.0) * std::max(0.00001, std::abs(curvature));
  }

  bool should_rotate_to_path(const std::pair<double,double>& lookahead_point) {
    return use_rotate_to_heading_ && 
           std::abs(std::atan2(lookahead_point.second, lookahead_point.first)) > rotate_to_heading_min_angle_;
  }

  double apply_v_constraints(double v_cmd, double curvature, const std::pair<double,double>& goal) {
    if (use_regulated_linear_velocity_scaling_) {
      double r = std::abs(1.0 / curvature);
      if (r < regulated_linear_scaling_min_radius_) {
        v_cmd *= (1.0 - std::abs(r - regulated_linear_scaling_min_radius_) / regulated_linear_scaling_min_radius_);
      }
    }

    double d = get_euclidean_distance({x_, y_}, goal);
    if (d < velocity_scaling_distance_) {
      v_cmd *= d / velocity_scaling_distance_;
    }

    double dv = std::clamp(v_cmd - v_, -dv_max_, dv_max_);
    return std::clamp(v_ + dv, -v_max_, v_max_);
  }

  double apply_w_constraints(double w_cmd) {
    return std::clamp(w_cmd, -w_max_, w_max_);
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Dynamic Path Planner initialized");
    reached_goal_ = false;
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    current_path_.clear();
    for (const auto& pose : msg->poses) {
      current_path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    request_preempt_.store(true);
    exec_cv_.notify_all();
  }

  void execution_loop() {
    std::unique_lock<std::mutex> lk(path_mutex_);
    while (running_.load() && !reached_goal_) {
      exec_cv_.wait_for(lk, 100ms, [&]{ 
        return !current_path_.empty() || request_preempt_.load() || !running_.load(); 
      });
      
      if (!running_.load()) break;
      if (request_preempt_.load()) {
        request_preempt_.store(false);
      }
      if (current_path_.empty()) continue;

      auto path = current_path_;
      lk.unlock();
      follow_path(path);
      lk.lock();
    }
  }

  double get_lookahead_distance() {
    if (!use_velocity_scaled_lookahead_) return lookahead_distance_;
    
    double ld = std::abs(v_) * lookahead_time_;
    return std::clamp(ld, min_lookahead_distance_, max_lookahead_distance_);
  }

  std::deque<std::pair<double,double>> trim_path_to_lookahead_point(
      std::deque<std::pair<double,double>> path, double lookahead_distance) {
    int i = 0;
    while (i < static_cast<int>(path.size()) && i <= 15) {
      double d = get_euclidean_distance({x_, y_}, path.front());
      if (d < lookahead_distance + 1e-6 && path.size() > 1) {
        path.pop_front();
      } else {
        break;
      }
      ++i;
    }
    return path;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacle_detected_ = false;
    
    constexpr double min_check_angle = -60.0 * M_PI / 180.0;
    constexpr double max_check_angle = 60.0 * M_PI / 180.0;
    constexpr double obstacle_threshold_ = 0.5;
    
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double current_angle = msg->angle_min + i * msg->angle_increment;
      
      if (current_angle >= min_check_angle && current_angle <= max_check_angle) {
        double range = msg->ranges[i];
        if (std::isfinite(range) && range > msg->range_min && 
            range < msg->range_max && range < obstacle_threshold_) {
          obstacle_detected_ = true;
          if (debug_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Obstacle detected at %.2fm, angle: %.2f°", 
                               range, current_angle * 180.0 / M_PI);
          }
          return;
        }
      }
    }
  }
  
  void follow_path(std::deque<std::pair<double,double>> path) {
    size_t target_idx = 0;
    rclcpp::Rate rate(30);
  
    while (rclcpp::ok() && target_idx < path.size() && !request_preempt_.load() && !reached_goal_) {
      geometry_msgs::msg::PoseStamped current_ps;
      if (!lookupTF(tf_buffer_, "base_footprint", "odom", current_ps, get_logger())) {
        RCLCPP_WARN(get_logger(), "TF lookup failed. Skipping control step");
        rate.sleep();
        continue;
      }
  
      x_ = current_ps.pose.position.x;
      y_ = current_ps.pose.position.y;
      yaw_ = yaw_from_quaternion(current_ps.pose.orientation.w, current_ps.pose.orientation.z);
  
      double d = get_euclidean_distance({x_, y_}, path.back());
      if (d < 0.2 && path.size() < 10) {
        set_cmd_vel(0.0, 0.0);
        RCLCPP_INFO(get_logger(), "Successfully reached goal");
        reached_goal_ = true;
        return;
      }
  
      bool obstacle_present;
      {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        obstacle_present = obstacle_detected_;
      }
  
      if (obstacle_present) {
        set_cmd_vel(0.0, 0.0);
        if (debug_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Obstacle in path - stopping robot");
        }
        rate.sleep();
        continue;
      }
  
      double lookahead_distance = get_lookahead_distance();
      path = trim_path_to_lookahead_point(path, lookahead_distance);
  
      auto lookahead_point = in_bf(path.front());
      double lookahead_curvature = calculate_curvature(lookahead_point);
  
      double v_cmd = 0.0;
      double w_cmd = 0.0;
  
      if (should_rotate_to_path(lookahead_point)) {
        double dyaw = std::atan2(lookahead_point.second, lookahead_point.first);
        double w = (dyaw > 0.0 ? 1.0 : -1.0) * desired_angular_vel_;
        double dw = std::clamp(w - w_, -dw_max_, dw_max_);
        w_cmd = std::clamp(w_ + dw, -w_max_, w_max_);
      } else {
        v_cmd = k_linear_ * desired_linear_vel_;
        v_cmd = apply_v_constraints(v_cmd, lookahead_curvature, path.back());
        w_cmd = apply_w_constraints(k_rot_ * v_cmd * lookahead_curvature);
      }
  
      geometry_msgs::msg::PointStamped lookahead_msg;
      lookahead_msg.header.stamp = now();
      lookahead_msg.header.frame_id = "odom";
      lookahead_msg.point.x = path.front().first;
      lookahead_msg.point.y = path.front().second;
      lookahead_pub_->publish(lookahead_msg);
  
      set_cmd_vel(v_cmd, w_cmd);
      rate.sleep();
    }
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    v_ = msg->twist.twist.linear.x;
    w_ = msg->twist.twist.angular.z;
  }

  void set_cmd_vel(double v_cmd, double w_cmd) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v_cmd;
    msg.angular.z = w_cmd;
    cmd_pub_->publish(msg);
  }

  std::pair<double,double> in_bf(const std::pair<double,double>& point) {
    double s = std::sin(yaw_), c = std::cos(yaw_);
    double px = point.first - x_, py = point.second - y_;
    return {c * px + s * py, -s * px + c * py};
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitTracker>());
  rclcpp::shutdown();
  return 0;
}