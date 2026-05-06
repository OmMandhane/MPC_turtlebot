#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

struct Point2D {
    double x, y;
    Point2D(double x = 0, double y = 0) : x(x), y(y) {}
    double dist(const Point2D& other) const {
        return std::hypot(x - other.x, y - other.y);
    }
};

class PathSmoother {
public:
    static std::vector<Point2D> smooth(const std::vector<Point2D>& waypoints, double resolution = 0.05) {
        if (waypoints.size() < 2) return waypoints;
        
        auto rounded = roundCorners(waypoints, 1.5);
        std::vector<Point2D> res;
        
        for (size_t i = 0; i < rounded.size() - 1; ++i) {
            double d = rounded[i].dist(rounded[i+1]);
            if (d < 1e-6) continue;
            
            int steps = std::max(1, static_cast<int>(std::ceil(d / resolution)));
            for (int s = 0; s < steps; ++s) {
                double t = static_cast<double>(s) / steps;
                res.emplace_back(
                    rounded[i].x + (rounded[i+1].x - rounded[i].x) * t,
                    rounded[i].y + (rounded[i+1].y - rounded[i].y) * t
                );
            }
        }
        res.push_back(rounded.back());
        return res;
    }
    
private:
    static std::vector<Point2D> roundCorners(const std::vector<Point2D>& waypoints, double radius) {
        if (waypoints.size() < 3) return waypoints;
        
        std::vector<Point2D> rounded;
        rounded.push_back(waypoints[0]);
        
        for (size_t i = 1; i < waypoints.size() - 1; i++) {
            const auto& prev = waypoints[i - 1];
            const auto& curr = waypoints[i];
            const auto& next = waypoints[i + 1];
            
            double dx1 = curr.x - prev.x, dy1 = curr.y - prev.y;
            double dx2 = next.x - curr.x, dy2 = next.y - curr.y;
            double len1 = std::hypot(dx1, dy1), len2 = std::hypot(dx2, dy2);
            
            if (len1 < 1e-6 || len2 < 1e-6) {
                rounded.push_back(curr);
                continue;
            }
            
            dx1 /= len1; dy1 /= len1;
            dx2 /= len2; dy2 /= len2;
            
            double dot = std::clamp(dx1*dx2 + dy1*dy2, -1.0, 1.0);
            double angle = std::acos(dot);
            
            if (angle > 0.1) {
                double offset = std::min({len1 * 0.3, len2 * 0.3, radius});
                
                Point2D p1(curr.x - dx1 * offset, curr.y - dy1 * offset);
                rounded.push_back(p1);
                
                for (int j = 1; j < 5; j++) {
                    double t = j / 5.0;
                    double s = 1 - t;
                    rounded.emplace_back(
                        s*s*p1.x + 2*s*t*curr.x + t*t*(curr.x + dx2*offset),
                        s*s*p1.y + 2*s*t*curr.y + t*t*(curr.y + dy2*offset)
                    );
                }
                rounded.emplace_back(curr.x + dx2 * offset, curr.y + dy2 * offset);
            } else {
                rounded.push_back(curr);
            }
        }
        
        rounded.push_back(waypoints.back());
        return rounded;
    }
};

class DynamicPathPlanner : public rclcpp::Node {
public:
    DynamicPathPlanner() : Node("dynamic_smoothing_node") {
        declare_parameter("lookahead_points", 10);
        declare_parameter("control_hz", 10.0);
        declare_parameter("path_file", "src/nav/waypoints/waypoint.csv");
        declare_parameter("frame_id", "odom");
        declare_parameter("path_resolution", 0.1);
        declare_parameter("visualize_paths", true);
        
        lookahead_pts_ = get_parameter("lookahead_points").as_int();
        control_hz_ = get_parameter("control_hz").as_double();
        frame_id_ = get_parameter("frame_id").as_string();
        path_resolution_ = get_parameter("path_resolution").as_double();
        visualize_paths_ = get_parameter("visualize_paths").as_bool();
        
        waypoints_viz_pub_ = create_publisher<nav_msgs::msg::Path>("/waypoints", 10);

        rclcpp::QoS path_qos(rclcpp::KeepLast(1));
        path_qos.reliable().transient_local();
        path_pub_ = create_publisher<nav_msgs::msg::Path>("path", path_qos);

        loadPathFromCSV();
        RCLCPP_INFO(get_logger(), "Dynamic Path Planner initialized");
    }
    
private:
    int lookahead_pts_;
    double control_hz_;
    std::string frame_id_;
    double path_resolution_;
    bool visualize_paths_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::vector<Point2D> current_path_;
    bool path_loaded_ = false;

    void loadPathFromCSV() {
        std::string file = get_parameter("path_file").as_string();
        std::ifstream in(file);
        if (!in.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open path file: %s", file.c_str());
            return;
        }
    
        std::vector<Point2D> waypoints;
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            double x, y;
            char comma;
            if (ss >> x >> comma >> y) {
                waypoints.emplace_back(x, y);
            }
        }
    
        if (waypoints.empty()) {
            RCLCPP_ERROR(get_logger(), "CSV file %s had no valid waypoints!", file.c_str());
            return;
        }

        publishPath(waypoints, waypoints_viz_pub_);
        current_path_ = PathSmoother::smooth(waypoints, path_resolution_);
        path_loaded_ = true;
        publishPath(current_path_, path_pub_);
        
        RCLCPP_INFO(get_logger(), "Published path with %zu points to Pure Pursuit topic.", current_path_.size());
    }
    
    void publishPath(const std::vector<Point2D>& path, 
                     const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub) {
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        
        for (const auto& pt : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }
        pub->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicPathPlanner>());
    rclcpp::shutdown();
    return 0;
}