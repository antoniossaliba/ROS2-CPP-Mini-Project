#include "vff_avoidance/AvoidanceNode.hpp"

#include <cmath>
#include <limits>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

using namespace std::chrono_literals;

AvoidanceNode::AvoidanceNode()
    : Node("avoidance_node"),
      tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
      have_goal_in_robot_frame(false),
      result_x(0.0), result_y(0.0),
      filt_result_x(0.0), filt_result_y(0.0),
      last_linear_cmd(0.0), last_angular_cmd(0.0)
{
    // Parameters (declare + get)
    laser_topic = this->declare_parameter<std::string>("laser_topic", "/scan");
    cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    marker_topic = this->declare_parameter<std::string>("marker_topic", "/vff_markers");
    goal_topic = this->declare_parameter<std::string>("goal_topic", "/goal_pose");

    attractive_gain = this->declare_parameter<double>("attractive_gain", 1.0);
    repulsive_gain_base = this->declare_parameter<double>("repulsive_gain_base", 0.8);
    repulsive_exponent = this->declare_parameter<double>("repulsive_exponent", 2.0);
    obstacle_threshold = this->declare_parameter<double>("obstacle_threshold", 1.0);

    num_sectors = this->declare_parameter<int>("num_sectors", 9);

    resultant_alpha = this->declare_parameter<double>("resultant_alpha", 0.2);
    cmd_alpha = this->declare_parameter<double>("cmd_alpha", 0.3);

    max_linear_speed = this->declare_parameter<double>("max_linear_speed", 0.4);
    max_angular_speed = this->declare_parameter<double>("max_angular_speed", 1.2);
    angular_deadzone = this->declare_parameter<double>("angular_deadzone", 0.05);
    angular_gain = this->declare_parameter<double>("angular_gain", 1.2);
    angular_damping = this->declare_parameter<double>("angular_damping", 2.0);

    timer_frequency = this->declare_parameter<double>("timer_frequency", 20.0);

    // Subs + pubs
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic, 10, std::bind(&AvoidanceNode::laserCallback, this, std::placeholders::_1));

    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, std::bind(&AvoidanceNode::goalCallback, this, std::placeholders::_1));

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

    // Timer
    auto period_ms = static_cast<int>(1000.0 / timer_frequency);
    timer = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                    std::bind(&AvoidanceNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "AvoidanceNode ready. Subscribed to %s, goal %s",
                laser_topic.c_str(), goal_topic.c_str());
}

void AvoidanceNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!msg)
    {
        RCLCPP_WARN(this->get_logger(), "Received null LaserScan");
        return;
    }

    // Validate timestamp
    auto now = this->now();
    auto msg_time = rclcpp::Time(msg->header.stamp);
    double age = (now - msg_time).seconds();
    if (age > 2.0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "LaserScan too old (%.2f s); ignoring until fresh data.", age);
        return;
    }

    last_scan = msg;
}

void AvoidanceNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg)
        return;
    last_goal_in_map = msg;

    // Transform goal into robot frame "base_link"
    geometry_msgs::msg::PoseStamped goal_in_robot;
    try
    {
        // Try to transform to base_link
        std::string target_frame = "base_link";
        goal_in_robot = tf_buffer->transform(*msg, target_frame, tf2::durationFromSec(0.2));
        goal_x_robot = goal_in_robot.pose.position.x;
        goal_y_robot = goal_in_robot.pose.position.y;
        have_goal_in_robot_frame = true;
        RCLCPP_DEBUG(this->get_logger(), "Received goal transformed to robot frame: (%.2f, %.2f)",
                     goal_x_robot, goal_y_robot);
    }
    catch (const std::exception &e)
    {
        // If transform fails, but goal is already in base_link frame, check header
        if (msg->header.frame_id == "base_link")
        {
            goal_x_robot = msg->pose.position.x;
            goal_y_robot = msg->pose.position.y;
            have_goal_in_robot_frame = true;
            RCLCPP_DEBUG(this->get_logger(), "Received goal in base_link frame: (%.2f, %.2f)",
                         goal_x_robot, goal_y_robot);
        }
        else
        {
            have_goal_in_robot_frame = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Cannot transform goal to base_link: %s", e.what());
        }
    }
}

void AvoidanceNode::sectorizeScan(const sensor_msgs::msg::LaserScan &scan,
                                  std::vector<double> &sector_min_range,
                                  std::vector<double> &sector_min_angle)
{
    int sectors = num_sectors;
    sector_min_range.assign(sectors, std::numeric_limits<double>::infinity());
    sector_min_angle.assign(sectors, 0.0);

    double total_angle = scan.angle_max - scan.angle_min;
    double sector_width = total_angle / static_cast<double>(sectors);

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        double r = scan.ranges[i];
        if (std::isnan(r) || std::isinf(r))
            continue;
        double angle = scan.angle_min + i * scan.angle_increment;

        int idx = static_cast<int>((angle - scan.angle_min) / sector_width);
        if (idx < 0)
            idx = 0;
        if (idx >= sectors)
            idx = sectors - 1;

        if (r < sector_min_range[idx])
        {
            sector_min_range[idx] = r;
            sector_min_angle[idx] = angle;
        }
    }
}

void AvoidanceNode::computeVectors()
{
    // Reset
    result_x = 0.0;
    result_y = 0.0;

    // Ensure we have a recent scan
    if (!last_scan)
        return;

    // Attractive vector toward goal (in robot frame)
    double att_x = 0.0;
    double att_y = 0.0;
    if (have_goal_in_robot_frame)
    {
        // goal vector = [goal_x_robot_, goal_y_robot_]
        double gx = goal_x_robot;
        double gy = goal_y_robot;
        double dist = std::hypot(gx, gy);
        if (dist > 1e-6)
        {
            // Normalize and scale by attractive_gain_
            att_x = attractive_gain * (gx / dist);
            att_y = attractive_gain * (gy / dist);
        }
    }
    else
    {
        // fallback: small constant forward attractor
        att_x = attractive_gain * 0.5;
        att_y = 0.0;
    }

    // Repulsive vector using sectorization
    std::vector<double> sector_min_range;
    std::vector<double> sector_min_angle;
    sectorizeScan(*last_scan, sector_min_range, sector_min_angle);

    double rep_x = 0.0;
    double rep_y = 0.0;

    for (int s = 0; s < num_sectors; ++s)
    {
        double r = sector_min_range[s];
        if (std::isfinite(r) && r < obstacle_threshold && r > last_scan->range_min)
        {
            // Compute dynamic gain that increases as r --> 0
            double d = std::max(1e-6, obstacle_threshold - r);
            double gain = repulsive_gain_base * std::pow((obstacle_threshold - r) / obstacle_threshold, repulsive_exponent);
            // Direction away from obstacle (angle is in robot frame relative to forward)
            double ang = sector_min_angle[s];
            double fx = -gain * std::cos(ang);
            double fy = -gain * std::sin(ang);
            rep_x += fx;
            rep_y += fy;
        }
    }

    // Sum up (Attractive + Repulsive)
    result_x = att_x + rep_x;
    result_y = att_y + rep_y;

    // Low-pass filter the resultant vector (exponential moving average)
    filt_result_x = resultant_alpha * filt_result_x + (1.0 - resultant_alpha) * result_x;
    filt_result_y = resultant_alpha * filt_result_y + (1.0 - resultant_alpha) * result_y;

    RCLCPP_DEBUG(this->get_logger(), "Att=(%.2f,%.2f) Rep=(%.2f,%.2f) Res=(%.2f,%.2f) Filt=(%.2f,%.2f)",
                 att_x, att_y, rep_x, rep_y, result_x, result_y, filt_result_x, filt_result_y);
}

void AvoidanceNode::publishCmd()
{
    geometry_msgs::msg::Twist cmd;

    // Use filtered resultant to produce commands:
    double rx = filt_result_x;
    double ry = filt_result_y;

    // Compute desired heading and magnitude
    double desired_angle = std::atan2(ry, rx); // radians (robot frame)
    double desired_mag = std::hypot(rx, ry);

    // Angular control with deadzone and damping
    double ang_err = desired_angle; // goal is forward in robot frame -> angle is error
    if (std::fabs(ang_err) < angular_deadzone)
    {
        ang_err = 0.0;
    }

    double raw_ang = angular_gain * ang_err;
    // Dampen/saturate smoothly (tanh)
    double ang_cmd = max_angular_speed * std::tanh(raw_ang / angular_damping);

    // Linear speed scales with magnitude but reduced for large heading errors
    double heading_factor = std::cos(desired_angle); // reduces forward speed if heading is sideways
    heading_factor = std::max(0.0, heading_factor);  // don't go backward due to heading factor
    double lin_cmd = std::min(desired_mag * heading_factor, max_linear_speed);

    // Smooth commands (exponential moving average)
    double out_lin = cmd_alpha * last_linear_cmd + (1.0 - cmd_alpha) * lin_cmd;
    double out_ang = cmd_alpha * last_angular_cmd + (1.0 - cmd_alpha) * ang_cmd;

    // Update trackers
    last_linear_cmd = out_lin;
    last_angular_cmd = out_ang;

    cmd.linear.x = out_lin;
    cmd.angular.z = out_ang;

    cmd_pub->publish(cmd);
    RCLCPP_DEBUG(this->get_logger(), "Cmd linear=%.3f angular=%.3f", out_lin, out_ang);
}

void AvoidanceNode::publishMarkers()
{
    if (marker_pub->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray ma;

    // Attractive (goal) marker - green
    visualization_msgs::msg::Marker att;
    att.header.frame_id = "base_link";
    att.header.stamp = this->now();
    att.ns = "vff";
    att.id = 0;
    att.type = visualization_msgs::msg::Marker::ARROW;
    att.action = visualization_msgs::msg::Marker::ADD;
    att.scale.x = 0.08;
    att.scale.y = 0.15;
    att.scale.z = 0.15;
    att.color.r = 0.0;
    att.color.g = 0.9;
    att.color.b = 0.0;
    att.color.a = 1.0;
    att.points.resize(2);
    att.points[0].x = 0.0;
    att.points[0].y = 0.0;
    att.points[0].z = 0.0;
    att.points[1].x = (have_goal_in_robot_frame ? goal_x_robot : attractive_gain * 0.5);
    att.points[1].y = (have_goal_in_robot_frame ? goal_y_robot : 0.0);
    ma.markers.push_back(att);

    // Repulsive sector markers - red (one per sector)
    std::vector<double> sector_min_range;
    std::vector<double> sector_min_angle;
    sectorizeScan(*last_scan, sector_min_range, sector_min_angle);
    for (int s = 0; s < num_sectors; ++s)
    {
        double r = sector_min_range[s];
        if (!std::isfinite(r) || r > obstacle_threshold)
            continue;
        visualization_msgs::msg::Marker rep;
        rep.header.frame_id = "base_link";
        rep.header.stamp = this->now();
        rep.ns = "vff";
        rep.id = 100 + s;
        rep.type = visualization_msgs::msg::Marker::ARROW;
        rep.action = visualization_msgs::msg::Marker::ADD;
        rep.scale.x = 0.06;
        rep.scale.y = 0.12;
        rep.scale.z = 0.12;
        rep.color.r = 0.9;
        rep.color.g = 0.0;
        rep.color.b = 0.0;
        rep.color.a = 1.0;
        rep.points.resize(2);
        rep.points[0].x = 0.0;
        rep.points[0].y = 0.0;
        rep.points[0].z = 0.0;
        double ang = sector_min_angle[s];
        // length proportional to (threshold - r)
        double len = std::min(0.8, obstacle_threshold - r);
        rep.points[1].x = -len * std::cos(ang); // away from obstacle
        rep.points[1].y = -len * std::sin(ang);
        ma.markers.push_back(rep);
    }

    // Resultant (blue)
    visualization_msgs::msg::Marker res;
    res.header.frame_id = "base_link";
    res.header.stamp = this->now();
    res.ns = "vff";
    res.id = 200;
    res.type = visualization_msgs::msg::Marker::ARROW;
    res.action = visualization_msgs::msg::Marker::ADD;
    res.scale.x = 0.1;
    res.scale.y = 0.18;
    res.scale.z = 0.18;
    res.color.r = 0.0;
    res.color.g = 0.0;
    res.color.b = 0.9;
    res.color.a = 1.0;
    res.points.resize(2);
    res.points[0].x = 0.0;
    res.points[0].y = 0.0;
    res.points[0].z = 0.0;
    res.points[1].x = filt_result_x;
    res.points[1].y = filt_result_y;
    ma.markers.push_back(res);

    marker_pub->publish(ma);
}

void AvoidanceNode::timerCallback()
{
    // If we don't have a fresh scan yet, skip
    if (!last_scan)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for LaserScan...");
        return;
    }

    // Compute vectors (att/rep/resultant)
    computeVectors();

    // Publish command
    publishCmd();

    // Publish markers (only if RViz subscribed)
    publishMarkers();
}
