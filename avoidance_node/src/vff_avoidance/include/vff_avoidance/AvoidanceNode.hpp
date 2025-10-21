#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

class AvoidanceNode : public rclcpp::Node
{
public:
    AvoidanceNode();

private:
    // Callbacks
    void timerCallback();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // VFF computations
    void computeVectors(); // computes att/rep/resultant (in robot frame)
    void publishCmd();     // publishes geometry_msgs/Twist
    void publishMarkers(); // publishes MarkerArray for debug

    // Helpers
    void sectorizeScan(const sensor_msgs::msg::LaserScan &scan,
                       std::vector<double> &sector_min_range,
                       std::vector<double> &sector_min_angle);

    // ROS entities
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr timer;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Parameters
    std::string laser_topic;
    std::string cmd_vel_topic;
    std::string marker_topic;
    std::string goal_topic;

    double attractive_gain;
    double repulsive_gain_base;
    double repulsive_exponent;
    double obstacle_threshold;
    int num_sectors;

    double resultant_alpha;
    double cmd_alpha;

    double max_linear_speed;
    double max_angular_speed;
    double angular_deadzone;
    double angular_gain;
    double angular_damping;

    double timer_frequency;

    // State
    sensor_msgs::msg::LaserScan::SharedPtr last_scan;
    geometry_msgs::msg::PoseStamped::SharedPtr last_goal_in_map; // raw goal as received
    // goal in robot frame:
    bool have_goal_in_robot_frame;
    double goal_x_robot;
    double goal_y_robot;

    double result_x; // raw resultant
    double result_y;
    double filt_result_x; // filtered resultant
    double filt_result_y;

    // command smoothing state
    double last_linear_cmd;
    double last_angular_cmd;
};