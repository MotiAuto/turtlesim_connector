#ifndef TS_CONNECTOR_HPP_
#define TS_CONNECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

namespace turtlesim_connector
{
    class TurtleSimConnector : public rclcpp::Node
    {
        public:
        explicit TurtleSimConnector(const rclcpp::NodeOptions& option= rclcpp::NodeOptions());
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);

        private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    };
}

#endif