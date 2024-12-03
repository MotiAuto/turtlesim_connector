#include "turtlesim_connector/ts_connector.hpp"

namespace turtlesim_connector
{
    TurtleSimConnector::TurtleSimConnector(const rclcpp::NodeOptions&option):Node("turtlesim_connector", option)
    {
        cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            0, 
            std::bind(&TurtleSimConnector::cmd_callback, this, _1));

        pose_sub = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            0,
            std::bind(&TurtleSimConnector::pose_callback, this, _1));

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current", 0);
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 0);
    }

    void TurtleSimConnector::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_publisher_->publish(*msg);
    }

    void TurtleSimConnector::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, msg->theta);

        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "map";
        t.header.stamp = this->get_clock()->now();
        t.child_frame_id = "odom";

        t.transform.rotation.w = q.w();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        tf_broadcaster->sendTransform(t);

        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.orientation = t.transform.rotation;
        p.pose.position.x = t.transform.translation.x;
        p.pose.position.y = t.transform.translation.y;
        p.pose.position.z = t.transform.translation.z;

        odom_publisher->publish(p);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(turtlesim_connector::TurtleSimConnector)