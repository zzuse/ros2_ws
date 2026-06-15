#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace nav2_custom_controller
{
    void CustomController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        if (!node_)
        {
            throw std::runtime_error("Failed to lock node in CustomController::configure");
        }

        plugin_name_ = name;
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        // costmap_ = costmap_ros_->getCostmap();

        // Get parameters from the parameter server
        nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
        node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
        nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
        node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
    }

    void CustomController::cleanup()
    {
        global_plan_.poses.clear();
        RCLCPP_INFO(node_->get_logger(), "CustomController cleaned up");
    }

    void CustomController::activate()
    {

        RCLCPP_INFO(node_->get_logger(), "CustomController activated");
    }

    void CustomController::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "CustomController deactivated");
    }

    geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;

        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Global plan is empty, stopping the robot");
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            return cmd_vel;
        }

        geometry_msgs::msg::PoseStamped pose_in_globalframe;
        if (!nav2_util::transformPoseInTargetFrame(pose, pose_in_globalframe, *tf_, global_plan_.header.frame_id, 0.1))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to transform current pose to global frame");
            return cmd_vel;
        }

        // Get the nearest target pose from the global plan
        geometry_msgs::msg::PoseStamped target_pose = getNearestTargetPose(pose_in_globalframe);

        // Calculate the angle difference between the current pose and the target pose
        double angle_diff = calculateAngleDifference(pose_in_globalframe, target_pose);
        cmd_vel.header.frame_id = pose_in_globalframe.header.frame_id;
        cmd_vel.header.stamp = node_->get_clock()->now();

        // angle_diff bigger than 0.314 radians (about 18 degrees), prioritize rotation
        if (fabs(angle_diff) > M_PI / 10.0)
        {
            cmd_vel.twist.linear.x = 0.0; // Stop linear movement when turning
            cmd_vel.twist.angular.z = fabs(angle_diff) / angle_diff * max_angular_speed_;
        }
        else
        {
            cmd_vel.twist.linear.x = max_linear_speed_;
            cmd_vel.twist.angular.z = 0.0; // No angular velocity when aligned
        }
        RCLCPP_INFO(node_->get_logger(), "Computed cmd_vel: linear.x=%.2f, angular.z=%.2f", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

        return cmd_vel;
    }

    void CustomController::setPlan(const nav_msgs::msg::Path &path)
    {
        global_plan_ = path;
        RCLCPP_INFO(node_->get_logger(), "Global plan set with %zu poses", global_plan_.poses.size());
    }

    void CustomController::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        if (percentage)
        {
            max_linear_speed_ = speed_limit * max_linear_speed_;
            max_angular_speed_ = speed_limit * max_angular_speed_;
        }
        else
        {
            max_linear_speed_ = speed_limit;
            max_angular_speed_ = speed_limit; // Assuming same limit for angular speed for simplicity
        }
        RCLCPP_INFO(node_->get_logger(), "Speed limits updated: max_linear_speed=%.2f, max_angular_speed=%.2f",
                    max_linear_speed_, max_angular_speed_);
    }

    geometry_msgs::msg::PoseStamped CustomController::getNearestTargetPose(const geometry_msgs::msg::PoseStamped &current_pose)
    {
        using nav2_util::geometry_utils::euclidean_distance;
        int nearest_pose_index = 0;
        double min_dist = euclidean_distance(current_pose.pose, global_plan_.poses[0].pose);

        for (unsigned int i = 1; i < global_plan_.poses.size(); i++)
        {
            double distance = euclidean_distance(current_pose.pose, global_plan_.poses.at(i).pose);
            if (distance < min_dist)
            {
                min_dist = distance;
                nearest_pose_index = i;
            }
        }

        global_plan_.poses.erase(global_plan_.poses.begin(), global_plan_.poses.begin() + nearest_pose_index);
        if (global_plan_.poses.size() == 1)
        {
            return global_plan_.poses[0];
        }
        return global_plan_.poses[1];
    }

    double CustomController::calculateAngleDifference(const geometry_msgs::msg::PoseStamped &current_pose, const geometry_msgs::msg::PoseStamped &target_pose)
    {
        double current_yaw = tf2::getYaw(current_pose.pose.orientation);
        double target_angle = std::atan2(target_pose.pose.position.y - current_pose.pose.position.y,
                                         target_pose.pose.position.x - current_pose.pose.position.x);
        double angle_diff = target_angle - current_yaw;

        // Normalize the angle difference to the range [-pi, pi]
        while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

        return angle_diff;
    }
} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)