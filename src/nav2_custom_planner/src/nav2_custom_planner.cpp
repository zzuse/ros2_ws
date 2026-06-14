#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner
{
    void Nav2CustomPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        if (!node_) {
            throw std::runtime_error("Failed to lock lifecycle node");
        }
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", resolution_);
        RCLCPP_INFO(node_->get_logger(), "Configured %s", name_.c_str());
    }

    void Nav2CustomPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up %s", name_.c_str());
    }

    void Nav2CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating %s", name_.c_str());
    }

    void Nav2CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating %s", name_.c_str());
    }

    nav_msgs::msg::Path Nav2CustomPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker)
    {
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
            RCLCPP_ERROR(node_->get_logger(), "Start or goal frame_id does not match global frame_id");
            return global_path;
        }

        int total_number_of_loop = 
            std::hypot(goal.pose.position.x - start.pose.position.x, goal.pose.position.y - start.pose.position.y) / resolution_;
        double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        for (int i = 0; i < total_number_of_loop; ++i) {
            if (cancel_checker()) {
                RCLCPP_WARN(node_->get_logger(), "Plan was canceled");
                return global_path;
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = start.pose.position.x + i * x_increment;
            pose.pose.position.y = start.pose.position.y + i * y_increment;
            pose.pose.position.z = 0.0;
            global_path.poses.push_back(pose);
        }

        for (geometry_msgs::msg::PoseStamped & pose : global_path.poses) {
            if (cancel_checker()) {
                RCLCPP_WARN(node_->get_logger(), "Plan was canceled");
                return global_path;
            }
            unsigned int mx, my;
            if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
                unsigned char cost = costmap_->getCost(mx, my);
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
                    RCLCPP_WARN(node_->get_logger(), "Path is in collision at (%f, %f)", pose.pose.position.x, pose.pose.position.y);
                    RCLCPP_ERROR(node_->get_logger(), "Planner failed to find a path.");
                }
            }
        }

        geometry_msgs::msg::PoseStamped final_pose = goal;
        final_pose.header.stamp = node_->now();
        final_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(final_pose);
        return global_path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::Nav2CustomPlanner, nav2_core::GlobalPlanner)