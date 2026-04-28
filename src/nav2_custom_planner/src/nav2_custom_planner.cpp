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
        nav_msgs::msg::Path path;
        return path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::Nav2CustomPlanner, nav2_core::GlobalPlanner)