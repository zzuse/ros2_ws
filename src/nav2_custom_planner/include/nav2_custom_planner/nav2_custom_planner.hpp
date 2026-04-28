#ifndef NAV2_CUSTOM_PLANNER_HPP
#define NAV2_CUSTOM_PLANNER_HPP

#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"


namespace nav2_custom_planner
{
    class Nav2CustomPlanner : public nav2_core::GlobalPlanner {
    public:
        Nav2CustomPlanner() = default;
        ~Nav2CustomPlanner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal,
            std::function<bool()> cancel_checker) override;
    private:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_costmap_2d::Costmap2D* costmap_;
        std::string global_frame_, name_;
        double resolution_;
    };
}
#endif // NAV2_CUSTOM_PLANNER_HPP