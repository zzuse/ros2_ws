#ifndef NAV2_RRTSTAR_PLANNER_HPP
#define NAV2_RRTSTAR_PLANNER_HPP

#include <memory>
#include <random>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_rrtstar_planner
{
    class Nav2RRTstarPlanner : public nav2_core::GlobalPlanner
    {
    public:
        Nav2RRTstarPlanner() = default;
        ~Nav2RRTstarPlanner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            std::function<bool()> cancel_checker) override;

    private:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::string global_frame_, name_;
        double resolution_;
        int max_iterations_;
        double step_size_;
        double neighbor_radius_;
        double goal_tolerance_;
        double goal_bias_;
        std::mt19937 rng_;
    };
}
#endif // NAV2_RRTSTAR_PLANNER_HPP