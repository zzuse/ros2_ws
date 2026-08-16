#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav2_rrtstar_planner/nav2_rrtstar_planner.hpp"

namespace nav2_rrtstar_planner
{
    namespace
    {
        // One vertex of the RRT* tree, stored in world coordinates.
        struct TreeNode
        {
            double x = 0.0;
            double y = 0.0;
            int parent = -1;
            double cost = 0.0; // cost from the root along tree edges
        };

        double euclidean(double ax, double ay, double bx, double by)
        {
            return std::hypot(bx - ax, by - ay);
        }

        bool isWalkable(unsigned char cost)
        {
            return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        }
    }

    void Nav2RRTstarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        if (!node_)
        {
            throw std::runtime_error("Failed to lock lifecycle node");
        }
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_iterations", rclcpp::ParameterValue(5000));
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".step_size", rclcpp::ParameterValue(0.3));
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".neighbor_radius", rclcpp::ParameterValue(0.6));
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.25));
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".goal_bias", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", resolution_);
        node_->get_parameter(name_ + ".max_iterations", max_iterations_);
        node_->get_parameter(name_ + ".step_size", step_size_);
        node_->get_parameter(name_ + ".neighbor_radius", neighbor_radius_);
        node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
        node_->get_parameter(name_ + ".goal_bias", goal_bias_);
        rng_.seed(std::random_device{}());
        RCLCPP_INFO(node_->get_logger(), "Configured %s", name_.c_str());
    }

    void Nav2RRTstarPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up %s", name_.c_str());
    }

    void Nav2RRTstarPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating %s", name_.c_str());
    }

    void Nav2RRTstarPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating %s", name_.c_str());
    }

    nav_msgs::msg::Path Nav2RRTstarPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> cancel_checker)
    {
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Start or goal frame_id does not match global frame_id");
            return global_path;
        }

        unsigned int mx, my;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my))
        {
            RCLCPP_ERROR(node_->get_logger(), "Start is outside the costmap");
            return global_path;
        }
        if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my))
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal is outside the costmap");
            return global_path;
        }
        if (!isWalkable(costmap_->getCost(mx, my)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal is in an obstacle");
            return global_path;
        }

        // A world-coordinate point is free if it maps into the costmap and its
        // cell is below the inscribed-obstacle threshold.
        auto isFree = [this](double wx, double wy)
        {
            unsigned int cx, cy;
            if (!costmap_->worldToMap(wx, wy, cx, cy))
            {
                return false;
            }
            return isWalkable(costmap_->getCost(cx, cy));
        };

        // Walk the segment at costmap resolution and reject it if any sample
        // touches a lethal or inscribed cell.
        const double check_step = costmap_->getResolution();
        auto isSegmentFree = [&isFree, check_step](double ax, double ay, double bx, double by)
        {
            const double dist = euclidean(ax, ay, bx, by);
            const int steps = std::max(1, static_cast<int>(std::ceil(dist / check_step)));
            for (int i = 0; i <= steps; ++i)
            {
                const double t = static_cast<double>(i) / steps;
                if (!isFree(ax + t * (bx - ax), ay + t * (by - ay)))
                {
                    return false;
                }
            }
            return true;
        };

        const double goal_x = goal.pose.position.x;
        const double goal_y = goal.pose.position.y;
        const double origin_x = costmap_->getOriginX();
        const double origin_y = costmap_->getOriginY();
        // The distribution is from left edge of the costmap to right edge, and from bottom edge to top edge.
        std::uniform_real_distribution<double> sample_x(origin_x, origin_x + costmap_->getSizeInMetersX());
        std::uniform_real_distribution<double> sample_y(origin_y, origin_y + costmap_->getSizeInMetersY());
        std::uniform_real_distribution<double> sample_bias(0.0, 1.0); // the value is between 0 and 1, and if it is less than goal_bias_, we sample the goal instead of a random point

        std::vector<TreeNode> tree;
        tree.reserve(max_iterations_ + 1);
        tree.push_back({start.pose.position.x, start.pose.position.y, -1, 0.0});

        // Best tree node that connects to the goal so far, and the total path
        // cost through it; later samples keep improving it (the "star" part).
        int best_goal_parent = -1;
        double best_goal_cost = std::numeric_limits<double>::infinity();

        int iterations = 0;
        for (; iterations < max_iterations_; ++iterations)
        {
            if (iterations % 256 == 0 && cancel_checker())
            {
                RCLCPP_WARN(node_->get_logger(), "Plan was canceled");
                return global_path;
            }

            // suppose goal_bias_ = 0.1, then 10% of the time we sample the goal, and 90% of the time we sample a random point in the map.
            // ~90% of iterations: the magnet is placed randomly → the tree spreads out and explores the whole map.
            // ~10% of iterations: the magnet is placed on the goal → the tree's closest frontier gets tugged one step goal-ward.
            // Without those occasional goal-pulls, the tree only reaches the goal by luck. Without the random ones, the tree just rams straight at the goal and gets stuck behind the first obstacle.
            // The < goal_bias_ comparison is simply a coin flip weighted 10/90 between "pull" and "explore."
            double rx, ry;
            if (sample_bias(rng_) < goal_bias_)
            {
                rx = goal_x;
                ry = goal_y;
            }
            else
            {
                rx = sample_x(rng_);
                ry = sample_y(rng_);
            }

            // Searches through all existing nodes in the tree (tree.size()) to find which node is closest to the newly sampled point.
            int nearest = 0;
            double nearest_dist = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < tree.size(); ++i)
            {
                const double d = euclidean(tree[i].x, tree[i].y, rx, ry);
                if (d < nearest_dist)
                {
                    nearest_dist = d;
                    nearest = static_cast<int>(i);
                }
            }

            // Steer from the nearest node toward the sample by at most step_size.
            double new_x = rx;
            double new_y = ry;
            if (nearest_dist > step_size_)
            {
                // √(Δx² + Δy²) · step_size_ / nearest_dist  =  nearest_dist · step_size_ / nearest_dist  =  step_size_
                new_x = tree[nearest].x + (rx - tree[nearest].x) * step_size_ / nearest_dist;
                new_y = tree[nearest].y + (ry - tree[nearest].y) * step_size_ / nearest_dist;
            }
            if (!isFree(new_x, new_y) || !isSegmentFree(tree[nearest].x, tree[nearest].y, new_x, new_y))
            {
                continue;
            }

            // Choose parent: among neighbors within neighbor_radius, pick the one
            // giving the lowest cost from the root through a collision-free edge.
            std::vector<int> neighbors;
            for (size_t i = 0; i < tree.size(); ++i)
            {
                if (euclidean(tree[i].x, tree[i].y, new_x, new_y) <= neighbor_radius_)
                {
                    neighbors.push_back(static_cast<int>(i));
                }
            }
            int parent = nearest;
            double new_cost = tree[nearest].cost + euclidean(tree[nearest].x, tree[nearest].y, new_x, new_y);
            for (int i : neighbors)
            {
                const double candidate = tree[i].cost + euclidean(tree[i].x, tree[i].y, new_x, new_y);
                if (candidate < new_cost && isSegmentFree(tree[i].x, tree[i].y, new_x, new_y))
                {
                    parent = i;
                    new_cost = candidate;
                }
            }

            const int new_idx = static_cast<int>(tree.size());
            tree.push_back({new_x, new_y, parent, new_cost});

            // Rewire: reroute neighbors through the new node when that is cheaper.
            for (int i : neighbors)
            {
                const double rerouted = new_cost + euclidean(new_x, new_y, tree[i].x, tree[i].y);
                if (rerouted < tree[i].cost && isSegmentFree(new_x, new_y, tree[i].x, tree[i].y))
                {
                    tree[i].parent = new_idx;
                    tree[i].cost = rerouted;
                }
            }

            // Try to connect the new node to the goal.
            const double goal_dist = euclidean(new_x, new_y, goal_x, goal_y);
            if (goal_dist <= goal_tolerance_ && new_cost + goal_dist < best_goal_cost && isSegmentFree(new_x, new_y, goal_x, goal_y))
            {
                best_goal_parent = new_idx;
                best_goal_cost = new_cost + goal_dist;
            }
        }

        if (best_goal_parent < 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planner failed to find a path.");
            return global_path;
        }

        // Backtrack from the goal connection through the parents, then reverse.
        // Rewiring may have updated parents/costs after the goal was reached, so
        // the chain reflects the best tree found over all iterations.
        std::vector<std::pair<double, double>> waypoints;
        waypoints.emplace_back(goal_x, goal_y);
        for (int idx = best_goal_parent; idx != -1; idx = tree[idx].parent)
        {
            waypoints.emplace_back(tree[idx].x, tree[idx].y);
        }
        std::reverse(waypoints.begin(), waypoints.end());

        // Interpolate along the tree edges so the controller gets a dense path.
        auto appendPose = [this, &global_path](double wx, double wy)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            global_path.poses.push_back(pose);
        };
        for (size_t i = 0; i + 1 < waypoints.size(); ++i)
        {
            const auto [ax, ay] = waypoints[i];
            const auto [bx, by] = waypoints[i + 1];
            // divide by resolution = "one interpolated pose per costmap cell of distance,"
            // std::max(1, …) — guarantee at least one step even for a nearly zero-length edge,
            // so the loop below always emits the segment's start point and never divides by zero at below (s / steps).
            const int steps = std::max(1, static_cast<int>(std::ceil(euclidean(ax, ay, bx, by) / resolution_)));
            for (int s = 0; s < steps; ++s)
            {
                const double t = static_cast<double>(s) / steps;
                appendPose(ax + t * (bx - ax), ay + t * (by - ay));
            }
        }

        geometry_msgs::msg::PoseStamped final_pose = goal;
        final_pose.header.stamp = node_->now();
        final_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(final_pose);
        RCLCPP_INFO(node_->get_logger(), "RRT* found a path with %zu poses (cost %.2f m, %zu tree nodes, %d iterations)",
                    global_path.poses.size(), best_goal_cost, tree.size(), iterations);
        return global_path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtstar_planner::Nav2RRTstarPlanner, nav2_core::GlobalPlanner)
