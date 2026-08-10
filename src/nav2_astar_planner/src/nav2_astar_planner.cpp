#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav2_astar_planner/nav2_astar_planner.hpp"

namespace nav2_astar_planner
{
    namespace
    {
        // One record per costmap cell, equivalent to NodeBase in the reference:
        // G = cost from start, H = heuristic to goal, F = G + H, connection = parent.
        struct AstarNode
        {
            int g = std::numeric_limits<int>::max();
            int h = 0;
            int parent = -1;
            bool closed = false; // the "processed" list
        };

        // Octile distance scaled by 10 (straight) / 14 (diagonal), as in the reference.
        int gridDistance(int ax, int ay, int bx, int by)
        {
            int dx = std::abs(ax - bx);
            int dy = std::abs(ay - by);
            // move diagonally as long as you can make progress, then move straight to the goal
            return 14 * std::min(dx, dy) + 10 * std::abs(dx - dy);
        }

        bool isWalkable(unsigned char cost)
        {
            return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        }
    }

    void Nav2AstarPlanner::configure(
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
        node_->get_parameter(name_ + ".interpolation_resolution", resolution_);
        RCLCPP_INFO(node_->get_logger(), "Configured %s", name_.c_str());
    }

    void Nav2AstarPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up %s", name_.c_str());
    }

    void Nav2AstarPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating %s", name_.c_str());
    }

    void Nav2AstarPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating %s", name_.c_str());
    }

    nav_msgs::msg::Path Nav2AstarPlanner::createPlan(
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

        unsigned int start_mx, start_my, goal_mx, goal_my;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my))
        {
            RCLCPP_ERROR(node_->get_logger(), "Start is outside the costmap");
            return global_path;
        }
        if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal is outside the costmap");
            return global_path;
        }
        if (!isWalkable(costmap_->getCost(goal_mx, goal_my)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal is in an obstacle");
            return global_path;
        }

        const int size_x = costmap_->getSizeInCellsX();
        const int size_y = costmap_->getSizeInCellsY();
        std::vector<AstarNode> nodes(static_cast<size_t>(size_x) * size_y);
        auto index = [size_x](int x, int y)
        { return y * size_x + x; };

        const int start_idx = index(start_mx, start_my);
        const int goal_idx = index(goal_mx, goal_my);

        // Min-heap ordered by F, ties broken by lower H — same selection rule the
        // reference applies with its linear scan over "toSearch".
        using OpenEntry = std::tuple<int, int, int>; // (f, h, index)
        std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> to_search;

        nodes[start_idx].g = 0;
        nodes[start_idx].h = gridDistance(start_mx, start_my, goal_mx, goal_my);
        to_search.emplace(nodes[start_idx].h, nodes[start_idx].h, start_idx);

        static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        bool found = false;
        int iterations = 0;
        while (!to_search.empty())
        {
            // cancel_checker() is polled every 1024 expansions
            if (++iterations % 1024 == 0 && cancel_checker())
            {
                RCLCPP_WARN(node_->get_logger(), "Plan was canceled");
                return global_path;
            }

            const auto [f, h, current] = to_search.top();
            to_search.pop();
            AstarNode &current_node = nodes[current];
            if (current_node.closed || f > current_node.g + current_node.h)
            {
                continue; // stale entry superseded by a cheaper path
            }
            current_node.closed = true;

            if (current == goal_idx)
            {
                found = true;
                break;
            }

            const int cx = current % size_x;
            const int cy = current / size_x;
            for (int i = 0; i < 8; ++i)
            {
                const int nx = cx + dx[i];
                const int ny = cy + dy[i];
                if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y)
                {
                    continue;
                }
                const unsigned char cell_cost = costmap_->getCost(nx, ny);
                if (!isWalkable(cell_cost))
                {
                    continue;
                }
                AstarNode &neighbor = nodes[index(nx, ny)];
                if (neighbor.closed)
                {
                    continue;
                }
                // Travel cost plus the costmap cell cost so the path prefers to
                // stay away from inflated obstacle regions.
                // cell_cost: 0, Free space. 1-252 Inflation zone.
                // 253, Inscribed inflated obstacle. 254, Lethal obstacle. 255, Unknown.
                const int cost_to_neighbor = current_node.g + gridDistance(cx, cy, nx, ny) + cell_cost;
                if (cost_to_neighbor < neighbor.g)
                {
                    neighbor.g = cost_to_neighbor;
                    neighbor.parent = current;
                    neighbor.h = gridDistance(nx, ny, goal_mx, goal_my);
                    to_search.emplace(neighbor.g + neighbor.h, neighbor.h, index(nx, ny));
                }
            }
        }

        if (!found)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planner failed to find a path.");
            return global_path;
        }

        // Backtrack from the goal through the parent connections, then reverse.
        std::vector<int> cell_path;
        for (int idx = goal_idx; idx != -1; idx = nodes[idx].parent)
        {
            cell_path.push_back(idx);
        }
        std::reverse(cell_path.begin(), cell_path.end());

        for (int idx : cell_path)
        {
            double wx, wy;
            costmap_->mapToWorld(idx % size_x, idx / size_x, wx, wy);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            global_path.poses.push_back(pose);
        }

        geometry_msgs::msg::PoseStamped final_pose = goal;
        final_pose.header.stamp = node_->now();
        final_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(final_pose);
        RCLCPP_INFO(node_->get_logger(), "A* found a path with %zu poses (%d cells expanded)",
                    global_path.poses.size(), iterations);
        return global_path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::Nav2AstarPlanner, nav2_core::GlobalPlanner)