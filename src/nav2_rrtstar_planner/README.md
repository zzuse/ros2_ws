RRT* (Rapidly-exploring Random Tree Star)
1. Sample a random point inside the costmap bounds, with a goal_bias chance (10%) of picking the goal itself.
2. Steer from the nearest tree node toward the sample, capped at step_size (0.3 m), and reject the edge if any point along it (checked at costmap resolution) touches an inscribed/lethal cell.
3. Choose parent — among all tree nodes within neighbor_radius (0.6 m), the new node attaches to whichever gives the lowest cost-from-start through a collision-free edge, not just the nearest.
4. Rewire — neighbors whose path would get cheaper by routing through the new node are re-parented to it. Steps 3–4 are what make it RRT* instead of plain RRT.
5. Goal connection — any new node within goal_tolerance (0.25 m) of the goal with a clear segment to it becomes a candidate finish; the search runs all max_iterations (5000) and keeps the cheapest connection, so the path keeps improving instead of stopping at the first hit.

The final path is backtracked through the parent chain and interpolated at interpolation_resolution so the controller gets a dense pose sequence, same as before.

All five knobs are ROS parameters under the planner's name (e.g. GridBased.step_size), so you can tune them in nav2_params.yaml without rebuilding. Two things worth knowing for your setup: