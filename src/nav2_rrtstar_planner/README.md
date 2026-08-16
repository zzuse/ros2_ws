RRT* (Rapidly-exploring Random Tree Star)
1. Sample a random point inside the costmap bounds, with a goal_bias chance (10%) of picking the goal itself.
2. Steer from the nearest tree node toward the sample, capped at step_size (0.15 m), and reject the edge if any point along it (checked at costmap resolution) touches an inscribed/lethal cell.
3. Choose parent — among all tree nodes within neighbor_radius (0.35 m), the new node attaches to whichever gives the lowest cost-from-start through a collision-free edge, not just the nearest.
4. Rewire — neighbors whose path would get cheaper by routing through the new node are re-parented to it. Steps 3–4 are what make it RRT* instead of plain RRT.
5. Goal connection — any new node within goal_tolerance (0.25 m) of the goal with a clear segment to it becomes a candidate finish; the search runs all max_iterations (10000) and keeps the cheapest connection, so the path keeps improving instead of stopping at the first hit.

6. The final path is backtracked through the parent chain and interpolated at interpolation_resolution so the controller gets a dense pose sequence, same as before.

Five RRT* parameters at their defaults so they're visible for tuning:

- max_iterations: 5000 — total samples per plan; lower this first if replans hog CPU on the 4-core server
- step_size: 0.3 — max edge length in meters
- neighbor_radius: 0.6 — rewiring radius (the "star" optimization)
- goal_tolerance: 0.25 — matches your goal checker's xy_goal_tolerance
- goal_bias: 0.1 — 10% of samples aim straight at the goal

One note: expected_planner_frequency is set to 20 Hz, and RRT* running all 5000 iterations per plan likely won't hit that — you'll see "planner loop missed its desired rate" warnings. That's harmless for testing (the BT replans at its own pace), but if it bothers you, drop expected_planner_frequency to something like 1.0 or reduce max_iterations.

Calibration
- robot_radius: 0.12 → 0.10 — the true half-width of your 20 cm cart. This shrinks the hard-blocked band along each wall from 12 cm to 10 cm, widening the legal channel from 16 cm to 20 cm.
- inflation_radius: 0.35 → 0.20, cost_scaling_factor: 2.5 → 5.0 — before, inflation from both walls overlapped across the whole hallway and the centerline sat at cost ≈ 207/254. Now cost decays to ~0 at the centerline, while the steeper gradient still pushes the planner and MPPI toward the middle of the corridor.
- step_size: 0.3 → 0.15, neighbor_radius: 0.6 → 0.35, max_iterations: 5000 → 10000 — a 0.3 m step almost never lands a collision-free segment inside a 20 cm channel; halving the step (with more iterations to compensate) lets the tree actually grow through the hallway. Your segment collision check already samples at costmap resolution, so no code change is needed.

Two caveats to keep in mind:

1. Robot length and rotation. robot_radius: 0.10 only covers the cart's width. If the cart is longer than 20 cm, it must not rotate in place inside the hallway — the corners would sweep past the 10 cm radius. Approach the hallway roughly aligned, and if Nav2's recovery spin behavior ever triggers mid-hallway it could clip a wall. If that becomes a problem, the fix is a polygon footprint instead of robot_radius plus consider_footprint: true in MPPI's CostCritic — but that costs CPU, and on your 4-core server MPPI is already tight, so I left the cheap point-check as is.
2. Map quality matters at these tolerances. With 5 cm map resolution and ±1 cell of wall smear, a 40 cm hallway can easily render as 35 cm on the map, eating your margin. If the robot still refuses the hallway after this change, check room.pgm around that corridor first — your notes say a map rebuild after the odometry calibration was still pending.