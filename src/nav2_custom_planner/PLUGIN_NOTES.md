# nav2_custom_planner — Reference Notes

How this plugin works, how Nav2 calls into it, and the plugin-registration pitfall
that broke bringup. File/line references are for `src/nav2_custom_planner.cpp` and
ROS 2 Jazzy headers under `/opt/ros/jazzy/include`.

## 1. Why a straight-line planner still needs `interpolation_resolution`

Nav2 does not consume a path as "start point + goal point" — a `nav_msgs::msg::Path`
must be a **dense sequence of poses**, because everything downstream works pose-by-pose:

- The controller (MPPI) tracks the path by finding the nearest path pose to the robot
  and scoring candidate trajectories against the upcoming poses. With only 2 poses
  5 m apart, the path-following critics have nothing to grip onto.
- The smoother, goal checker, and collision checking (including this plugin's own
  loop at `nav2_custom_planner.cpp:77-90`) all iterate over individual poses.

So `resolution_` is simply the **spacing between consecutive waypoints along the
line, in meters**. With the default `0.1`, a pose is dropped every 10 cm:

- `total_number_of_loop` (line 58) = straight-line distance ÷ resolution → number of segments.
- `x_increment` / `y_increment` = how much x and y advance per step so that after
  all steps you land at the goal.
- The loop walks from start toward goal in those steps, pushing one pose per step;
  line 95 appends the exact goal as the final pose.

Smaller resolution = denser path = smoother tracking and finer collision checks,
but more poses to process. 0.1 m is a sensible match for the costmap resolution.

## 2. Where `cancel_checker` comes from

It is a `std::function<bool()>` — a callable, not data — **required by the base
class interface** this plugin implements. See
`/opt/ros/jazzy/include/nav2_core/global_planner.hpp:70-81`:

```cpp
/**
 * @param cancel_checker Function to check if the action has been canceled
 */
virtual nav_msgs::msg::Path createPlan(..., std::function<bool()> cancel_checker) = 0;
```

The **caller supplies it**: when `planner_server` invokes the plugin, it passes a
lambda that asks its ComputePathToPose action server "has the client (the behavior
tree) requested cancellation?" — essentially
`[this]() { return action_server_->is_cancel_requested(); }` in Nav2's
`planner_server.cpp`. The plugin never defines it; it just calls it periodically
inside long loops (lines 64, 78) so a slow planning job can abort early instead of
blocking the server. For a trivial linear planner it will realistically never fire,
but real planners (Smac, Theta*) can search for seconds, and this hook is how Nav2
keeps them interruptible.

## 3. `worldToMap`, `mx`, `my`

The costmap is a 2D grid of cells; each cell holds a cost byte. Two coordinate
systems are in play:

- **World coordinates** (`wx`, `wy`): meters in the global frame (`map`), where poses live.
- **Map (grid) coordinates** (`mx`, `my`): integer **cell indices** — column and
  row in the grid array.

`worldToMap`
(`/opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d.hpp:183`)
converts the former into the latter: roughly `mx = (wx - origin_x) / cell_resolution`.
`mx`/`my` are output parameters (passed by reference), and the boolean return tells
you whether the point is actually inside the grid — that is why line 83 wraps it in
an `if`; a pose outside the map bounds cannot be cost-checked.

Once you have cell indices, `getCost(mx, my)` reads that cell's cost.
`LETHAL_OBSTACLE` (254) means the cell contains an obstacle, so the check at
lines 83-89 asks "does my straight line pass through a wall?"

**Known limitation:** the plugin only *logs* a warning/error on collision but still
returns the full path (line 96). A production planner would return an empty path or
throw so the planner server reports failure — as written, Nav2 will send the robot
along a line through a wall and let MPPI's obstacle critic fight it out locally.

## 4. How the LifecycleNode ends up calling the plugin

The plugin is not a node — it is a class loaded *into* the `planner_server` node
(a `nav2_util::LifecycleNode`).

**At startup (lifecycle transitions):**

1. `lifecycle_manager_navigation` sends the `configure` transition to `planner_server`.
2. In its `on_configure()`, planner_server reads `planner_plugins: ["GridBased"]`
   and `GridBased.plugin: "nav2_custom_planner::Nav2CustomPlanner"` from the params
   file, and uses `pluginlib::ClassLoader<nav2_core::GlobalPlanner>` to look that
   name up in the registered plugin XMLs and instantiate the class (this is the
   step that fails on a name mismatch — see §5).
3. It then calls the plugin's `configure(parent, name, tf, costmap_ros)`, handing it
   a weak pointer to *itself* (the `LifecycleNode` locked into `node_`), the name
   `"GridBased"` (which is why the parameter is looked up as
   `GridBased.interpolation_resolution`), the TF buffer, and the global costmap.
4. On the `activate` transition, `on_activate()` calls the plugin's `activate()`;
   same pattern for `deactivate()`/`cleanup()`. That is why those methods exist
   even though they only log here.

**At runtime (per navigation goal):**

1. RViz "Nav2 Goal" → `bt_navigator`'s behavior tree hits its `ComputePathToPose` BT node.
2. That sends a `ComputePathToPose` **action goal** to planner_server.
3. Planner_server's action callback looks up the current robot pose, selects the
   plugin by planner id (`"GridBased"`), builds the `cancel_checker` lambda, and
   calls `createPlan(start, goal, cancel_checker)`.
4. The returned `Path` goes back through the action result to the BT, which forwards
   it to `controller_server` (MPPI) via `FollowPath` — where the dense pose spacing
   from §1 gets used.

So: pluginlib gets the code into the process, the lifecycle transitions manage its
setup/teardown, and the action interface triggers the actual planning calls.

Then rebuild so the installed copy of the XML is refreshed:

```bash
colcon build --packages-select nav2_custom_planner
```

(The workspace is not built with `--symlink-install`, so editing the XML in `src/`
alone does nothing until rebuilt.)
