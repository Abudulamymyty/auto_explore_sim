# Nav2 Tuning Guide for G1 Robot

This guide documents the Nav2 tuning process for the G1 robot, focusing on reliable performance in tight office environments (0.8m doorways) with the MPPI controller. It addresses common issues encountered and the specific parameter changes made to resolve them.

---

## 🚀 Key Problems & Solutions

### 1. Robot Getting Stuck Near Obstacles
**Symptom:** The robot freezes when close to a wall, refuses to move, or recovery behaviors (spin/backup) abort immediately.
**Root Cause:**
*   **Invisible Walls:** Local costmap inflation was too small (`inflation_radius` < `robot_radius`), so the controller saw zero cost until it hit a "lethal" wall, causing it to fall off a "cliff" into a stuck state.
*   **Over-cautious Safety:** Collision monitor froze the robot completely when near walls.
*   **Pessimistic Recovery:** Backup/spin behaviors looked too far ahead (`2.0s`), predicting collisions in tight spaces and refusing to run.

**Solution:**
*   **Gradient Visibility:** Increased local costmap `inflation_radius` to **0.40m** (from 0.20m) and lowered `cost_scaling_factor` to **5.0**. This creates a smooth cost gradient so MPPI "feels" the wall approaching and steers away *before* getting stuck.
*   **Two-Zone Safety:** Split collision monitor into `FootprintApproach` (slow down at 0.5s) and defined a `PolygonStop` (emergency stop at 0.25s). *Note: Currently `PolygonStop` is commented out to test smoother approach/flow.*
*   **Aggressive Recovery:** Reduced `simulate_ahead_time` to **0.8s** (from 2.0s) and lowered `min_rotational_vel` to **0.2 rad/s**. This allows the robot to make small adjustments in tight spaces without panicking.

### 2. Planner Failures ("Failed to create a plan from potential")
**Symptom:** Robot aborts immediately with planner errors, especially when start or goal is near an obstacle.
**Root Cause:**
*   **NavFn Limitations:** The default NavFn planner relies on gradient descent, which fails over flat or sharp costmap gradients.
*   **Start/Goal Occupied:** Global costmap `robot_radius` (0.22m) made any start/goal within 22cm of a wall "lethal" and invalid.

**Solution:**
*   **SmacPlanner2D:** Switched to `nav2_smac_planner::SmacPlanner2D`. It uses A* graph search, which is robust and guaranteed to find a path if one exists.
*   **Permissive Planning:** Reduced global costmap `robot_radius` to **0.15m**. This shrinks the "lethal" zone around walls in the *planning* map only, allowing plans to start/end closer to walls while relying on the local costmap/collision monitor for actual safety.
*   **Flexible Goals:** Increased SmacPlanner `tolerance` to **1.0m** so it snaps to the nearest valid point if the goal is clicked inside a wall.

### 3. Path Oscillation & erratic Behavior
**Symptom:** Robot constantly switches between two similar paths, or looks indecisive.
**Root Cause:**
*   MPPI was too "loose," frequently re-evaluating and switching to slightly lower-cost trajectories.
*   Low path-following weights meant the robot didn't commit strongly to the global plan.

**Solution:**
*   **High Commitment:** Drastically increased `PathAlignCritic.cost_weight` (14.0 → **22.0**) and `PathFollowCritic.cost_weight` (5.0 → **12.0**).
*   **Lookahead:** Increased `PathFollowCritic.offset_from_furthest` to **20** points (~1m), acting as an anchor to pull the robot along the path.
*   **Path Inertia:** Increased `prune_distance` to **2.5m** to keep more of the previous path history.

### 4. Poor Reversing Behavior
**Symptom:** Robot tries to back up in tight spots and gets stuck or hits things.
**Root Cause:**
*   MPPI allowed reversing (`vx_min: -0.5`).
*   In tight spaces, backing up is dangerous given the limited rear visibility/sensor coverage.

**Solution:**
*   **Rotate-and-Drive:** Set `vx_min` to **-0.10** (virtually no reverse) and boosted `PreferForwardCritic` weight to **15.0**. This forces the robot to rotate in place to face the goal and drive forward, which is safer and more reliable.

---

## 🛠️ Tuning Guide for Future Development

### Determining if you need to re-tune:
1.  **Environment Change:** If moving to a warehouse (wide open) or cluttered home (very tight), adjust `inflation_radius`.
2.  **Robot Change:** If `robot_radius` or dynamics (speed/accel) change, update `max_velocity`, `robot_radius`, and MPPI `ax_max`/`wz_max`.

### Tuning Approach:

**Step 1: The Gradient (Local Costmap)**
*   Ensure `inflation_radius` > `robot_radius`.
*   Visualise `local_costmap` in RViz. You should see a grey gradient extending from obstacles. If it's a sharp black-to-white cliff, MPPI will crash.
*   *Tweak:* `cost_scaling_factor`. Lower = wider gradient (safer, stays further from walls). Higher = tighter gradient (can squeeze through smaller gaps).

**Step 2: The Controller (MPPI)**
*   **Distance to walls:** Adjust `CostCritic.cost_weight`. Higher = stays further away.
*   **Path Tracking:** if robot cuts corners too much, increase `PathAlignCritic` and `PathFollowCritic`.
*   **Smoothness:** If robot jitters, increase `PathAlignCritic` or `wz_std` (noise).

**Step 3: The Planner (Global Costmap)**
*   If plans fail in narrow deviations, lower global `robot_radius` slightly (e.g. 0.15m for a 0.22m robot).
*   Always use `SmacPlanner2D` for 2D indoor navigation.

**Step 4: Safety (Collision Monitor)**
*   Don't rely on the costmap for emergency stops. Use the collision monitor on raw scan data.
*   Use a "slow down" zone (`approach`) that is larger than the "hard stop" zone.

## 📄 Key Parameters Reference (Current Best Profile)

| Component | Parameter | Value | Reason |
| :--- | :--- | :--- | :--- |
| **Planner** | Plugin | `SmacPlanner2D` | Reliable A*, no gradient descent bugs. |
| **Global Costmap** | `robot_radius` | `0.15` | **Permissive:** Allows planning near walls. |
| **Local Costmap** | `inflation_radius` | `0.40` | **Gradient:** Lets MPPI "see" walls coming. |
| **Local Costmap** | `cost_scaling_factor` | `5.0` | **Gradient:** Smooth decay for better steering. |
| **MPPI** | `vx_min` | `-0.10` | Disables reversing, forces rotation. |
| **MPPI** | `PreferForwardCritic` | `15.0` | Enforces forward-only motion. |
| **MPPI** | `PathAlignCritic` | `22.0` | Strong commitment to global path. |
| **Recovery** | `simulate_ahead_time` | `0.8` | Allows spinning/backup in tight corridors. |
