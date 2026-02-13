# GVD-Based Frontier Exploration Improvements

## Overview

This document records the improvements made to the frontier exploration system
by introducing a **Generalized Voronoi Diagram (GVD)** skeleton, fixing the
**"arrive-and-loop"** bug, and adding **GVD path-following navigation** so the
robot traverses corridor centres instead of cutting across open space.

---

## 1. GVD Voronoi Skeleton

### Problem

The original explorer selected frontier **centroids** as navigation goals.
Centroids can land in narrow passages, near walls, or in positions that are
difficult for Nav2 to reach, leading to frequent navigation failures and
suboptimal paths.

### Solution

Compute a true Voronoi skeleton (GVD) of the free space and snap frontier goals
onto it so the robot always navigates through corridor centres.

### Algorithm — Two-Phase GVD Computation

**Phase 1 — Connected-Component Labeling**

All occupied cells (`occupancy > 50`) are grouped into connected obstacle
regions via 8-connected BFS flood-fill. Each region receives a unique integer
label. Unknown cells (`-1`) are **excluded** — they represent the exploration
frontier, not permanent obstacles.

```
obstacle = map_array > 50          # real walls only
region_id[cell] = connected-component label (BFS 8-connected)
```

**Phase 2 — BFS Distance Transform with Region Labels**

A multi-source BFS starts from every obstacle cell, propagating both the
Manhattan distance and the region label outward through all cells (free and
unknown). Each cell learns:
- `dist[y, x]` — distance to nearest obstacle (in cells)
- `label[y, x]` — which obstacle region is nearest

**GVD Extraction**

A cell is a GVD point if:
1. It is a **free cell** (`occupancy == 0`) — not unknown, not obstacle
2. At least one 4-connected neighbor has a **different obstacle label**
   (equidistant from 2+ distinct walls)
3. Its distance to the nearest obstacle `>= gvd_min_clearance` (default 5
   cells) — filters out noisy skeleton branches too close to walls

The extraction is fully vectorised using NumPy shifted-view comparisons:

```python
for direction in (right, left, down, up):
    diff = (src_label != nbr_label) & (src >= 0) & (nbr >= 0)
    gvd_mask |= diff
gvd_mask &= (map_array == 0)        # free cells only
gvd_mask &= (dist_map >= min_c)     # clearance threshold
```

### RViz Visualization

The GVD skeleton is published as a `LINE_LIST` marker on `/explore/gvd`.
Adjacent GVD cells (8-connected) are connected as line segments, producing
continuous cyan lines overlaid on the map.

| Property | Value |
|----------|-------|
| Topic | `/explore/gvd` |
| Type | `visualization_msgs/MarkerArray` (LINE_LIST) |
| Color | Cyan (0, 1, 1, 0.8) |
| Line width | 0.02 m |
| Lifetime | 10 s |

### Frontier Goal Snapping

After selecting the best frontier, its centroid is snapped to the nearest GVD
point via BFS outward search (radius controlled by `gvd_snap_radius`, default
2.0 m). A directional filter ensures the snapped point lies **between the robot
and the frontier** — never behind the robot:

```
cand_to_robot < robot_to_frontier   # must be closer to frontier than robot is
```

If no valid GVD point is found, the original centroid is used as a fallback.

---

## 2. GVD Path-Following Navigation

### Problem

Even with frontier goals snapped to GVD points, the global planner
(SmacPlanner2D) still plans freely — the robot may cut through open areas or
hug walls instead of following the safe corridor skeleton.

### Solution

Compute an A\* path along the GVD skeleton from the robot to the frontier goal,
sample waypoints at regular intervals, and send them to Nav2 via
`NavigateThroughPoses` so the robot follows the skeleton corridor.

### A\* Pathfinding on GVD Skeleton

The A\* search operates on 8-connected cells with a **skeleton preference
penalty**:

| Cell type | Movement cost |
|-----------|---------------|
| GVD skeleton cell | 1.0 (straight) / √2 (diagonal) |
| Non-GVD free cell | 5.0 (straight) / 5√2 (diagonal) |
| Obstacle / unknown | impassable |

This allows the path to **bridge gaps** in the skeleton (common early in
exploration when the map is sparse) while strongly preferring skeleton cells.
The heuristic is Euclidean distance.

```python
OFF_GVD_PENALTY = 5.0
base_cost = SQRT2 if diagonal else 1.0
if not gvd_mask[ny, nx]:
    base_cost *= OFF_GVD_PENALTY
```

### Nearest GVD Cell Search

Before A\* runs, the robot and goal positions are each snapped to their nearest
GVD cell via BFS outward search (max 1.5 m). If either has no nearby GVD cell,
the system falls back to single-goal navigation.

### Waypoint Sampling

The raw A\* path has one point per map cell (typically 0.05 m apart). It is
down-sampled to waypoints spaced ~0.5 m apart to avoid overwhelming Nav2.
The first and last points are always included.

### NavigateThroughPoses Action

Waypoints are sent as an array of `PoseStamped` via the `NavigateThroughPoses`
action (already configured in `bt_navigator`). Nav2's `RemovePassedGoals` BT
node (radius 0.7 m) consumes waypoints as the robot passes them.

### Graceful Fallback

If no connected GVD path exists (skeleton gaps too large, robot or goal too far
from any GVD cell), the system falls back to the original `NavigateToPose` with
the snapped single goal. This ensures exploration never stalls.

### Path Visualization

The current navigation path is published as a magenta `LINE_STRIP` + sphere
markers on `/explore/gvd_path`.

| Property | Value |
|----------|-------|
| Topic | `/explore/gvd_path` |
| Type | `visualization_msgs/MarkerArray` (LINE_STRIP + SPHERE) |
| Color | Magenta (1, 0, 1, 1.0) |
| Line width | 0.05 m |
| Sphere size | 0.1 m |
| Lifetime | 30 s |

---

## 3. "Arrive-and-Loop" Bug Fix

### Problem

The robot would:
1. Select nearest frontier → navigate
2. Nav2 reports `SUCCEEDED` (due to `xy_goal_tolerance: 0.5m` and SmacPlanner
   `tolerance: 1.0m`) even though the robot stopped **before reaching** the
   frontier
3. The frontier still exists (not yet observed by SLAM)
4. `prev_goal` was set to `None` after success → same-goal detection bypassed
5. Replan → same frontier selected → **infinite loop**

### Root Cause Analysis

| Component | Setting | Effect |
|-----------|---------|--------|
| Goal checker | `xy_goal_tolerance: 0.5m` | Robot stops 0.5 m from goal |
| SmacPlanner2D | `tolerance: 1.0m` | Planner snaps goal up to 1.0 m to find free cell |
| Frontier explorer | Only blacklisted on ABORT/REJECT | SUCCESS never triggered blacklist |
| Frontier explorer | `prev_goal = None` after success | Same-goal detection bypassed |

Combined worst case: robot "succeeds" up to ~1.5 m from the original frontier
centroid, frontier persists, robot re-selects it indefinitely.

### Solution — Three-Layer Defense

**Layer 1: Blacklist frontier centroid on every navigation success**

```python
if status == SUCCEEDED:
    self._blacklist_point(self.current_frontier)
```

If the frontier was truly explored, it disappears from BFS naturally and the
blacklist entry is harmless. If Nav2 "succeeded" via tolerance without actually
reaching it, the blacklist prevents the loop. The blacklist expires after
`blacklist_timeout` (default 120 s).

**Layer 2: Minimum goal distance guard**

Before sending a goal, check if it's `< 0.3 m` from the robot. If so,
blacklist the frontier centroid immediately (it would trigger instant success).

```python
if dist_to_goal < 0.3:
    self._blacklist_point(best.centroid)
    return
```

**Layer 3: GVD snap directional filter**

The `_snap_to_gvd()` method rejects GVD candidates that are behind the robot
(`cand_to_robot >= robot_to_frontier`), preventing the goal from collapsing
onto the robot's current position.

---

## 4. Parameters

All parameters are in `config/frontier_explorer_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gvd_min_clearance` | 5 | Min distance-to-obstacle (cells) for valid GVD point |
| `gvd_snap_radius` | 2.0 | Max search radius (m) to find nearest GVD point |
| `visualize_gvd` | true | Publish GVD skeleton markers on `/explore/gvd` |
| `min_frontier_size` | 20 | Min cells in a frontier cluster to be valid |
| `blacklist_timeout` | 120.0 | Blacklist expiry time (s) |
| `blacklist_radius` | 0.5 | Radius around blacklisted point (m) |
| `clearance_scale` | 0.2 | GVD clearance tiebreaker weight in cost function |
| `progress_timeout` | 30.0 | Stuck detection timeout (s) |
| `planner_frequency` | 0.5 | How often the planner runs (Hz) |

---

## 5. Files Modified

| File | Changes |
|------|---------|
| `scripts/frontier_explorer.py` | GVD computation, skeleton viz, goal snapping, A\* path planning, NavigateThroughPoses, loop fix |
| `config/frontier_explorer_params.yaml` | Added `gvd_*` parameters, tuned `min_frontier_size` and `gvd_min_clearance` |
| `rviz/explore.rviz` | Added GVD Skeleton, GVD Path, and Exploration Goal displays |

### Key Methods

| Method | Purpose |
|--------|---------|
| `_compute_distance_and_labels()` | CC labeling + BFS distance transform with region labels |
| `_extract_gvd_mask()` | Vectorised GVD point extraction from label map |
| `_snap_to_gvd()` | BFS search for nearest valid GVD point with directional filter |
| `_nearest_gvd_cell()` | BFS to find nearest GVD cell to any world point |
| `_find_gvd_path()` | A\* pathfinding on free cells with GVD preference penalty |
| `_sample_waypoints()` | Down-sample dense path to ~0.5 m spaced waypoints |
| `_navigate_through_poses()` | Send waypoint array via NavigateThroughPoses action |
| `_publish_gvd_markers()` | LINE_LIST marker for RViz skeleton visualization |
| `_publish_path_markers()` | LINE_STRIP + SPHERE markers for current nav path |
| `_blacklist_point()` | Generic point blacklisting with timeout |

---

## 6. Architecture Diagram

```
 ┌──────────────────────────────────────────────────┐
 │  /map (OccupancyGrid from SLAM Toolbox)          │
 └─────────────────────┬────────────────────────────┘
                       │
                       ▼
 ┌─────────────────────────────────────────────────────┐
 │  _compute_distance_and_labels()                     │
 │  ├── Phase 1: Connected-component labeling (obs)    │
 │  └── Phase 2: BFS dist + region label propagation   │
 └─────────────────────┬───────────────────────────────┘
                       │
                       ▼
 ┌─────────────────────────────────────────────────────┐
 │  _extract_gvd_mask()                                │
 │  label boundary + free-cell filter + clearance      │
 └───────┬──────────────────┬──────────────────────────┘
         │                  │
         ▼                  ▼
 ┌───────────────┐   ┌──────────────────────────────────┐
 │  _publish_    │   │  _find_gvd_path()                │
 │  gvd_markers()│   │  A* from robot→goal on GVD mask  │
 │  → /explore/  │   │  (free cell traversal, 5x penalty│
 │    gvd        │   │   for off-skeleton cells)        │
 └───────────────┘   └───────────────┬──────────────────┘
                                     │
                          ┌──────────┴──────────┐
                          │ GVD path found?      │
                          ├── YES ───────────────┤── NO ─────────┐
                          ▼                                      ▼
                ┌──────────────────┐               ┌──────────────────┐
                │ _sample_waypoints│               │ _snap_to_gvd()   │
                │ (spacing=0.5m)   │               │ BFS + dir filter │
                └────────┬─────────┘               └────────┬─────────┘
                         │                                  │
                         ▼                                  ▼
              ┌────────────────────┐             ┌──────────────────┐
              │ NavigateThrough-   │             │ NavigateToPose   │
              │ Poses (waypoints)  │             │ (single goal)    │
              │ → /explore/gvd_path│             └──────────────────┘
              └────────────────────┘
```

---

## 7. RViz Topics Summary

| Topic | Type | Color | Description |
|-------|------|-------|-------------|
| `/explore/frontiers` | MarkerArray | Green/Red | Frontier centroids (green = best, red = others) |
| `/explore/gvd` | MarkerArray (LINE_LIST) | Cyan | GVD Voronoi skeleton |
| `/explore/gvd_path` | MarkerArray (LINE_STRIP + SPHERE) | Magenta | Current navigation waypoint path |
| `/explore/goal` | Marker (SPHERE) | Yellow | Current navigation goal |

---

## 8. Design Decisions & Rationale

### Why exclude unknown cells from GVD?

Unknown cells are the exploration frontier boundary. Including them as
"obstacles" creates false GVD lines along the frontier edge that shift every
time the map updates. The skeleton should only represent equidistance between
**permanent walls**, providing stable navigation corridors.

### Why connected-component labeling instead of per-cell labels?

Per-cell labeling (each obstacle pixel gets its own ID) creates false Voronoi
boundaries between adjacent pixels of the **same wall**. Connected-component
labeling ensures that all pixels belonging to one continuous wall share the same
label, so GVD lines only appear between **distinct** obstacle regions.

### Why A\* with penalty instead of strict GVD-only traversal?

The GVD skeleton is a 1-pixel-wide line. In practice, the skeleton has gaps —
especially early in exploration when the map is incomplete. Strict GVD-only A\*
frequently fails to find any path. Allowing traversal through any free cell
with a 5x cost penalty bridges these gaps while keeping 80-95% of the path on
the skeleton.

### Why NavigateThroughPoses instead of just snapping the final goal?

Snapping only the final goal to a GVD point still lets the global planner
(SmacPlanner2D) route freely. The robot may cut corners, hug walls, or take
paths through open areas that are not along the skeleton. Sending waypoints
along the skeleton forces the robot to follow corridor centres throughout the
entire trajectory.

### Why blacklist on every success instead of tightening Nav2 tolerances?

Tightening `xy_goal_tolerance` risks navigation failures in tight spaces where
the robot genuinely cannot reach the exact goal. Blacklisting on success is a
robust application-level fix that works regardless of Nav2 tolerance settings.
The blacklist has a timeout, so frontiers can be retried later if the map has
changed.

### Why directional filtering in `_snap_to_gvd()`?

Without it, the nearest GVD point might be **behind** the robot (between the
robot and its starting area), not between the robot and the frontier. This
causes the robot to navigate backwards to a point it already passed through,
triggering instant success and a loop.
