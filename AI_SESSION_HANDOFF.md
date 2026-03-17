# Auto Explore Sim: Session Handoff for the Next AI

## Purpose of This Document

This file is a direct handoff for another AI agent.

It explains:

- what the current `auto_explore_sim` explorer stack does,
- what was changed in this session,
- why those changes were made,
- what behavioral constraints the user explicitly wants,
- what is still imperfect,
- and what the next stage of work should focus on.

This is not a user manual. It is an engineering-state document.

---

## Scope of This Session

The work in this session focused on the C++ exploration stack inside `auto_explore_sim`, mainly:

- `src/frontier_explorer.cpp`
- `src/gvd_map.cpp`
- `src/frontier_search.cpp`
- `config/frontier_explorer_params.yaml`
- `config/nav2_params.yaml`

The broad goal was to turn the exploration behavior into:

- `frontier = direction`
- `GVD = actual railway / track`

The user repeatedly emphasized that the robot should prefer to travel on the GVD skeleton, not simply use frontiers as direct navigation goals.

---

## High-Level Behavior the User Wants

These are the most important behavioral requirements expressed by the user during this session.

1. A frontier is only a directional target.
   The frontier says where to go next, but the robot should travel using GVD points whenever possible.

2. The frontier should be snapped to the nearest usable GVD point.
   Frontier centroids often appear too close to obstacles, so they should not be used raw unless no GVD route is usable.

3. Once a frontier is locked, do not switch to another frontier too early.
   Even if the frontier disappears from the live frontier set during exploration, keep pursuing the same logical target and keep rebuilding the GVD toward it.

4. GVD is dynamic and must be updated during navigation.
   This matters especially when the frontier is far away and the map/GVD structure changes as SLAM grows.

5. The robot should avoid leaving the GVD track during normal frontier travel.
   This is a strong user preference.

6. If the robot gets stuck, it should recover toward a GVD point first, then replan.

7. If the robot has already passed the current GVD segment goal, it should not flip around just to touch that old waypoint.

8. Frontier retry / blacklist logic must not loop on unchanged maps.

9. Nav2 should not enforce heading unnecessarily at GVD waypoints or after reaching goals.

---

## Current Architecture

### 1. `frontier_explorer.cpp`

This is the orchestration/state-machine layer.

It is responsible for:

- subscriptions to map / odom / tf / resume,
- locking and tracking the current frontier,
- blacklisting,
- GVD-aware planning decisions,
- incremental dispatch of `NavigateThroughPoses`,
- recovery when stuck,
- visualization publishing,
- and runtime replanning.

Important locations:

- map-content hashing and cache control:
  - `src/frontier_explorer.cpp:70`
  - `src/frontier_explorer.cpp:81`
- GVD cache rebuild gate:
  - `src/frontier_explorer.cpp:462`
- frontier plan construction:
  - `src/frontier_explorer.cpp:492`
- dispatch of GVD waypoint plans:
  - `src/frontier_explorer.cpp:632`
- in-motion GVD refresh:
  - `src/frontier_explorer.cpp:670`
- map callback:
  - `src/frontier_explorer.cpp:755`
- main planning tick:
  - `src/frontier_explorer.cpp:964`
- failure / blacklist handling:
  - `src/frontier_explorer.cpp:1831`

### 2. `gvd_map.cpp`

This is the GVD construction and GVD path utility layer.

It is responsible for:

- building the Brushfire + Voronoi-style GVD from the occupancy grid,
- snapping a frontier/goal to a nearby GVD point,
- building same-component GVD paths,
- labeling connected GVD components,
- and providing compatibility helpers for component-entry or direct-safe planning.

Important locations:

- boundary-site seed detection:
  - `src/gvd_map.cpp:63`
- obstacle-boundary wavefront / distance propagation:
  - `src/gvd_map.cpp:391`
- GVD cell acceptance based on equal-distance and direction constraints:
  - `src/gvd_map.cpp:471`
- GVD connected-component labeling:
  - `src/gvd_map.cpp:556`
- snapping to GVD:
  - `src/gvd_map.cpp:596`
- same-component GVD path search:
  - `src/gvd_map.cpp:644`

### 3. `frontier_search.cpp`

This is the frontier extraction layer.

It is responsible for:

- identifying unknown cells adjacent to free cells,
- clustering frontier cells,
- computing centroid / size / distance metadata,
- filtering by a local update radius,
- and sorting candidates by heuristic distance.

Important locations:

- frontier cell definition:
  - `src/frontier_search.cpp:28`
- frontier cluster build:
  - `src/frontier_search.cpp:69`
- local frontier gating:
  - `src/frontier_search.cpp:133`
- heuristic nearest-first sorting:
  - `src/frontier_search.cpp:213`

---

## Current Functional Behavior

## GVD Construction

The current GVD is built as a discrete Voronoi-like skeleton over the occupancy grid.

Mechanism:

1. Obstacle boundary cells are used as Voronoi sites.
2. A wavefront propagates the nearest site label and distance across free space.
3. A free cell becomes a GVD cell only if:
   - it has enough clearance,
   - it has at least two competing nearest obstacle sites,
   - the site distances are close enough,
   - and the site directions are sufficiently different.
4. Accepted GVD points are projected toward the local Voronoi bisector for nicer world-space geometry.
5. Connected components of the GVD mask are labeled.

Why this was done:

- the user rejected earlier GVD masks that were too region-boundary-like or too discretized,
- the implementation was moved toward a stricter Voronoi interpretation,
- but still kept practical tolerances because exact equality is rare on a grid.

Key tuning knobs:

- `gvd_min_clearance`
- `gvd_distance_tolerance_cells`
- `gvd_max_site_direction_dot`
- `allow_gvd_bridge`

Current default direction:

- user wants a strict GVD “railway” feeling,
- so `allow_gvd_bridge` is currently `false` in config.

---

## Frontier Selection

Current frontier search is nearest-first by Euclidean heuristic distance.

Mechanism:

1. Build all frontier clusters.
2. Keep only frontiers within `frontier_update_radius`, unless local search finds none.
3. If local search is empty and no frontier is currently locked, fallback to full-map frontier search.
4. Sort candidates by:
   - heuristic distance,
   - then min distance,
   - then larger cluster size.

Why this was done:

- the user explicitly asked to use heuristic distance rather than Manhattan distance,
- and to select the nearest frontier and stop searching for the next one until the current one is finished or blacklisted.

Important caveat:

- the system still computes a full frontier set before choosing,
- but behaviorally it follows the user’s nearest-first policy.

---

## Locked Frontier Semantics

This is one of the most important current behaviors.

Once a frontier is selected:

- it becomes `current_frontier_`,
- the explorer tries to keep it,
- and it does not immediately switch away if that frontier disappears from the live frontier set.

Mechanism:

- `match_locked_frontier()` tries to match the locked target to a currently observed frontier.
- if it disappears, the system keeps using the stored frontier coordinates.
- planning continues toward the same logical frontier target.

Why this was done:

- the user explicitly said disappearing frontiers should not immediately cause switching,
- because that is inefficient and leads to wasted motion.

Current behavior:

- this locked frontier policy is preserved,
- but planning still depends on whether a usable GVD route can be built.

---

## GVD-Only Navigation Contract

Normal exploration is now intentionally strict:

- the dispatched frontier plan must be a `GvdWaypoints` plan,
- otherwise it is refused.

This is enforced in:

- `src/frontier_explorer.cpp:632`

Meaning:

- normal frontier travel uses `NavigateThroughPoses` on GVD waypoints,
- not a direct `NavigateToPose(frontier)` fallback.

Why this was done:

- the user repeatedly insisted that GVD is the “railway”,
- and that the robot should not leave the track during normal frontier motion.

Note:

- some compatibility code still exists in `gvd_map.cpp` for component-entry or direct-safe planning,
- but the current explorer dispatch path rejects non-GVD frontier plans.
- another AI should treat those helpers as legacy/compatibility scaffolding, not the current intended behavior.

---

## Segment-Based GVD Navigation

The robot does not always send the entire GVD path at once.

Mechanism:

1. Build a full GVD path to the snapped frontier anchor.
2. Convert it into waypoint candidates.
3. Optionally truncate that full waypoint sequence into a shorter segment.
4. Send that segment via `NavigateThroughPoses`.
5. On success, replan the next segment toward the same locked frontier.

Why this was done:

- long paths evolve as SLAM grows,
- GVD may change while moving,
- shorter segments reduce brittle long-horizon commitment.

There is also adaptive segment sizing:

- if GVD tracking is stable, segment length grows,
- if motion is jerky or progress is poor, segment length shrinks.

Relevant code:

- quality check:
  - `src/frontier_explorer.cpp:375`
- adaptive growth/shrink:
  - `src/frontier_explorer.cpp:405`
- segment dispatch:
  - `src/frontier_explorer.cpp:517`

---

## Continuous GVD Replanning During Navigation

The current navigator can refresh the GVD route while moving.

Triggers:

1. map content changes,
2. robot moves beyond `gvd_snap_radius` relative to the last planning pose,
3. or adaptive shortening forces a refresh.

Important detail:

- map updates are now content-based, not timestamp-based.

Why this matters:

- SLAM often republishes the same map with a new timestamp,
- that used to create false “map updated” events,
- which caused repeated replanning and retry loops.

Relevant code:

- refresh trigger and logic:
  - `src/frontier_explorer.cpp:670`

---

## Stuck Recovery

If no meaningful progress is made for `progress_timeout`, the system treats it as stuck.

Current recovery behavior:

1. Try to find a recovery point on the active GVD path.
2. If unavailable, try the nearest GVD point within `stuck_recovery_gvd_radius`.
3. Send a short `NavigateToPose` recovery goal to that GVD point.
4. After recovery succeeds, replan toward the same locked frontier.

Why this was done:

- the user wanted the robot to get back to GVD first, not improvise in free space.

Relevant code:

- `src/frontier_explorer.cpp:1534`
- `src/frontier_explorer.cpp:1548`

---

## Passed-Waypoint / “Do Not Flip Around” Fix

Late in this session, another issue was addressed:

Problem:

- after the user reduced Nav2 XY goal tolerance,
- the robot could physically move past the current segment’s last GVD waypoint,
- but Nav2/explorer still considered that old point the active goal,
- causing the robot to turn around and touch it.

Current fix:

- if the robot has already passed the final waypoint of the active GVD segment,
- and its lateral offset is still small enough,
- the current segment is canceled,
- and the explorer replans forward from the current pose instead of forcing a reversal.

Mechanism:

- project robot position onto the direction of the last path segment,
- check whether the robot is beyond the goal by `gvd_goal_pass_projection_margin`,
- require lateral error <= `gvd_goal_pass_lateral_tolerance`,
- if true, cancel current segment and immediately re-enter planning.

Relevant code:

- detection:
  - `src/frontier_explorer.cpp:463`
- cancellation + forward replan:
  - `src/frontier_explorer.cpp:488`
- feedback integration:
  - `src/frontier_explorer.cpp:901`

Why this was done:

- the user explicitly wanted to avoid inefficient flip-around behavior.

---

## Blacklist and Retry Logic

The blacklist logic was heavily modified in this session.

### Current intended meaning

Blacklisting is used for two different purposes:

1. mark a frontier as completed so it is not immediately reselected,
2. temporarily avoid a frontier that repeatedly fails planning/execution.

### Main problem that was fixed

There was a loop like this:

- frontiers failed with “no same-component GVD route”
- they got deferred / blacklisted
- system waited for a “new map update”
- SLAM republished the same map content with a different timestamp
- blacklist got cleared
- same frontiers were retried again
- loop repeated forever

### Current fix

Map change is now judged by content hash, not timestamp.

Also:

- frontier planning failure count only increases once per distinct map content,
- repeated failures on the same unchanged map do not keep incrementing the retry counter,
- blacklist reset after “all frontiers blacklisted” also waits for a real map-content change.

Relevant code:

- map hashing:
  - `src/frontier_explorer.cpp:81`
- all-blacklisted wait:
  - `src/frontier_explorer.cpp:1021`
- failure note structure:
  - `src/frontier_explorer.cpp:1838`
- same-map retry suppression:
  - `src/frontier_explorer.cpp:1868`

Why this was done:

- to stop false retry loops driven only by SLAM message timestamps.

---

## Nav2 Changes Made in This Session

This session also changed `config/nav2_params.yaml`.

### What changed

1. `FollowPath` now uses MPPI directly:
   - `nav2_mppi_controller::MPPIController`
   - not `RotationShimController`

2. Goal yaw is intentionally relaxed:
   - `yaw_goal_tolerance: 3.14`

3. Goal-angle and path-angle pressure were reduced:
   - `GoalAngleCritic.enabled: false`
   - `PathAngleCritic.enabled: false`
   - `PreferForwardCritic.cost_weight` reduced

### Why

The user explicitly said:

- do not request angle unnecessarily,
- do not make the robot rotate back and forth around goals,
- and do not discuss this as “Nav2 tracking deviation”; the concern was the requested navigation behavior itself.

Relevant config:

- `config/nav2_params.yaml:66`
- `config/nav2_params.yaml:75`
- `config/nav2_params.yaml:129`
- `config/nav2_params.yaml:180`

### Important remaining runtime issue

Even after controller cleanup, there is still an existing warning:

- inflation radius is too small relative to the robot footprint

This is not just cosmetic.
It can contribute to awkward controller behavior and poor obstacle handling.

The current logs still show:

- local/global costmap inflation-radius warnings
- occasional startup TF timing issues

These are not the main focus of this session, but they remain real.

---

## Current Parameters That Matter Most

The current behavior depends strongly on these values in `config/frontier_explorer_params.yaml`.

### GVD shape / density

- `gvd_min_clearance`
- `gvd_snap_radius`
- `gvd_distance_tolerance_cells`
- `gvd_max_site_direction_dot`

### GVD-only navigation policy

- `allow_gvd_bridge`
- `allow_safe_direct_frontier_nav`

### Segment behavior

- `gvd_segment_length`
- `gvd_segment_length_min`
- `gvd_segment_length_max`
- `gvd_waypoint_spacing`

### Replanning

- `replan_while_navigating`
- `gvd_replan_interval`
- `gvd_replan_min_remaining_distance`

### Stuck and pass-ahead behavior

- `progress_timeout`
- `stuck_recovery_gvd_radius`
- `gvd_goal_pass_projection_margin`
- `gvd_goal_pass_lateral_tolerance`

---

## What Was Explicitly Requested by the User

These are not abstract preferences. They are direct constraints from the user and should be preserved unless the user explicitly changes them.

1. Use the C++ frontier explorer, not the Python version.

2. Keep exploration behavior GVD-centric.

3. Treat frontier as direction, not raw travel endpoint.

4. Keep the same locked frontier instead of switching too early.

5. Update GVD while navigating.

6. Only refresh nearby frontier candidates, but keep the locked frontier.

7. Do not ban or discard a frontier too early just because the current GVD is sparse.

8. Do not force robot heading at Nav2 targets.

9. If a GVD target has already been effectively passed, do not flip around to touch it.

10. When stuck, get back to GVD first.

11. Avoid retry loops caused by unchanged SLAM maps.

---

## Current Known Limitations

These are the most important remaining issues.

### 1. Strict same-component GVD routing can still reject reachable exploration directions

The current normal frontier dispatch path is intentionally strict.
That matches user preference, but it can also be conservative in sparse or broken skeleton regions.

This means:

- some frontiers may remain deferred while the GVD is still incomplete,
- even though a human would say “the robot basically knows the right corridor to take.”

This is not necessarily wrong, but it is a practical limit of the current strict-track interpretation.

### 2. GVD sparsity is still a fundamental challenge

The GVD mask is stricter and better than earlier session versions, but it is still discrete.
In complicated environments the skeleton can still be sparse, fragmented, or only partially built.

### 3. Costmap inflation settings remain inconsistent with the robot footprint

This is visible in runtime warnings and can affect controller behavior.

### 4. Startup TF timing still has occasional race conditions

There are still runs where Nav2 reports temporary `base_link -> map` timing issues at startup.

---

## Recommended Next Stage of Work

If another AI continues from here, the next stage should probably focus on:

## Stage 1: Make “stay on track” more robust without forcing backwards corrections

This is the most aligned next step.

Focus:

- refine passed-waypoint handling,
- possibly advance to the next waypoint or next segment earlier,
- and prevent stale segment tails from forcing reversals.

Why:

- this directly matches the latest user complaint,
- and it improves efficiency without violating the GVD-track constraint.

### Likely concrete tasks

1. Add explicit active-segment progress indexing, not just “current goal = last waypoint.”
   Right now the segment is represented mostly by its last waypoint.
   A richer notion of “which waypoint has already been passed” could reduce unnecessary reversals further.

2. Consider dropping already-passed interior waypoints from the active segment.
   This may be better than canceling the full segment every time.

3. Re-evaluate whether `NavigateThroughPoses` segment ends should align with waypoint boundaries or with projected progress along the polyline.

## Stage 2: Improve effective GVD connectivity without violating the user’s “stay on the track” policy

The user does not want normal frontier travel to leave the GVD track.
So the next AI should avoid simply reintroducing loose free-space fallback.

Better directions:

- improve the GVD graph itself,
- improve snapping / component selection,
- or make sparse GVD usable through better waypoint chaining on the skeleton.

Possible work:

1. Better component-aware routing.

2. Better handling of sparse-but-directionally-correct GVD points.

3. Better dynamic segment advance when the robot has effectively moved onto the next usable part of the same skeleton.

## Stage 3: Clean up Nav2 local behavior around obstacles

This is secondary to exploration logic, but still important.

Focus:

- inflation radius vs footprint,
- controller overshoot near obstacles,
- possible overcommitment to stale local path fragments.

---

## What Another AI Should Not Do Blindly

Do not do these without checking with the user first.

1. Do not reintroduce a broad direct-to-frontier fallback.
   The user explicitly rejected that direction multiple times.

2. Do not assume frontier disappearance means “pick another frontier now.”
   The user wants locked frontier persistence.

3. Do not treat timestamp-only map updates as meaningful updates.
   This was a real bug and was explicitly addressed.

4. Do not re-enable heavy heading enforcement at waypoints or goals.

5. Do not silently “fix” GVD sparsity by simple morphological thickening unless the user agrees.
   The user cares about the semantic definition of GVD, not just visual continuity.

---

## Build / Run Commands

Typical build:

```bash
cd /home/chou/ros_ws
colcon build --packages-select auto_explore_sim
```

Typical run:

```bash
source /home/chou/ros_ws/install/setup.bash
ros2 launch auto_explore_sim auto_explore.launch.py
```

---

## Final State Summary

At the end of this session, the project is in this state:

- C++ explorer only
- modularized frontier / GVD responsibilities
- strict GVD-track exploration policy
- frontier lock persistence
- GVD updates during navigation
- content-hash-based retry suppression to stop same-map loops
- no explicit heading requirement for waypoint navigation
- direct MPPI controller instead of rotation shim
- passed-waypoint detection to avoid flip-around on already-cleared GVD goals

The most natural next work item is:

**make active GVD segment progress handling even smarter, so the robot can continue forward along the track without wasting motion on stale segment endpoints.**
