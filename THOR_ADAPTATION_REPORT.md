# Thor Adaptation Report

## Scope
Compared `real_GVD` vs `on_thor` in `src/auto_explore_sim`.

- Compared branches: `real_GVD..on_thor`
- Divergence: one adaptation commit (`48ff405`, message: `adept to thor env`)
- Files changed: `config/nav2_params.yaml`, `launch/auto_explore.launch.py`, `launch/nav2_tuning.launch.py`, `package.xml`, `scripts/frontier_explorer.py`

## What Changed

### 1) Hard compatibility changes (Thor-specific)

1. Removed GVD/Voronoi costmap plugin from Nav2 global costmap:
   - `plugins` changed from `["static_layer", "obstacle_layer", "inflation_layer", "voronoi_layer"]`
     to `["static_layer", "obstacle_layer", "inflation_layer"]`
   - Removed `voronoi_layer` block using `nav2_voronoi_layer::VoronoiLayer`
2. Removed runtime dependency from `package.xml`:
   - deleted `<exec_depend>nav2_voronoi_layer</exec_depend>`
3. Made explorer interpreter explicit:
   - shebang changed from `#!/usr/bin/env python3` to `#!/usr/bin/python3`

### 2) Launch behavior changes

In both launch files:
- `launch/auto_explore.launch.py`
- `launch/nav2_tuning.launch.py`

Gazebo GUI condition changed from `UnlessCondition(headless)` to:
- `IfCondition(PythonExpression(["\"", use_rviz, "\" == \"true\" or \"", headless, "\" == \"false\""]))`

Effect: GUI opens whenever `use_rviz:=true`, even if `headless:=true` was passed.

### 3) Nav2 tuning changes

`config/nav2_params.yaml` contains controller/critic tuning updates, including:
- `controller_frequency: 20.0 -> 25.0`
- `costmap_update_timeout: 0.30 -> 0.20`
- MPPI horizon/noise/iterations/tolerance tuning:
  - `time_steps: 56 -> 48`
  - `model_dt: 0.05 -> 0.04`
  - `vx_std: 0.1 -> 0.14`
  - `wz_std: 0.2 -> 0.28`
  - `iteration_count: 2 -> 3`
  - `prune_distance: 2.5 -> 1.8`
  - `transform_tolerance: 0.5 -> 0.2`
  - `temperature: 0.3 -> 0.25`
- Critic weight/threshold changes (`GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`)
- Recovery/smoothing updates:
  - `max_rotational_vel: 1.0 -> 2.0`
  - `smoothing_frequency: 20.0 -> 25.0`

These improve responsiveness and path commitment but are not the primary startup blocker.

## Why `real_GVD` Fails on Thor but `on_thor` Runs

Primary root cause: missing `nav2_voronoi_layer` on Thor.

Evidence from this machine:
- `ros2 pkg prefix nav2_voronoi_layer` returns `Package not found`.

`real_GVD` requires `nav2_voronoi_layer` in both config and package dependency. At Nav2 bringup time, pluginlib tries to load `nav2_voronoi_layer::VoronoiLayer`; if the plugin package is absent, global costmap configuration fails, then lifecycle activation fails, so navigation/exploration does not run.

`on_thor` removes that plugin and dependency, so Nav2 starts with only standard layers (`static`, `obstacle`, `inflation`), which are available on Thor.

Secondary hardening in `on_thor`:
- Explicit `/usr/bin/python3` shebang avoids interpreter ambiguity on environments where `python3` resolves to non-system Python first (Thor has Conda-first `python3` in PATH).

## Same Adaptation Applied to `src/go2w_office_sim`

Created branch:
- `nre` (from `master`)

Applied corresponding Thor adjustments:
1. `config/nav2_params.yaml`
   - mirrored the same MPPI/controller/critic/recovery/smoother tuning deltas used in `on_thor`
2. `scripts/frontier_explorer.py`
   - shebang set to `#!/usr/bin/python3`

Not needed in `go2w_office_sim`:
- No `nav2_voronoi_layer` usage found in config/package, so no Voronoi removal was required.
- No `headless`-gated Gazebo client logic matching `auto_explore_sim`, so no equivalent GUI-condition patch was required.
