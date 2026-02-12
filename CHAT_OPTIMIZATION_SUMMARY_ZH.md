# 自主探索优化会话总结（中文）

## 1. 目标与背景
本次会话围绕 `auto_explore_sim` 的前沿探索器与 Nav2 导航参数做了连续优化，核心目标是：

1. 降低前沿目标“贴墙/落在膨胀区”导致的失败率。  
2. 让目标更符合 Voronoi / GVD 中轴线思想（靠近走廊中心）。  
3. 解决“重复同一目标、看起来不更新前沿”的卡顿行为。  
4. 提升运动激进度，尤其在狭窄和近障环境下速度与通过性。  
5. 将前沿选择的距离评价从欧氏距离改为可达路径距离（heuristic / graph distance）。

---

## 2. 核心优化项（最终状态）

### 2.1 前沿目标选择与代价函数
文件：`scripts/frontier_explorer.py`

1. `Frontier` 数据结构扩展：增加 `cells` 与 `goal` 字段。  
2. 前沿评分支持多因素：  
   `cost = path_distance - clearance_scale * clearance - orientation_scale * heading_bonus`  
3. “距离”已改为**可达路径距离**（Dijkstra 8 邻域），不是直线欧氏距离。  
4. 仍保留 `min_goal_distance=0.5` 作为防止近点循环的重要约束。  

---

### 2.2 GVD / Voronoi 相关实现
文件：`scripts/frontier_explorer.py`

1. 新增基于多源 BFS 的障碍距离场与最近障碍标签（Voronoi 区域标签）。  
2. 基于标签不连续性提取 Voronoi 边（近似 GVD skeleton）。  
3. 为每个自由栅格预计算“最近 skeleton 栅格投影”。  
4. 关键策略已按你要求改为：  
   - **先正常选择前沿目标**（不在候选阶段做 skeleton 过滤），  
   - **再将“已选中目标”投影到最近 skeleton 边**。  
5. `gvd_projection_radius <= 0` 时，表示不限制投影半径（全局最近边）。  
6. 每次 `/map` 更新时会使 GVD 缓存失效并重建，保证边随 SLAM 动态更新。  

---

### 2.3 可视化增强（RViz）
文件：`scripts/frontier_explorer.py`

1. 继续在 `/explore/frontiers` 发布前沿球体。  
2. 增加 `gvd_skeleton` 的 `Marker.POINTS` 显示（同一 `MarkerArray` 内），可直接查看 skeleton。  
3. 日志增加 `on_gvd` 与 `projected`，用于确认目标是否在骨架上、是否发生投影。  

---

### 2.4 卡顿与重复目标修复
文件：`scripts/frontier_explorer.py`, `config/frontier_explorer_params.yaml`

1. 增加 `same_goal_cooldown` 机制。  
2. 当目标短时间不变时先抑制重复发送；超过冷却时间后允许重发，避免永久“只打印 Best frontier 不发送 goal”的假死。  

---

### 2.5 近墙卡住恢复策略
文件：`scripts/frontier_explorer.py`, `config/frontier_explorer_params.yaml`

1. 当 `progress_timeout` 超时且检测为近墙场景，触发恢复目标（`recovery`）。  
2. 恢复目标优先选择更开阔区域，成功后立即回到前沿重规划。  
3. 前沿黑名单仅针对 `frontier` 目标，不污染恢复目标。  

---

### 2.6 Nav2 激进度提升
文件：`config/nav2_params.yaml`

已做偏激进调参，主要包括：

1. MPPI 加速度/角加速度上调（`ax_max/ax_min/az_max`）。  
2. `CostCritic` 降低保守度（`cost_weight` 下调、`near_collision_cost` 调整到更靠近 lethal 才强制减速）。  
3. `PathFollowCritic` 略增强以提高高速路径跟随稳定性。  
4. `velocity_smoother` 加减速度上调。  
5. `collision_monitor` 的 `FootprintApproach` 提前时间收紧为更激进策略。  

---

## 3. 关键参数（当前值）
文件：`config/frontier_explorer_params.yaml`

1. `min_goal_distance: 0.5`  
2. `same_goal_cooldown: 4.0`  
3. `gvd_goal_on_skeleton: true`  
4. `gvd_projection_radius: 0.0`（无限制，找最近边）  
5. `gvd_min_clearance: 0.1`  
6. `gvd_visualize: true`  
7. `orientation_scale: 0.5`  
8. `orientation_window_deg: 60.0`  
9. `stuck_escape_enabled: true`

---

## 4. 本轮解决过的典型问题

1. **“到达即成功导致循环”**：由目标容差与近点选择共同触发，已通过多处策略（最小目标距离、同目标冷却等）抑制。  
2. **“All frontier goals rejected”**：经历过筛选过严阶段，最终改为“选中后再投影”，避免候选集被过早清空。  
3. **“Best frontier 重复打印但不发 goal”**：已由 `same_goal_cooldown` + 超时重发机制修复。  

---

## 5. 验证建议

1. 启动后在日志中观察：`on_gvd=`、`projected=`、`Same goal persisted ... resending`。  
2. RViz 观察 `/explore/frontiers`：  
   - 前沿球体是否更新，  
   - `gvd_skeleton` 点云是否随地图扩展变化。  
3. 若激进参数导致震荡，可先小幅回调：  
   - `CostCritic.cost_weight`、  
   - `collision_monitor.FootprintApproach.time_before_collision`、  
   - `velocity_smoother.max_accel`。  

---

## 6. 变更文件清单

1. `scripts/frontier_explorer.py`  
2. `config/frontier_explorer_params.yaml`  
3. `config/nav2_params.yaml`

