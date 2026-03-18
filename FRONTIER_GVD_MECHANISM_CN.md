# Frontier 与 GVD 机制说明

本文档面向当前 `auto_explore_sim` 的 `frontier_explorer` 实现，目标不是介绍“理想中的探索算法”，而是准确描述**当前代码到底在做什么**、**每一步用什么判据**、**为什么会停住/跳过/黑名单/完成**，方便后续继续排查问题。

这份文档主要对应以下文件：

- `src/frontier_explorer.cpp`
- `src/frontier_search.cpp`
- `src/gvd_map.cpp`
- `include/auto_explore_sim/gvd_map.hpp`
- `config/frontier_explorer_params.yaml`

如果代码和文档冲突，以代码为准。本文档反映的是当前阶段的实现状态，不代表最终设计已经稳定。

## 1. 当前设计目标

这套系统当前遵循的核心思路是：

- `frontier` 负责给机器人一个**探索方向**
- `GVD` 负责给机器人一条**实际可走的轨道**
- 正常情况下，机器人应该优先沿 `GVD skeleton` 走
- 当 `GVD` 对当前 frontier 不可用时，系统可以按配置退化为 `direct frontier fallback`
- 当前用户偏好的语义是：
  - 尽量不要离开 GVD
  - frontier 消失后也不要立刻换目标
  - GVD 更新不应该抹掉以前见过但还没访问的 frontier

所以判断一个 frontier 是否可执行，不只是“地图上有没有 frontier”，而是：

1. 有没有 frontier
2. frontier 能不能吸附到当前机器人所在的 GVD 连通分量
3. 吸附以后有没有可用 GVD path
4. 如果没有 GVD path，是否允许 direct fallback，并且 direct fallback 是否足够安全

## 2. 模块划分

### 2.1 `FrontierSearch`

职责：

- 在 occupancy grid 上提取 frontier
- 对 frontier 做聚类
- 计算 frontier 的 centroid、大小、距离信息
- 按启发距离排序

文件：

- `src/frontier_search.cpp`

### 2.2 `GvdMap`

职责：

- 从地图构建 GVD
- 提供 `dist_map / label_map / component_map / mask`
- 提供 frontier 到 GVD 的吸附
- 提供 GVD 上的路径搜索
- 提供 direct fallback 的 free-space path 搜索

文件：

- `src/gvd_map.cpp`
- `include/auto_explore_sim/gvd_map.hpp`

### 2.3 `FrontierExplorer`

职责：

- 维护地图缓存、frontier 缓存、黑名单、已完成目标
- 维护当前 locked frontier
- 调用 `FrontierSearch` 和 `GvdMap`
- 生成 `NavigateThroughPoses` / `NavigateToPose`
- 处理成功、失败、超时、卡住恢复

文件：

- `src/frontier_explorer.cpp`

## 3. 关键数据结构

### 3.1 Frontier

当前一个 frontier 至少带这些信息：

- `centroid_x / centroid_y`
- `size`
- `heuristic_distance`
- `min_distance`
- `cost`

当前排序规则是：

1. `heuristic_distance` 小的优先
2. 如果相同，`min_distance` 小的优先
3. 如果仍相同，`size` 大的优先

也就是说，当前是**纯最近优先**，不是“最大信息增益优先”。

### 3.2 GVD 数据

`GvdData` 里最关键的是：

- `dist_map`
  - 每个 cell 到最近障碍 boundary site 的距离
- `label_map`
  - 每个 cell 当前最近的 obstacle boundary site id
- `mask`
  - 这个 cell 是否被接受成 GVD cell
- `component_map`
  - GVD skeleton 的连通分量编号
- `world_x / world_y`
  - 该 GVD cell 对应的世界坐标，可能是 bisector 投影点，不一定是 cell 中心

## 4. Frontier 提取机制

### 4.1 什么叫 frontier cell

当前定义在 `src/frontier_search.cpp` 中：

- 当前 cell 必须是 `unknown`，即地图值为 `-1`
- 它的 8 邻域里至少有一个 `free` cell，地图值为 `0`

换句话说：

**frontier cell = 已知自由区和未知区的边界上的 unknown cell**

### 4.2 从哪里开始搜 frontier

搜索不是全图直接扫，而是：

1. 先从机器人当前位置投影到地图坐标
2. 如果机器人不在 free cell 上，先找附近最近 free cell
3. 从这个 free cell 开始做 BFS
4. 在 BFS 经过的 free 空间边缘找 frontier cell

这意味着当前 frontier 搜索天然有一个重要性质：

**它更容易找到“和机器人当前已知自由空间连着”的 frontier。**

如果某些 frontier 虽然在全局地图上看得见，但和当前 BFS 可达自由区域的拓扑关系不好，它们不一定能在本轮 `observed frontiers` 里出现。

### 4.3 frontier 聚类

当发现一个 frontier cell 后，会继续用 frontier BFS 把相邻 frontier cell 并成一个 cluster，然后统计：

- centroid
- `size`
- 机器人到该 cluster 的最近距离 `min_distance`

### 4.4 frontier 过滤条件

一个 frontier cluster 必须满足：

1. `size >= min_frontier_size`
2. `min_distance <= frontier_update_radius`
   - 只有在 `frontier_update_radius > 0` 时才启用

当前参数里：

- `min_frontier_size: 400`
- `frontier_update_radius: 5.0`

所以当前“当前 frontier 列表”其实是**大 frontiers + 离机器人较近的 frontiers**。

### 4.5 当前列表与缓存列表

`FrontierExplorer` 里维护两类 frontier：

1. `observed frontiers`
   - 当前这一轮 `FrontierSearch::search()` 刚搜出来的列表
2. `known_frontiers_`
   - 历史见过但还没访问、也没黑名单/完成的 frontier 缓存

系统当前的选择顺序是：

1. 先从当前 `observed frontiers` 里挑
2. 如果都不行，再去 `known_frontiers_` 里挑

这比旧版本“当前 + 历史混起来统一排序”更合理，因为它优先使用**当前地图上的最新 frontier**。

### 4.6 frontier 何时从缓存里删除

当前不会因为 GVD 更新就删除历史 frontier。

frontier 只会在以下情况被从 `known_frontiers_` 移除：

- 被 `complete_frontier(...)` 标记完成
- 被 `blacklist_point(...)` 标记黑名单

这点很重要，因为它意味着：

**地图/GVD 更新本身不会 wipe out 以前见过但还没访问的 frontier。**

## 5. GVD 构建机制

### 5.1 GVD 的输入

`GVD` 直接从当前 occupancy grid 构建。

关键配置在 `GvdConfig` 和参数文件里：

- `gvd_min_clearance`
- `gvd_snap_radius`
- `gvd_distance_tolerance_cells`
- `gvd_max_site_direction_dot`
- `allow_gvd_bridge`

### 5.2 第一步：找 obstacle boundary sites

当前不是把所有 obstacle cell 都当 site，而是只把：

**障碍物边界 cell**

当成 Voronoi site。

一个 obstacle cell 只要满足：

- 它本身是 obstacle
- 并且 8 邻域里至少有一个不是 obstacle，或者越界

就会变成一个 boundary site。

这些 boundary site 是 Brushfire / wavefront 的源点。

### 5.3 第二步：Brushfire 距离传播

对每个 free cell，计算：

- 到最近 boundary site 的距离 `dist_map`
- 最近 boundary site 的标签 `label_map`

这里不是简单曼哈顿距离，而是以 boundary site 为源做 wavefront，并用几何距离评估 candidate cost。

结果是每个 free cell 都会知道：

- “离最近障碍 boundary 多远”
- “最近的是哪一个 boundary site”

### 5.4 第三步：GVD cell 判定

一个 free cell 要被接受为 GVD cell，需要同时满足以下条件：

1. 它是 free cell
2. `dist_map >= gvd_min_clearance`
3. 从自己和 8 邻域收集到至少两个不同的 candidate site
4. 存在一对 site：
   - 它们到当前 cell 的距离差 `<= gvd_distance_tolerance_cells`
   - 指向这两个 site 的方向向量点积 `<= gvd_max_site_direction_dot`

这两个阈值分别代表：

- `gvd_distance_tolerance_cells`
  - “两侧障碍是否足够接近等距”
- `gvd_max_site_direction_dot`
  - “这两个障碍方向是否足够对向，而不是来自差不多同一侧”

所以当前 GVD 不是“所有中间位置都算”，而是：

**只有在离障碍够远、而且几何上确实像处在两侧障碍中间的 free cell，才进入 GVD mask。**

### 5.5 第四步：投影到局部 bisector

当某个 cell 被接受成 GVD cell 后，系统不会直接把它的世界坐标设成 cell center，而是会：

- 取刚才选中的两组 site
- 计算它们的局部等距 bisector
- 把这个 cell 的 world point 投影到 bisector 上

所以 `GVD marker` 看起来会比原始格子中心略平滑一些。

### 5.6 第五步：GVD 连通分量

在 `mask` 提取完成后，会对 GVD cell 做连通分量标号，得到 `component_map`。

这个结构后面非常关键，因为当前 frontier 的 GVD 吸附和 GVD 路径规划，都是围绕：

**机器人当前所在的 GVD 连通分量**

来做的。

## 6. Frontier 到 GVD 的吸附机制

### 6.1 当前规则

当前 frontier **只能**吸附到：

**机器人当前所在的 GVD 连通分量**

而不是“优先当前分量，不行再吸别的分量”。

具体流程：

1. 先找机器人当前位置最近的 GVD cell
2. 读出这个 cell 的 `component_id`
3. 在 frontier 附近、`gvd_snap_radius` 内，只搜索这个 component 里的 GVD cell
4. 选其中离 frontier 最近的点作为 snapped anchor

如果当前分量里根本没有候选点，就返回：

- `has_gvd_anchor = false`

### 6.2 吸附的真实半径

当前代码已经修过一个 bug：

以前虽然看起来用了 `gvd_snap_radius`，但实现上是扫一个方形窗口，所以方框角上的点可能实际欧氏距离已经超过半径。

现在已经改成：

- 候选点真实欧氏距离必须 `<= gvd_snap_radius`

所以现在 `snap radius` 是真正按圆形半径生效的。

### 6.3 为什么 same-component snap 很重要

如果 frontier 吸到别的 GVD 分量上，就会出现一种“假成功”：

- snap 成功了
- 但后面根本没有 same-component GVD route

所以现在同分量吸附的意义是：

**让“可吸附”和“可达”在拓扑上保持一致。**

## 7. GVD 路径规划机制

### 7.1 起点与终点

GVD path 不是从 frontier 本身开始，而是：

- 起点：机器人当前位置最近的 GVD cell
- 终点：snapped anchor 最近的 GVD cell

### 7.2 搜索方式

当前 `find_path(...)` 做的是：

- skeleton-only A*
- 邻接是 8 邻域
- 代价是几何步长 + 低清障惩罚
- 如果 `allow_gvd_bridge == false`
  - 只允许走 GVD mask 上的 cell
- 如果 `allow_gvd_bridge == true`
  - 当 skeleton-only 失败时，允许 free-space bridge

当前参数里：

- `allow_gvd_bridge: false`

所以当前正常探索更接近：

**严格沿 skeleton 走。**

### 7.3 GVD path 到 waypoint 的变换

如果找到 GVD cell path，则：

1. 先把 cell path 转成 world points
2. 按 `gvd_waypoint_spacing` 从原始 GVD 点序列中抽样
3. 如果路径质量好，则整段一次发出去
4. 如果路径质量一般，则只截取一段，形成 segment

当前已经去掉了原来会把 waypoint 从 GVD 上拉开的平滑步骤，所以：

**发给 `NavigateThroughPoses` 的 waypoint 现在应该来自原始 GVD 点序列，而不是插值平滑后的偏移点。**

## 8. 直达 frontier 的 fallback 机制

### 8.1 什么时候触发

当前 `build_direct_frontier_navigation_plan(...)` 只有在：

1. `allow_safe_direct_frontier_nav == true`
2. 没有可用 GVD route
3. 存在 free-space path
4. 这条 free-space path 的最小 clearance `>= direct_frontier_min_clearance_cells`

时才会返回一个 `DirectGoal` 计划。

当前参数：

- `allow_safe_direct_frontier_nav: true`
- `direct_frontier_min_clearance_cells: 6.0`

### 8.2 为什么很多 frontier 还是会停住

如果日志是：

- `no same-component GVD route to the snapped anchor; keeping it for a future GVD update`

而不是：

- `keeping direct navigation as a fallback`

那就说明：

**这个 frontier 的 direct fallback 也没有构出来。**

通常只有两种原因：

1. 根本没有 free-space path
2. 有 path，但 clearance 小于 `direct_frontier_min_clearance_cells`

所以“地图上还有 frontier，但 robot 停住”不一定是 frontier list 丢了，更可能是：

**所有已知 frontier 在当前约束下都不可执行。**

## 9. FrontierExplorer 的主流程

下面按 `make_plan()` 当前主干逻辑描述。

### 9.1 入口检查

每个 tick，节点先检查：

1. 是否拿到地图
2. 如果要求 live map，live map 是否 ready
3. 能否拿到机器人位置

### 9.2 正在导航时

如果 `navigating_ == true`，不会重新选新 frontier，而是：

- 先检查 progress / stuck
- 再检查是否需要**记录一次 deferred replan**

注意当前行为不是“中途 cancel 旧 GVD path 并重发新路径”，而是：

**只记录 map / robot 已变化，等当前 segment 走完以后再更新。**

### 9.3 空闲时重新规划

如果当前不在导航：

1. 取地图
2. 获取/重建 GVD
3. 搜索当前 frontier 列表
4. 如果附近没有 frontier，并且当前没有 locked frontier，再扩到全图搜索
5. 刷新 `known_frontiers_`
6. 得到：
   - `observed_valid`
   - `cached_valid`

### 9.4 如果有 locked frontier

系统优先处理当前已锁定的 frontier：

1. 尝试在当前 frontier 列表中匹配它
2. 如果匹配不到，也继续保留该 locked frontier
3. 对这个 locked frontier 重新构造 plan

如果 locked frontier：

- 没有 GVD anchor
- 没有 same-component GVD path
- 规划输入没 ready

则不会立刻放弃，而是先累计失败，再决定是否黑名单。

### 9.5 如果没有 locked frontier

则按顺序尝试：

1. 当前 frontier 列表
2. 缓存 frontier 列表
3. 如果本地都不行，再扩到全图 frontier 列表
4. 如果还是都不行，再尝试 direct fallback

### 9.6 真的没有可执行 frontier 时

会进入等待态，并记住：

- 当前 map hash

之后只有当地图内容真的变化时，才重新尝试，而不是仅因为 `stamp` 在变就反复刷循环。

## 10. 成功、完成、黑名单的区别

这一部分非常重要，因为很多“停住/误判”都出在这里。

### 10.1 `Navigation succeeded`

Nav2 成功只代表：

**当前一次 dispatch 的 goal 成功了**

这不一定代表：

- frontier 完成了

### 10.2 `Intermediate frontier anchor reached`

如果当前 segment 成功，但满足以下任一条件：

1. 当前 segment 不是直达 anchor 的最终段
2. 当前 goal 虽然成功，但离真正 frontier 还不够近

则这次成功只会被认为：

- 到达了一个中间 anchor

接下来会保留同一个 locked frontier，继续重规划下一步。

### 10.3 `Completing frontier`

只有在 explorer 判断：

- 当前 goal 已经真正到 frontier 附近

才会 `complete_frontier(...)`。

完成的语义是：

- 这是一个已访问完的 frontier
- 不应该继续作为候选目标

它和黑名单不是一回事。

### 10.4 `Blacklisting`

黑名单是“暂时别再试这个目标”，通常来自：

- goal 被 Nav2 reject
- 多次规划失败
- 多次执行失败
- 长时间 no progress 且恢复也失败

黑名单目标会在：

- `blacklist_timeout`

之后过期。

## 11. 为什么系统会“看起来还有 frontier，但 robot 停住”

这通常不是单一原因，而是下面几类机制叠加。

### 11.1 frontier 搜到了，但都没有 same-component GVD anchor

日志特征：

- `no usable GVD anchor within snap radius`

这表示：

- frontier 有
- 但在机器人当前 GVD 连通分量里，`gvd_snap_radius` 内没有可吸附点

### 11.2 frontier 有 anchor，但没有 same-component GVD route

日志特征：

- `no same-component GVD route to the snapped anchor`

这表示：

- frontier 已经吸到当前 GVD 分量上的某个 anchor
- 但机器人当前 GVD 点到这个 anchor 的 skeleton path 不连通

### 11.3 GVD 不通，同时 direct fallback 也没过

日志特征：

- `keeping it for a future GVD update`

而不是：

- `keeping direct navigation as a fallback`

这说明：

- GVD 不可用
- direct fallback 也不可用

此时系统就只能等待地图变化。

### 11.4 frontier 被错误地提前“完成”

这是前面修过的一个重要 bug：

- frontier 可能被吸到一个离 robot 很近、但离真正 frontier 很远的 anchor
- robot 很快到达这个 anchor
- explorer 却把整个 frontier 当完成

这个问题的本质是：

**“到达 anchor”不等于“到达 frontier”。**

### 11.5 frontier centroid 抖动，看起来像重复 frontier

同一个物理 frontier 在不同 map 更新下，cluster 大小和 centroid 可能略变，所以你会看到：

- `(-3.07, 1.81)`
- `(-3.09, 1.81)`

这种几厘米级别的变化。

它们不一定是两个不同 frontier，更可能是：

**同一个 frontier 的 centroid 在抖。**

## 12. 目前最值得重点讨论的失效点

如果我们接下来要继续排查“为什么还会错”，我建议重点盯下面几个问题。

### 12.1 same-component 约束会不会过严

当前 frontier 只能吸到 robot 当前 GVD 分量。

优点：

- 拓扑一致
- 避免假成功

缺点：

- 在 GVD 很碎时，可执行 frontier 会骤减

所以当前系统“停住”的一个主要来源，就是：

**GVD 分量约束太硬，而地图上的有效 frontier 还不少。**

### 12.2 `gvd_snap_radius` 是否太小

如果半径太小，很多 frontier 会直接报：

- `no usable GVD anchor within snap radius`

这不是 frontier 本身无效，而是：

**当前 GVD 吸附窗口不够大。**

### 12.3 `direct_frontier_min_clearance_cells` 是否太高

当前是：

- `6.0`

如果这个值太高，很多本来“还能走”的 direct fallback 会被判掉，系统就会停住等待 map growth。

### 12.4 `min_frontier_size` 是否太大

当前是：

- `400`

这会过滤掉很多小 frontier。对噪声有好处，但也可能让一些真实可探索入口被忽略。

### 12.5 当前 frontier 搜索仍然是 BFS-based reachable frontier search

它天然更偏向“当前 free 区边缘”的 frontier，而不是全图几何意义上的所有 frontier。

所以如果你在 RViz 里“肉眼能看到 frontier”，但 explorer 不一定把它当作当前可选候选，这件事本身是有可能的。

## 13. 看日志时怎么解释

下面给出一组常见日志到语义的映射。

### 13.1 `Best frontier: (...)`

表示当前已经选出了一个候选 frontier，后面会马上尝试给它构 plan。

### 13.2 `Planning: frontier anchor moved from ... to nearest GVD point ...`

表示 frontier 成功吸附到了当前机器人 GVD 分量中的某个 anchor。

### 13.3 `GVD path found: ...`

表示：

- robot 当前 GVD 点到该 anchor 之间找到了可用 GVD path

### 13.4 `Intermediate frontier anchor reached`

表示：

- 当前 segment 成功
- 但这还不是 frontier 完成
- 接下来仍应继续朝同一个 frontier 重规划

### 13.5 `Completing frontier (...)`

表示：

- explorer 认为这个 frontier 已经真正到达，应从候选里移除

如果这条日志出现得太早，就说明“完成判据”可能还不对。

### 13.6 `No usable frontier remains ... waiting for a real map change`

这句的准确含义不是“地图上没有 frontier”，而是：

**当前 observed 列表 + cached 列表里，没有一个 frontier 在当前约束下能形成可 dispatch 的 plan。**

## 14. 当前机制的一句总结

当前实现的真实链条可以概括成：

1. 机器人附近 BFS 找 frontier
2. frontier 进入 observed list
3. 和 known frontier cache 合并成候选池
4. frontier 只能吸到 robot 当前 GVD 连通分量
5. 优先找 same-component GVD path
6. GVD path 存在就走 `NavigateThroughPoses`
7. GVD path 不存在就尝试 direct fallback
8. direct fallback 也失败就等待地图变化
9. segment 成功不等于 frontier 完成，只有真正靠近 frontier 才 `complete`

如果后面要继续讨论“哪里错了”，建议按这 9 步逐步定位，而不是只看最终停住的现象。

## 15. 建议的后续排查顺序

如果我们接下来要继续 debug，我建议按下面顺序看：

1. 当前 frontier 有没有被搜索出来
2. 它有没有进入 `observed_valid` 或 `cached_valid`
3. 它有没有 same-component GVD anchor
4. 它有没有 same-component GVD path
5. direct fallback 为什么没接住
6. 成功后的 `complete` 是否发生得太早

这样最容易把“frontier list 问题”和“GVD reachability 问题”分开。
