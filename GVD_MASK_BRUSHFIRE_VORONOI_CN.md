# GVD 掩膜改进说明（相对上一版 Git 提交）

本文说明 `auto_explore_sim/src/frontier_explorer.cpp` 中，GVD 掩膜与 GVD 路径规划相对上一版 Git 基线的改进。这里的“上一版”指当前分支 `wrong_gvd` 上工作树修改前的提交 `975c3b2 (v1)`。

## 1. 问题背景

在前沿探索里，GVD（Generalized Voronoi Diagram）承担两个作用：

1. 给前沿评分提供“离障碍的几何中心线”信息。
2. 给 `NavigateThroughPoses` 提供一条尽量沿走廊中线的导航路径。

上一版实现虽然已经有 “GVD” 相关逻辑，但严格来说它更接近“对可通行区域做形态学细化得到的骨架”，而不是“到两组最近障碍等距的 Voronoi 脊线”。这会导致两个直接问题：

1. GVD 掩膜在几何意义上不稳定，容易随栅格形状出现偏移。
2. 在 SLAM 仍不完整时，骨架容易断裂，远端 frontier 经常退化成单点目标。

## 2. 上一版实现的问题

上一版主要流程是：

1. 以障碍和未知区域作为波前源点做距离传播。
2. 把满足最小 clearance 的所有自由单元先全部标成前景。
3. 对这个前景做 Zhang-Suen thinning，得到 1 像素骨架。
4. 在这条“细化骨架”上做 A*，否则退回单目标导航。

这个做法有几个核心问题。

### 2.1 掩膜语义不是严格 Voronoi

Zhang-Suen thinning 是形态学骨架提取，不是 Voronoi 条件判定。  
它保证“细”，但不保证“该点到两类最近障碍等距”。

因此，上一版的 `gvd_mask_` 本质上是：

- “自由空间中 clearance 足够的区域”
- 再经过一次拓扑细化

而不是：

- “由至少两个障碍边界 site 竞争产生的 Voronoi 脊线”

### 2.2 未知区域被当成障碍源点

上一版把 `-1` 未知栅格也作为波前 seed。这样做虽然会让骨架在未知边界附近更“满”，但会引入一个几何偏差：

- frontier 边界本身不是永久障碍
- 但被当成障碍 source 后，会把 GVD 拉向未知边界

结果就是：

- GVD 在探索初期容易被未观测边界污染
- frontier 附近的“中轴线”不再只由真实障碍定义

### 2.3 远端 frontier 时骨架容易断裂

上一版的 skeleton 很依赖当前局部地图拓扑。一旦 SLAM 还没补全，细化后的骨架容易出现：

- 局部断点
- 细小孤立分支
- 组件之间不连通

这会让远端 frontier 虽然在自由空间里可达，但在 skeleton 上不可达，最终频繁退回到单点目标。

## 3. 这次改进的目标

这次改动的目标不是“让骨架看起来更细”，而是让 GVD 满足更接近几何定义的约束：

1. 只由真实障碍边界定义 GVD。
2. GVD 点由多个障碍边界 site 的竞争关系决定。
3. RViz 里看到的 GVD 线条尽量贴近局部等距中线，而不是简单走格子中心。
4. 当 partial SLAM 导致 skeleton 断开时，仍尽量输出一条“以 GVD 为主、必要时短距离离开 GVD”的多航点路径。

## 4. 具体改动

### 4.1 Brushfire 源点改为“真实障碍边界 site”

当前实现不再把未知区域当作 seed，也不再把整个自由区直接细化。

现在的做法是：

1. 只对 `occupancy > 50` 的真实障碍建模。
2. 只把“障碍边界单元”作为 Voronoi site。
3. 通过多源 wavefront（Brushfire / priority queue）传播最近 site。

代码上新增了：

- `WavefrontEntry`
- `site_xs_ / site_ys_`
- `label_map_` 记录每个格点最近的障碍边界 site
- `dist_map_` 使用 `double` 保存更平滑的距离值

这样得到的距离场语义是：

- 每个自由格点都关联到一个最近障碍边界 site
- 后续 Voronoi 判定可以基于“site 竞争”而不是“形态学细化”

### 4.2 GVD 掩膜改为“局部 site 竞争 + 等距判定”

新的 `extract_gvd_mask()` 不再先把整片自由区做 thinning，而是对每个自由格点直接检查：

1. 当前点是否满足最小 clearance。
2. 当前点及邻域内是否存在至少两组不同的候选 site。
3. 这两组 site 到当前点的距离差是否足够小。
4. 两个方向向量是否足够分离，而不是来自同一侧墙面。

当前判定策略是：

- 距离差阈值：`kVoronoiDistanceToleranceCells = 0.75`
- 方向分离约束：`dot <= 0.5`

含义是：

- 只有当两个障碍边界 site 对当前点形成明显“竞争关系”时，当前点才会被接受为 Voronoi 点
- 可以有效抑制同一面墙上相邻栅格带来的伪 Voronoi 点

### 4.3 GVD 显示点改为“投影到局部二等分线”

上一版 RViz 里的 GVD 和路径点主要落在格心或格角上，视觉上会比较离散。

现在新增了：

- `project_to_voronoi_bisector()`
- `gvd_world_point_for_cell()`
- `gvd_world_x_ / gvd_world_y_`

对每个被接受的 GVD 栅格：

1. 取出构成该点 Voronoi 条件的两个最佳 site。
2. 计算它们的中垂线。
3. 把当前格点投影到这条局部 bisector 上。

这样做的效果是：

- RViz 中看到的 GVD 线段更贴近连续中线
- `find_gvd_path()` 输出的路径点不再只是“踩格子中心”

## 5. 为什么还要改路径规划

即使 GVD 掩膜已经更正确，partial SLAM 仍然会带来一个现实问题：

- GVD 组件可能在当前时刻不连通
- 但对应 frontier 在已知自由空间里其实是可达的

如果坚持“只能走纯 skeleton”，远端 frontier 仍然会频繁失败。

因此，这次对路径层也做了补强，但没有改变 GVD 掩膜定义。

### 5.1 两阶段搜索

当前 `find_gvd_path()` 采用两阶段：

1. 先做纯 GVD skeleton A*。
2. 如果 skeleton 断开，则退到“GVD 引导的自由空间 A*”。

第二阶段仍然偏向 GVD，而不是退化成完全普通的自由空间最短路。

### 5.2 GVD 引导的自由空间 bridge

在 bridge 模式里，A* 允许走已知自由空间，但会附加两个代价：

- `kOffSkeletonStepPenalty = 3.0`
- `kLowClearancePenaltyScale = 0.5`

这意味着：

- 只要 skeleton 连得上，路径会优先留在 skeleton 上
- 只有在 skeleton 断开时，才会短距离离开 GVD 去桥接两个组件
- 桥接时也会倾向于 clearance 更高的自由空间

## 6. 实际效果

在本次验证里，已经观察到两类典型结果。

### 6.1 完全在 GVD 上的路径

运行日志中可见：

```text
GVD rebuilt: 75 boundary sites, 67 Voronoi cells
GVD path: 18 cells, 18 on skeleton (100%)
GVD path found: 18 cells -> 3 waypoints
```

这说明：

- 当前掩膜已经不是之前那种大面积形态学骨架
- 但仍足够连通，能直接输出纯 GVD 多航点路径

### 6.2 Skeleton 断开但仍成功桥接

更关键的一类日志是：

```text
GVD path: skeleton disconnected after exploring 31 cells, retrying with free-space bridge
GVD path: 25 cells, 23 on skeleton (92%)
GVD path found: 25 cells -> 4 waypoints
```

这说明：

1. skeleton 的确在当前局部地图上断开了
2. 系统没有直接退回单点目标
3. 而是只用很短的非 GVD 段完成桥接
4. 最终仍输出 `NavigateThroughPoses` 所需的多航点路径

这正是这次想解决的核心问题。

## 7. 与上一版相比，实质上改了什么

可以把这次改动概括成三句话：

1. **GVD 定义从“细化骨架”改成了“障碍边界 site 竞争产生的 Voronoi 掩膜”。**
2. **GVD 显示与路径点从“格点显示”改成了“局部 bisector 投影显示”。**
3. **路径规划从“纯 skeleton 失败就退单点”改成了“纯 skeleton 优先，失败后做最小化自由空间桥接”。**

## 8. 仍然存在的边界条件

这次改动已经把“partial SLAM 导致 skeleton 断开”从主要失败原因降下来了，但仍有两个边界条件需要明确：

1. 如果已知自由空间本身就不连通，bridge A* 也无法构造多航点路径。
2. Voronoi 判定里的阈值（`0.75` / `0.5`）仍然是工程参数，不是连续空间解析解。

不过和上一版相比，现在失败条件已经收缩到了真正的“地图不可达”，而不是“GVD 定义本身不稳定”。

## 9. 相关代码位置

主要实现位于：

- `src/frontier_explorer.cpp`
  - `compute_distance_and_labels()`
  - `extract_gvd_mask()`
  - `project_to_voronoi_bisector()`
  - `find_cell_path()`
  - `find_gvd_path()`

如果后续还要继续优化，建议优先考虑：

1. 把 Voronoi 判定阈值参数化到 yaml。
2. 对 bridge A* 的代价项继续做参数化。
3. 在 RViz 里区分“纯 GVD 段”和“bridge 段”。
