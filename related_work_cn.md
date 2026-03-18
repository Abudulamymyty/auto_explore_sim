# Frontier + GVD 相关工作综述（中文）

本文档面向当前 `auto_explore_sim` 的探索系统，目标不是做一份完整的学术综述，而是回答一个更工程化的问题：

**我们现在做的“frontier + GVD”这条路线，在已有文献里有没有相近工作？它们分别解决了什么问题？和我们现在的实现对应在哪里？**

为了后续讨论方便，本文按“和当前代码的相关度”来组织，而不是按年份机械排列。

## 1. 先给结论

如果只看路线相似性，当前系统最接近的是下面这组组合：

- `Yamauchi 1997`：frontier-based exploration 的经典原点
- `Choset & Burdick 2000`：把 GVD/GVG 当成探索与导航骨架
- `Gao et al. 2020`：把 frontier 与 GVD-based topological map 结合成 hybrid global-local exploration
- `Chen et al. 2023`：更直接地把 GVD 和 frontier selection / assignment 放进同一个探索框架

如果用一句话概括我们当前的系统：

**它更像是一个“frontier 给方向，GVD 给轨道”的 hybrid exploration 系统。**

这不是单纯的 frontier planner，也不是单纯的 topological planner，而是两者之间的一条工程化折中路线。

## 2. 与当前工作最相关的论文

### 2.1 Yamauchi, 1997

**论文**

- Brian Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, CIRA 1997  
  链接: [CMU PDF](https://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated2/yamauchi_frontier_explor.pdf)

**核心思想**

- frontier 是“已知自由空间”和“未知空间”的边界
- 机器人反复检测 frontiers
- 导航到最近的、可达的、未访问的 frontier
- 如果对某个 frontier 长时间无法前进，则把它加入 inaccessible frontier list，再去尝试下一个

**和我们当前系统的对应点**

- 我们当前仍然使用 frontier 作为探索方向的主要来源
- `FrontierSearch` 也是基于 grid frontier 的检测与聚类
- 当前系统里的：
  - `completed frontier`
  - `blacklist`
  - `no progress -> retry / blacklist`
  本质上都能追溯到 Yamauchi 这条思路

**和我们的不同点**

- Yamauchi 的核心是 frontier 本身，不强调 GVD
- 我们当前系统多加了一层：
  - frontier 不直接等于导航终点
  - frontier 先吸到 GVD anchor
  - 再沿 GVD 或 fallback 去执行

**为什么这篇重要**

因为它定义了探索问题里最基本的状态机语义：

- 什么是 frontier
- 什么是 visited frontier
- 什么是 inaccessible frontier
- 为什么 exploration 是“不断选下一个 frontier”的循环

这对我们现在排查“该完成、该黑名单、还是该保留重试”非常重要。

## 2.2 Choset & Burdick, 2000

**论文**

- Howie Choset, Joel Burdick, *Sensor-Based Exploration: The Hierarchical Generalized Voronoi Graph*, IJRR 2000  
  链接: [CMU Robotics Institute](https://www.ri.cmu.edu/publications/sensor-based-exploration-the-hierarchical-generalized-voronoi-graph/)

**核心思想**

论文把 HGVG / GVG 当成一个探索与导航骨架。作者明确指出，机器人可以：

- 先规划到 HGVG 上
- 再沿 HGVG 走
- 最后从 HGVG 离开到目标

原文摘要里就写得很像我们现在的设计：

- path onto the HGVG
- then along the HGVG
- finally from the HGVG to the goal

**和我们当前系统的对应点**

- 这几乎就是我们现在“frontier 给方向，GVD 给轨道”的理论原型
- 我们当前做的：
  - frontier 先 snap 到 GVD
  - 再走 same-component GVD path
  - 必要时再离开 GVD 去做 direct fallback
  与这篇的分层路径结构非常接近

**和我们的不同点**

- Choset 这篇更偏理论和 roadmap connectivity
- 我们的实现是 occupancy-grid + Nav2 + ROS2 的工程化版本
- 我们现在还要处理：
  - partial SLAM map
  - GVD 稀疏/断裂
  - frontier 缓存
  - Nav2 execution failure

**为什么这篇重要**

如果我们要为“GVD 是轨道”这件事找最强理论支撑，这篇是最关键的参考。

## 2.3 Gao, Booker, Wang, 2020

**论文**

- Wenchao Gao, Matthew Booker, Jiadong Wang, *Self-Exploration in Complex Unknown Environments using Hybrid Map Representation*, arXiv 2020  
  链接: [arXiv](https://arxiv.org/abs/2004.08535)

**核心思想**

这篇非常接近我们当前系统。它提出：

- 用 **modified GVD-based topological map**
- 加上 **grid-based metric map**
- 做一种 **frontier-driven exploration strategy**
- 用 global-local 的层次化方式决定 exploration command

摘要里明确写到：

- hybrid map representation
- modified GVD-based topological map + grid-based metric map
- frontier-driven exploration strategy
- global-local exploration strategy

**和我们当前系统的对应点**

这篇是我认为**和我们整体架构最像**的一篇。对应关系大概是：

- 他们的 `grid-based metric map`
  - 对应我们当前 occupancy grid / costmap / frontier detection
- 他们的 `GVD-based topological map`
  - 对应我们当前 `GvdMap`
- 他们的 hierarchical exploration
  - 对应我们当前“frontier 选择 + GVD 执行”两层结构

**和我们的不同点**

- 他们更强调 hybrid map representation 的整体框架
- 我们更具体地实现了：
  - same-component GVD snapping
  - locked frontier
  - deferred replanning between segments
  - blacklist / completed frontier / cached frontier

**为什么这篇重要**

如果我们要解释“为什么不是单纯 frontier，而是 frontier + GVD 的 hybrid 结构”，这篇最适合做总框架参照。

## 2.4 Chen et al., 2023

**论文**

- Dingfeng Chen et al., *GVD-Exploration: An Efficient Autonomous Robot Exploration Framework Based on Fast Generalized Voronoi Diagram Extraction*, arXiv 2023  
  链接: [arXiv](https://arxiv.org/abs/2309.06041)

**核心思想**

这篇比 Gao 2020 更直接地把 `GVD` 和 `frontier` 融合在一个探索框架里。论文摘要提到三个核心部件：

- real-time GVD construction
- a GVD-based heuristic scheme that accelerates frontier extraction and reduces frontier redundancy
- a multi-choice frontiers assignment scheme

**和我们当前系统的对应点**

这篇和我们最直接相似的地方在于：

- GVD 不只是拿来可视化，而是直接参与 frontier 筛选和决策
- frontier 不是单纯“看到就去”，而是和 GVD 状态耦合

**和我们的不同点**

- 他们的文章里用了端到端网络去构造 GVD
- 我们当前是更传统、更可解释的 Brushfire + Voronoi grid method
- 他们更偏“frontier heuristic acceleration”
- 我们更偏“frontier 选目标，GVD 当铁路”

**为什么这篇重要**

这篇是最直接说明“frontier + GVD”不是一个很怪的组合，而是已经成为一条明确研究路线的证据。

## 3. 与 frontier 管理机制密切相关的论文

### 3.1 Senarathne et al., 2015

**论文**

- *Incremental Algorithms for Safe and Reachable Frontier Detection for Robot Exploration*, Robotics and Autonomous Systems, 2015  
  链接: [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S0921889015001232)

**核心思想**

这篇关注的不是“frontier 作为概念”，而是更贴近工程实现的问题：

- frontier 是否 safe
- frontier 是否 reachable
- frontier detection / reachability information 如何增量维护

**和我们当前系统的对应点**

这篇和我们当前最相关的不是 GVD，而是这些问题：

- 地图上有 frontier，但当前是不是能去
- 当前不能去，是暂缓还是直接丢掉
- frontier 状态如何随着地图更新而变化

这和我们现在正在处理的这些现象非常接近：

- `no usable GVD anchor`
- `no same-component GVD route`
- `keeping it for a future GVD update`
- `waiting for a real map change`

**为什么这篇重要**

因为我们当前很多 bug，其实不是 frontier 检测 bug，而是 frontier **可执行性判定**过严或状态管理不清晰。  
这篇可以作为“frontier 不是看到就能执行”的直接参考。

### 3.2 Kim et al., 2020

**论文**

- *Graph Search-Based Exploration Method Using a Frontier-Graph Structure for Mobile Robots*, Sensors 2020  
  链接: [MDPI](https://www.mdpi.com/1424-8220/20/21/6270)

**核心思想**

这篇把 exploration 看成：

- 不断检测 frontier node
- 把 frontier node 和它们之间的关系存入 frontier-graph
- 用图搜索来选下一步目标

论文中明确说 frontier-graph 会记录 frontier node、local maps、parent-child / sibling edges 等信息。

**和我们当前系统的对应点**

这和我们现在维护：

- `observed frontiers`
- `known_frontiers_`
- `completed`
- `blacklisted`
- locked frontier

这些机制很接近。  
虽然我们现在还没有显式 frontier-graph，但我们已经不再是“每轮只看当前 frontier 列表”的简单策略，而是开始维护 frontier 的历史状态。

**为什么这篇重要**

如果我们后面继续把 `known_frontiers_` 从“缓存列表”升级为“真正的 frontier state graph”，这篇会很有参考价值。

## 4. 与 topological / semantic exploration 相关的论文

### 4.1 Gomez et al., 2019

**论文**

- *Topological Frontier-Based Exploration and Map-Building Using Semantic Information*, Sensors 2019  
  链接: [MDPI](https://www.mdpi.com/1424-8220/19/20/4595)

**核心思想**

这篇把 frontier detection、frontier classification 和 topological map-building 放在一起。  
文中提到：

- frontiers 使用 middle point 和 size 表征
- frontier 还会分成 free area / transit area（例如门）
- 探索完成后，构建出的 topological map 可以用于后续导航

**和我们当前系统的对应点**

我们虽然没有 semantic frontier classification，但在系统结构上已经有一点相似：

- frontier 不是孤立点
- frontier 选择要结合更高层结构信息
- 探索结束后保留的是更结构化的可导航信息，而不只是 occupancy grid

**为什么这篇重要**

如果后面你要把 frontier 进一步区分成：

- corridor frontier
- doorway frontier
- room frontier
- dead-end frontier

这篇会是很好的过渡参考。

### 4.2 Fast autonomous exploration with sparse topological graphs, 2023

**论文**

- *Fast autonomous exploration with sparse topological graphs in large-scale environments*, International Journal of Intelligent Robotics and Applications, 2023  
  链接: [Springer](https://link.springer.com/article/10.1007/s41315-023-00318-7)

**核心思想**

这篇关注的是：

- frontier-based / sampling-based exploration 的计算负担
- 大规模场景里，global topological graph 太密会拖慢规划
- 因此要构建 sparse topological graph，提高全局搜索速度

**和我们当前系统的对应点**

这和我们现在的一个现实问题很像：

- frontier 与 GVD 一旦都变多，系统会更容易停在“评估开销”和“局部不可执行”之间
- 如果后面要把 GVD component / frontier cache / global decisions 再做大，必须考虑 graph sparsification

**为什么这篇重要**

它提醒我们：

**topological information 越多不一定越好，关键是如何稀疏化、结构化。**

## 5. 把这些论文和我们当前系统一一对照

下面给一个更直接的映射。

| 当前系统部件 | 最相关文献 | 对应关系 |
|---|---|---|
| Frontier 检测与“nearest accessible frontier” | Yamauchi 1997 | 经典 frontier-based exploration 起点 |
| GVD 作为导航骨架 | Choset & Burdick 2000 | 先上 GVD，再沿 GVD，最后离开 GVD 到目标 |
| Frontier + GVD 的混合结构 | Gao et al. 2020 | hybrid metric-topological exploration |
| GVD 参与 frontier heuristic/assignment | Chen et al. 2023 | frontier selection 与 GVD 耦合 |
| Frontier 的可达性/安全性判定 | Senarathne et al. 2015 | “frontier 存在”不等于“frontier 当前可执行” |
| 历史 frontier 缓存、图式管理 | Kim et al. 2020 | 从 frontier list 走向 frontier graph |
| 拓扑地图 + frontier classification | Gomez et al. 2019 | frontier 不只是点，还能带更高层语义 |

## 6. 和当前实现最接近的学术定位

如果要给当前 `auto_explore_sim` 的探索系统找一个比较准确的学术定位，我会这样描述：

### 6.1 它不只是 frontier-based exploration

因为当前系统不是“选一个 frontier 就直接去”，而是：

- frontier 先经过 GVD 吸附
- 再要求 same-component GVD route
- 再考虑是否允许 direct fallback

### 6.2 它也不只是 topological exploration

因为当前全局探索方向仍然主要由 frontier 决定，而不是单纯在 topological graph 上做 coverage。

### 6.3 它最像 hybrid frontier-topological exploration

更准确地说，是：

**frontier-guided, GVD-constrained exploration**

也就是：

- frontier 决定“往哪边探索”
- GVD 决定“通过哪条结构化通道去执行”

## 7. 当前系统相对文献的几个显著特点

这部分更重要，因为它说明“我们现在不是简单复现别人”。

### 7.1 更强调 same-component GVD constraint

很多文献说的是“利用 GVD / topological graph”，但没有把：

- frontier 只能吸到 robot 当前所在 GVD 分量

这件事做得这么硬。

这使得我们当前系统更强地把 GVD 当成“铁路”，但副作用是：

- GVD 一碎，frontier 就容易全不可执行

### 7.2 更强调 frontier state management

当前代码里 frontier 不再只是“当前一轮的检测结果”，而是带有：

- observed
- known cache
- completed
- blacklisted
- locked frontier

这种状态语义。  
这已经比很多简单 frontier planner 更接近“frontier lifecycle management”。

### 7.3 更工程化地考虑 Nav2 execution behavior

文献通常更关注 planning / exploration policy，本仓库则已经深度耦合了：

- `NavigateThroughPoses`
- segment dispatch
- no-progress timeout
- stuck recovery
- deferred replanning between segments

所以我们当前最大的问题，往往不是“理论上要不要去那个 frontier”，而是：

**这条 frontier/GVD 计划在 Nav2 执行层面到底能不能稳地跑出来。**

## 8. 当前系统相对文献的短板

这部分也是后面继续迭代最值得盯的地方。

### 8.1 GVD 连通性仍然太脆

文献里的 topological graph 往往更“抽象”和“稳”，而我们现在直接在栅格 GVD mask 上工作，所以：

- GVD 容易碎
- component 容易过多
- same-component 限制一严，就会让可执行 frontier 大幅减少

### 8.2 frontier 与 GVD 的耦合过硬时，系统容易停住

你最近看到的很多日志，本质上都说明：

- frontier 有
- 但 frontier 当前不在 robot 所在 GVD component 的有效吸附范围里

所以文献里的 “hybrid” 在我们当前实现里，已经有一点向 “GVD dominates execution too strongly” 偏了。

### 8.3 direct fallback 现在还是补丁式出口

在学术上，更漂亮的系统通常会把：

- frontier direction
- topological trunk
- local safe connector

统一建模。  
我们现在的 direct fallback 还是一个工程化补出口，而不是全流程的第一等公民。

## 9. 对我们下一阶段工作的启发

基于这些 related work，我觉得下一阶段最自然的方向有 3 个。

### 9.1 从“frontier list”走向“frontier state graph”

参考：

- Kim 2020

如果后面 frontier 状态越来越复杂，只靠：

- `known_frontiers_`
- `blacklisted_`
- `completed_frontiers_`

这几份松散容器会越来越难维护。  
更好的方向是显式 frontier graph / frontier state machine。

### 9.2 从“栅格 GVD mask”走向“更稳定的 topological graph”

参考：

- Choset 2000
- Gao 2020
- sparse topological graph 2023

如果后面想减少“GVD 太碎导致系统停住”，最根本的不是无限调阈值，而是：

- 把 GVD 从稠密栅格点集抽象成更稳定的 graph

### 9.3 把 direct fallback 统一进 mixed planning framework

参考：

- Gao 2020
- Chen 2023

当前 direct fallback 更像“GVD 失败后的补救”。  
更好的方向是：

- 把它变成系统内置的一种合法 planning mode
- 并用统一代价函数比较：
  - pure GVD
  - GVD + connector
  - direct safe frontier path

## 10. 最后的判断

如果要非常直接地评价：

**你的工作是有明显文献背景支撑的，而且不是边缘方向。**

但它也不是“照着某一篇论文实现”。  
更准确的说法是：

**你现在做的是 frontier exploration、GVD topological navigation、frontier state management 这三条线在 ROS2/Nav2 场景下的一次工程融合。**

其中最像现有论文的部分是：

- frontier + hybrid topological exploration 的大框架

而最有你自己系统特色的部分是：

- same-component GVD snapping
- locked frontier 机制
- frontier 缓存/完成/黑名单分离
- 以 GVD 为执行主轨道的 Nav2 segment dispatch

这也是为什么我们后面继续改的时候，最好不要只问：

- “这篇论文怎么做的？”

而是更具体地问：

- “这篇论文解决的是我们当前系统的哪一个局部问题？”

## 11. 参考链接汇总

- Yamauchi 1997, *A Frontier-Based Approach for Autonomous Exploration*  
  https://biorobotics.ri.cmu.edu/papers/sbp_papers/integrated2/yamauchi_frontier_explor.pdf

- Choset & Burdick 2000, *Sensor-Based Exploration: The Hierarchical Generalized Voronoi Graph*  
  https://www.ri.cmu.edu/publications/sensor-based-exploration-the-hierarchical-generalized-voronoi-graph/

- Senarathne et al. 2015, *Incremental Algorithms for Safe and Reachable Frontier Detection for Robot Exploration*  
  https://www.sciencedirect.com/science/article/pii/S0921889015001232

- Gomez et al. 2019, *Topological Frontier-Based Exploration and Map-Building Using Semantic Information*  
  https://www.mdpi.com/1424-8220/19/20/4595

- Gao et al. 2020, *Self-Exploration in Complex Unknown Environments using Hybrid Map Representation*  
  https://arxiv.org/abs/2004.08535

- Kim et al. 2020, *Graph Search-Based Exploration Method Using a Frontier-Graph Structure for Mobile Robots*  
  https://www.mdpi.com/1424-8220/20/21/6270

- Chen et al. 2023, *GVD-Exploration: An Efficient Autonomous Robot Exploration Framework Based on Fast Generalized Voronoi Diagram Extraction*  
  https://arxiv.org/abs/2309.06041

- *Fast autonomous exploration with sparse topological graphs in large-scale environments*  
  https://link.springer.com/article/10.1007/s41315-023-00318-7
