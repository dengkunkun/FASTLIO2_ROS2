# MapUpdater

增量全局点云地图更新节点。加载初始 PCD 地图，订阅 LiDAR 扫描，增量添加新观测到的点，可选删除过时点，并定期发布更新后的点云地图供 fire_tracker 使用。

## 架构

```
/livox/lidar_base_link_filtered ──┐
                                  ▼
                         ┌──────────────────┐
            TF ────────▶ │ MapUpdaterNode    │
  (map_camera_init       │                  │
   → base_link)          │  onLidarScan()   │ ──▶ processLoop()
                         │   坐标变换+入队   │       ├─ addPoints (ikd-tree)
                         │                  │       ├─ rayCastAndRemove (可选)
                         │                  │       └─ publishUpdatedMap
                         └──────────────────┘
                                  │
                                  ▼
                    /map_updater/map_updater_node/updated_map
                                  │
                                  ▼
                         ┌──────────────────┐
                         │  fire_tracker    │
                         │  ray_pcd_localizer│
                         └──────────────────┘
```

## 核心组件

### ikd-tree (增量KD树)
从 FASTLIO2 复制的数据结构，支持增量添加/删除点，内建体素降采样。模板实例化为 `pcl::PointXYZI`。

### VoxelHashMap (过时点检测 — Log-odds 概率占据模型)
基于 3D-DDA (Amanatides & Woo) 射线遍历算法的体素哈希表，采用 OctoMap 风格的 log-odds 概率占据模型。

每个 0.05m 体素维护一个 `log_odds` 置信值（正=占据，负=空闲）：
- **命中 (hit)**: 射线端点所在体素 `+= log_odds_hit (+0.847)`，约为 miss 的 2 倍强度
- **穿越 (miss)**: 射线路径中间体素 `-= log_odds_miss (-0.405)`，每帧每体素最多 1 次（per-scan 去重）
- **删除**: `log_odds < free_threshold (-1.0)` 时认为该区域是空闲的，从 ikd-tree 中删除对应点

关键保护机制：
1. **不对称更新**: hit >> miss，稳定墙面天然抵抗误删
2. **Per-scan 去重**: 同一体素每帧最多 1 次 miss，防止多射线在一帧内压倒一个体素
3. **Clamping**: log-odds 限制在 `[clamp_min, clamp_max]`，防止无限累积
4. **26-邻域 hit 传播**: 端点命中体素时，同时向 26 个相邻体素（3×3×3 立方体）注入 hit 证据。因 scan_resolution(0.15m) > voxel_size(0.05m)，端点之间存在 ~3 个体素间隙永远不会被直接命中；邻域传播确保这些间隙体素也获得正向证据，不会被 miss 射线侵蚀。仅更新已有体素，不创建幻象体素。
5. **端点保护球**: 射线在端点附近提前停止，避免 VoxelGrid 质心偏移导致误 miss（二级防线，与 26-邻域互补）

### addPoints 密度保护
`addPoints` 在添加扫描点前，逐点检查 ikd-tree 中 `map_resolution/2` 范围内是否已有地图点。仅向**空白区域**添加新点（`Add_Points(..., false)`），防止 ikd-tree 的体素降采样破坏原始地图密度。

**与 FASTLIO2 的对比：**

| | FASTLIO2 `incrCloudMap()` | map_updater `addPoints()` |
|---|---|---|
| 空白区域 | 添加 (`downsample=false`) | 添加 (`downsample=false`) ✓ |
| 已有覆盖，新点更优 | 替换 (`downsample=true`) | **跳过** |
| 已有覆盖，旧点更优 | 跳过 | **跳过** |

差异原因：FASTLIO2 主循环的目标是持续优化地图质量（选更接近体素中心的点），map_updater 的目标是**保持已建图密度**，仅扩展新区域。因此 map_updater 不做"更优点替换"。副作用：被射线误删的点不会被新扫描恢复。

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `initial_pcd_path` | `""` | 初始 PCD 地图路径，空则从零开始 |
| `map_frame` | `"map_camera_init"` | 地图坐标系 |
| `scan_topic` | `"/fastlio2/world_cloud"` | LiDAR 输入 topic (FASTLIO2 世界坐标系点云) |
| `odom_topic` | `"/fastlio2/lio_odom"` | 里程计 topic (射线追踪起点) |
| `scan_frame` | `"camera_init"` | scan/odom 坐标系 |
| `scan_process_interval_s` | `0.5` | 最小处理间隔，控制 CPU 占用 |
| `map_resolution` | `0.2` | ikd-tree 体素大小 (m) |
| `scan_resolution` | `0.15` | 输入扫描预过滤分辨率 (m) |
| `removal_voxel_resolution` | `0.05` | VoxelHashMap 体素分辨率 (m) |
| `publish_rate_hz` | `1.0` | 地图发布频率 |
| `enable_point_removal` | `true` | 是否启用过时点射线追踪删除 |
| `ray_cast_skip` | `1` | 每 N 个点做一次射线追踪 |
| `log_odds_hit` | `0.847` | 每次命中的占据证据 (logodds(0.7)) |
| `log_odds_miss` | `0.405` | 每次穿越的空闲证据 |
| `log_odds_clamp_max` | `3.5` | 最大占据置信 |
| `log_odds_clamp_min` | `-2.0` | 最大空闲置信 |
| `log_odds_free_threshold` | `-1.0` | 删除阈值 |
| `log_odds_initial` | `2.0` | 新建体素初始置信值 |
| `endpoint_protection_base_m` | `0.05` | 端点保护球基础半径 (m) |
| `endpoint_protection_angle_factor` | `0.02` | 每米距离增加的保护半径 |
| `diag_interval_s` | `10.0` | 诊断日志间隔 |
| `diag_sector_radius_m` | `6.0` | 定向诊断扇区半径 |
| `diag_sector_fov_deg` | `90.0` | 定向诊断扇区视场角 |

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `<scan_topic>` | `sensor_msgs/PointCloud2` | 订阅 | LiDAR 扫描输入 |
| `updated_map` | `sensor_msgs/PointCloud2` | 发布 | 更新后的全局点云地图 |

## 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `save_map` | `interface/srv/SaveMaps` | 保存当前地图到 PCD 文件 |
| `reload_map` | `interface/srv/SaveMaps` | 重新加载 PCD 地图 (替换当前地图) |

## 启动

```bash
# Standalone
ros2 launch map_updater map_updater.launch.py

# 指定初始地图
ros2 launch map_updater map_updater.launch.py
# (修改 config/map_updater.yaml 中的 initial_pcd_path)

# 运行时保存地图
ros2 service call /map_updater/map_updater_node/save_map interface/srv/SaveMaps \
  "{file_path: '/tmp/updated_map.pcd', save_patches: false}"

# 运行时重载地图
ros2 service call /map_updater/map_updater_node/reload_map interface/srv/SaveMaps \
  "{file_path: '/tmp/new_map.pcd', save_patches: false}"
```

## 编译

```bash
cd ~/firebot_dragon
colcon build --packages-select map_updater \
  --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

## 线程模型

- **回调线程**: `onLidarScan()` — TF 查询、PCL 转换、坐标变换、入队
- **处理线程**: `processLoop()` — 消费队列、ikd-tree 操作、发布地图
- **ikd-tree 内部线程**: 后台重平衡 (pthread，由 ikd_Tree 自身管理)

所有 ikd-tree 和 VoxelHashMap 操作仅在处理线程执行，服务回调通过条件变量转发给处理线程，保证线程安全。

## 依赖

- `rclcpp`, `rclcpp_components` — ROS2 节点框架
- `sensor_msgs` — PointCloud2
- `tf2_ros`, `tf2_eigen` — 坐标变换
- `pcl_conversions` — PCL↔ROS 转换
- `interface` — SaveMaps 服务定义 (FASTLIO2_ROS2 共享)
- `PCL` — 点云处理
- `yaml-cpp` — 配置文件加载
- `Eigen3` — 线性代数

# 实际测试
[map_updater.log ](../../../src/Log/map_updater.log) 记录了实际测试过程中的日志输出。测试环境为室内，机器人静止，人工移动障碍物。
机器人不移动，启动命令：ros2 launch map_updater map_updater.launch.py |tee src/Log/map_updater.log
人去移动障碍物，从rviz中可以看到人的移动痕迹和新的障碍物位置

## 已解决的问题

1. **天花板缺失**: scan_topic 使用了过滤后的点云，没有天花板，导致较高的点云没有射线终点即没有被穿过。切换到 world_cloud 解决。
2. **墙壁稀疏化**: 原始整数 miss_count 模型中，一帧多条射线穿过同一体素导致 count 瞬间超阈值。引入 log-odds 概率模型 + per-scan 去重 + 端点保护球解决射线侧问题。
3. **ikd-tree 降采样破坏密度**: `Add_Points(..., downsample=true)` 每个 0.2m 体素只保留 1 点，原始稠密地图被大量删减（占总丢失的 98%）。通过 Nearest_Search 过滤已有覆盖区域的扫描点，仅向空白区域添加新点解决。
