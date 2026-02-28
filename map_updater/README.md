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

### VoxelHashMap (过时点检测)
基于 3D-DDA (Amanatides & Woo) 射线遍历算法的体素哈希表。每个体素记录被射线"穿越"但未命中的次数 (miss_count)，超过阈值则认为该区域物体已被移走，从地图中删除对应点。

默认关闭 (`enable_point_removal: false`)，测试稳定后再启用。

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `initial_pcd_path` | `""` | 初始 PCD 地图路径，空则从零开始 |
| `map_frame` | `"map_camera_init"` | 地图坐标系 |
| `scan_topic` | `"/livox/lidar_base_link_filtered"` | LiDAR 输入 topic |
| `tf_timeout_s` | `0.2` | TF 查询超时 |
| `scan_process_interval_s` | `0.5` | 最小处理间隔，控制 CPU 占用 |
| `map_resolution` | `0.2` | 地图体素大小 (m)，越小越精确但越占内存 |
| `scan_resolution` | `0.15` | 输入扫描预过滤分辨率 (m) |
| `publish_rate_hz` | `1.0` | 地图发布频率 |
| `enable_point_removal` | `false` | 是否启用过时点射线追踪删除 |
| `miss_count_threshold` | `20` | 穿越次数阈值 |
| `ray_cast_skip` | `10` | 每 N 个点做一次射线追踪 |
| `diag_interval_s` | `10.0` | 诊断日志间隔 |

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
