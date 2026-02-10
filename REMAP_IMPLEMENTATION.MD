# FASTLIO2 在已有地图上继续建图（方案1：map_frame 为全局世界）

> **分支**: `feature/remap_on_exist_map`
>
> - 2026-01-13：初版实现 reset_mapping + PGO load_session
> - 2026-01-14：**修复关键缺陷**——加载 map.pcd 时正确执行 `map_frame → world_frame` 坐标变换；加固 reset 并发安全（`m_builder_mutex`）

---

## 核心改动摘要

| 问题 | 修复 |
|------|------|
| **加载 PCD 坐标系错位** | `resetMappingCB` 在查到 `TF(world←map)` 后，同时把 `r_w_m/t_w_m` 填入 `ResetRequest`，`LidarProcessor::loadMapFromPCD()` 会把 map_frame 点云变换到 world_frame 再建 KD-tree |
| **reset 并发崩溃/状态撕裂** | 新增 `m_builder_mutex`，`processOnce()` 与 `resetMappingCB()` 互斥；`syncPackage()` 内部锁 `imu_mutex/lidar_mutex` 避免订阅回调与处理线程数据竞争 |
| **TF 不可用时 fallback 导致抖动** | 当 `map_frame != world_frame` 时，TF 查不到直接返回失败，不再 fallback（避免 localizer 与 lio_node 坐标系不一致） |

---

## 文件变更清单

```
fastlio2/src/lio_node.cpp          # 主要改动
fastlio2/src/map_builder/map_builder.h/.cpp
fastlio2/src/map_builder/lidar_processor.h/.cpp
fastlio2/src/map_builder/imu_processor.h/.cpp
interface/srv/ResetMapping.srv
interface/srv/LoadPGOSession.srv
pgo/src/pgo_node.cpp
pgo/src/pgos/simple_pgo.h/.cpp
```

---

## ⚠️ 重要：ResetMapping pose 的 frame 语义

- `/fastlio2/reset_mapping` 的 `x/y/z/roll/pitch/yaw` 始终表示 **IMU/body（即 `body_frame`）的位姿**，不要填 `base_link`。
- 本项目默认 `world_frame=camera_init`、`body_frame=body`、`map_frame=map_camera_init`（见 [fastlio2/config/lio.yaml](fastlio2/config/lio.yaml)）。

---

## 方案1 架构：TF 链与坐标系

```
              PGO / localizer 发布
                     ↓
    map_camera_init ──────────────────► camera_init ──────────────────► body
         (map_frame)    TF: map→local      (world_frame)   TF: world→body    (body_frame)
                                                               ↑
                                                         FASTLIO2 发布
```

- **map.pcd** 由 `/pgo/save_maps` 输出，坐标系为 **map_frame (map_camera_init)**
- **FASTLIO2 内部里程计** 在 **world_frame (camera_init)** 下运行
- 加载 map.pcd 时，`LidarProcessor::loadMapFromPCD()` 会用 `T_world_map` 把点云从 map_frame 变换到 world_frame

---

## ✅ 2026-01-14 修复：加载地图坐标变换 + 线程安全

### 1. 加载 PCD 坐标变换（关键修复）

之前只把请求位姿从 `map_frame` 转到 `world_frame`，但 **没有把 map.pcd 点云也做变换**，导致 KD-tree 里的地图点与 IESKF 状态不在同一坐标系。

现在 `resetMappingCB()` 在查到 `TF(world←map)` 后，同时把变换矩阵填入 `ResetRequest`：

```cpp
reset_req.has_map_to_world_tf = has_world_from_map_tf;
reset_req.r_w_m = r_w_m;
reset_req.t_w_m = t_w_m;
```

`LidarProcessor::loadMapFromPCD()` 会据此变换每个点和法向量。

### 2. 线程安全设计

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           lio_node                                      │
├─────────────────────────────────────────────────────────────────────────┤
│  imuCB()  ──► imu_mutex ──► imu_buffer                                  │
│  lidarCB() ──► lidar_mutex ──► lidar_buffer                             │
│                                                                         │
│  processLoop() ──► reset_pending check ──► processOnce()                │
│                                               │                         │
│                                    ┌──────────┴──────────┐              │
│                                    │   m_builder_mutex   │   ← 新增     │
│                                    └──────────┬──────────┘              │
│                                               │                         │
│                                    syncPackage() + m_builder->process() │
│                                                                         │
│  resetMappingCB() ──► reset_pending=true ──► m_builder_mutex.lock()     │
│                       ──► clear buffers ──► m_builder->reset()          │
│                       ──► reset_pending=false                           │
└─────────────────────────────────────────────────────────────────────────┘
```

关键互斥点：
- `m_builder_mutex`：`processOnce()` 与 `resetMappingCB()` 互斥，避免 KD-tree 重建与 scan matching 并发
- `syncPackage()` 内部锁 `imu_mutex` + `lidar_mutex`：避免订阅回调与处理线程数据竞争
- 移除不可靠的 `sleep_for(100ms)` 等待

### 3. TF 不可用时直接失败

当 `map_frame != world_frame` 时，TF 查不到直接返回失败，不再 fallback。

---

## 使用示例

```sh
# 固定点重建（从零开始）
ros2 service call /fastlio2/reset_mapping interface/srv/ResetMapping \
  '{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, load_map: false, clear_path: true}'

# 在已有地图上继续建图（方案1：需要 TF map→local 可用）
ros2 service call /fastlio2/reset_mapping interface/srv/ResetMapping \
  '{x: -8.901, y: 6.569, z: 0.101, roll: 0.0, pitch: 0.0, yaw: 1.530, 
   load_map: true, map_pcd_path: "/home/firebot/firebot_dragon/maps/test_site_1/map_optimized.pcd",
   reuse_bias: true, clear_path: false}'
```

### 只有 base_link 时的转换

如果你手上只有 `map -> base_link`（例如 RViz 的 2D Pose Estimate），需要通过静态外参转换成 `map -> body`：

$$T_{map\leftarrow body} = T_{map\leftarrow base} \cdot T_{base\leftarrow body}$$

---

# ✅ PGO 历史会话加载（在已有地图上继续增量 & 可继续优化）

## 背景

`/pgo/save_maps` 默认只会保存 **当前 PGO 节点运行期间**累计的 keyposes。

当你 reset 或重启 PGO 后，如果想在已有 `maps/test_site_x` 的基础上继续：

- 追加新的 patches
- 输出新的 map.pcd
- 继续做回环优化

就需要在 PGO 启动后先把历史 `patches + poses.txt` 加载进 PGO。

## 实现内容

1. 新增服务定义：`interface/srv/LoadPGOSession.srv`
2. PGO 新增服务端点：`/pgo/load_session`
3. `SimplePGO` 新增 API：
   - `appendPriorKeyPose(...)`
   - `commitAppendedPriors()`

加载策略：

- 读取 `${maps_path}/poses.txt`（格式：`patch_name tx ty tz qw qx qy qz`）
- 逐条加载 `${maps_path}/patches/${patch_name}`
- 将每个 patch 作为历史 keypose 灌入因子图并提交到 ISAM2

## 使用方法（推荐流程）

### 1) 定位模式下：仅启动 lio_node + localizer_node

通过重定位/TF 得到 `map_camera_init -> body`，然后调用 reset：

```bash
ros2 service call /fastlio2/reset_mapping interface/srv/ResetMapping \
  "{x: -9.333, y: 6.030, z: -0.183, roll: 0.034, pitch: 0.538, yaw: -1.702, \
    load_map: true, map_pcd_path: '/home/firebot/firebot_dragon/maps/test_site_1/map_optimized.pcd', \
    reuse_bias: true, clear_path: false}"
```

### 2) 后续启动 PGO，并加载历史会话

启动：

```bash
ros2 launch pgo pgo_launch.py remap_mode:=true     
```

加载历史 session：

```bash
ros2 service call /pgo/load_session interface/srv/LoadPGOSession \
  "{maps_path: '/home/firebot/firebot_dragon/maps/test_site_2', load_patches: true}"
```

此时 PGO 内部已经包含旧的 patches/keyposes，后续收到新的点云+里程计会继续追加并做回环优化。

### 3) 保存（会输出全量：旧 + 新）

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/firebot/firebot_dragon/maps/fastlio_remap_on_2', save_patches: true}"
```

输出：

- `map.pcd`
- `patches/`（从 0 开始重新编号，内容为"全量 keyposes"对应的 patches）
- `poses.txt`

---

## 后续测试建议

1. **回放 bag 测试**: 正常建图 → 保存地图 → 调用 `/reset_mapping` → 验证 TF/odom 连续
2. **固定点重建**: 直接指定位姿，不加载地图
3. **大地图加载压测**: 验证 KD-Tree 构建时间和内存占用
4. **增量 + 优化闭环**（推荐）：
   - 启动 lio+localizer 定位 → `/fastlio2/reset_mapping(load_map=true)`
   - 启动 pgo → `/pgo/load_session(maps_path=..., load_patches=true)`
   - 继续运行采集 → `/pgo/save_maps(save_patches=true)`
   - 用输出的 `map.pcd`/`patches` 继续给 localizer 或 hba 使用

---

## ⚠️ 已知限制：PGO 跨 session 续跑

当前 `/pgo/load_session` 假设 **local_frame 连续**（即 FASTLIO2 没有重启）。如果重启了 FASTLIO2 使得 `camera_init` 坐标系与历史不同，需要额外机制把当前 `map→local` offset 对齐到历史 session，否则因子图约束会错位。

后续可考虑：
- 新增 `/pgo/set_offset` 服务注入当前 map→local 变换
- 或在 load_session 后用当前 TF 覆盖 offset
