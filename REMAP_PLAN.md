# FASTLIO2 现状与重新建图方案

## 代码梳理（现有能力）

- `fastlio2/src/lio_node.cpp`：IMU+Livox 点云同步，`MapBuilder` 在 IMU 初始对准后从零开始建局部/全局地图，未暴露重置或加载已有地图的接口。
- `fastlio2/src/map_builder/*`：`LidarProcessor::initCloudMap` 只从首帧点云建 KD-Tree，`IMUProcessor` 初始化时固定使用前 `imu_init_num` 帧估计姿态和 bias，状态由 `IESKF` 管理。
- `localizer/src/localizer_node.cpp` + `localizers/icp_localizer.cpp`：提供 `/localizer/relocalize` 服务，加载已有 PCD 地图并用粗+精 ICP 求解 map→lidar 初始位姿，但结果只保存在本节点（对外仅 TF，不返回位姿）。
- `pgo/src/pgo_node.cpp`：提供 `/pgo/save_maps`，可导出全局地图及分块 patch，方便作为重定位或先验地图。
- Launch/Topic：`lio_node` 以 world_frame（配置中的 `world_frame`）发布 TF/odom/path；`localizer` 发布 map→local TF 和优化后的全局地图点云。

## 可行性判断

- KD-Tree 构建函数已存在，只需支持从 PCD/patch 直接初始化即可将已有地图作为初始地图或背景；`initCloudMap` 接受点集，加载逻辑易于扩展。
- IMU 状态与滤波器可通过重新实例化或添加 reset 方法重新对齐初始位姿，代码无硬编码阻止复位。
- 现有重定位节点已能在任意位置得到 map→lidar 位姿；缺少的是将该位姿回灌给 LIO 并切换到新的建图会话。
- 因此可以在当前架构上增加“重定位->重置->继续建图”闭环，技术上可行；需关注线程同步与帧一致性。

## 实现计划

1) 新增“重新建图”服务（建议放在 `fastlio2`）

   - 新建 `interface/srv/ResetMapping.srv`（示例字段：`string map_pcd_path`、`float64 x/y/z/yaw/pitch/roll`、`bool load_map`、`bool clear_prev_map`、`bool reuse_bias`）。
   - `lio_node` 接收到请求后：暂停 `processLoop`、清空 IMU/Lidar 缓冲，选择性保留上次估计的 `ba/bg`，根据姿态字段直接设定 `IESKF::x().r_wi/t_wi`，并重置 `IMUProcessor` 缓存时间戳。若 `load_map` 为真，读取 PCD 并调用 `LidarProcessor::initCloudMap` 填充 KD-Tree，同时重置 `LocalMap` 状态。恢复线程后继续建图。
2) 重定位结果回灌

   - 扩展 `interface/srv/Relocalize.srv` 响应包含最终位姿和得分，或新增 `GetRelocResult` 服务/话题，让 `lio_node` 能拿到 map→lidar 位姿。
   - `lio_node` 在执行 ResetMapping 前调用重定位服务：用户可传固定初始猜测（固定点重建）或使用默认值（任意位置重建），等待成功后将返回姿态填入 ResetMapping 请求。
3) 帧与地图管理

   - 约定 map_frame 作为全局框架，ResetMapping 时将 `world_frame` 设为 map_frame，`broadCastTF` 使用新的初始位姿发 TF。
   - 若加载旧地图继续增量建图，需记录“旧地图点云所属 frame == map_frame”，避免重复姿态变换。可在服务中增加 `map_in_map_frame` 标志。
4) 稳定性与性能细节

   - 新增线程锁保护 reset 过程，防止 `processLoop` 与回调竞争。
   - 对大地图加载添加体素滤波/分块（可复用 `localizer` 的降采样配置）以控制 KD-Tree 构建时间。
   - 允许配置复用上一段运行中估计的 bias，减少重新收敛时间；若 IMU 静止重新上电则可选重新估计。
5) 测试验证路径

   - 回放已保存 bag：正常建图→保存地图→随机移动起点→调用重定位+ResetMapping→验证 TF/odom 连续且地图正确增长。
   - 测试固定点重建：直接提供预设姿态调用 ResetMapping，不调用重定位，验证能在固定初始姿态下重建。
   - 压测大地图加载时间与内存，确认线程暂停/恢复无死锁。
   - 若使用 PGO/HBA，验证重置后 loop/优化节点仍使用新的 map_frame/TF。
