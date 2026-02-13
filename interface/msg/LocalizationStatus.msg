# 定位系统健康状态
# 由 localizer_node 以 1Hz 发布
# 行为树 / 导航系统据此决定是否需要重定位

builtin_interfaces/Time stamp

# 状态枚举
uint8 STATUS_OK = 0           # ICP 正常工作
uint8 STATUS_DEGRADED = 1     # ICP 间歇失败，但尚可恢复
uint8 STATUS_LOST = 2         # 连续 ICP 失败，定位丢失
uint8 STATUS_UNINITIALIZED = 3 # 尚未初始化 (无地图/未 relocalize)
uint8 status

# 定量指标
uint32 icp_consecutive_fails    # 当前连续 ICP 失败次数
float64 seconds_since_last_icp  # 距上次成功 ICP 的秒数
float64 last_icp_fitness         # 最近一次 ICP 的 refine fitness score
bool has_valid_offset            # localizer 是否有有效的 offset
