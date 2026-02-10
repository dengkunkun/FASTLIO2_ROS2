#pragma once
#include "ieskf.h"
#include "commons.h"

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

    /**
     * @brief 重置 IMU 处理器状态
     * @param reuse_bias 是否保留当前的 ba/bg 估计值
     * @details 清空 IMU 缓存，重置时间戳，可选保留偏置估计
     */
    void reset(bool reuse_bias = false);

private:
    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    double m_last_propagate_end_time;
    Vec<IMUData> m_imu_cache;
    Vec<Pose> m_poses_cache;
    V3D m_last_acc;
    V3D m_last_gyro;
    M12D m_Q;
    IMUData m_last_imu;
};