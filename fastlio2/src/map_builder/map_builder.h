#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }    
    std::shared_ptr<LidarProcessor> lidar_processor(){return m_lidar_processor;}
    std::shared_ptr<IMUProcessor> imu_processor(){return m_imu_processor;}

    /// 设置是否冻结地图（定位模式时不往ikd-Tree添加新点）
    void setFreezeMap(bool freeze) { m_lidar_processor->setFreezeMap(freeze); }
    bool isFreezeMap() const { return m_lidar_processor->isFreezeMap(); }

    /// 获取 ikd-Tree 有效点数量（诊断用）
    int getIkdTreeSize() const { return m_lidar_processor->getIkdTreeSize(); }

private:
    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IMUProcessor> m_imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
};
