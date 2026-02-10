#include "icp_localizer.h"

ICPLocalizer::ICPLocalizer(const ICPConfig &config, rclcpp::Logger logger) 
: m_config(config), m_logger(logger)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        RCLCPP_ERROR(m_logger, "Map file not found: %s", path.c_str());
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0)
    {
        RCLCPP_ERROR(m_logger, "[ICP] Map not loaded! refine_tgt: %zu, rough_tgt: %zu", 
                  m_refine_tgt->size(), m_rough_tgt->size());
        return false;
    }
    
    RCLCPP_DEBUG(m_logger, "[ICP] Input cloud - rough: %zu, refine: %zu", 
              m_rough_inp->size(), m_refine_inp->size());
    RCLCPP_DEBUG(m_logger, "[ICP] Target map - rough: %zu, refine: %zu", 
              m_rough_tgt->size(), m_refine_tgt->size());
    RCLCPP_DEBUG(m_logger, "[ICP] Initial guess: t=(%.3f, %.3f, %.3f)", 
              guess(0,3), guess(1,3), guess(2,3));
    
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    
    m_last_rough_fitness = m_rough_icp.getFitnessScore();
    m_last_refine_fitness = 0.0;  // 重置，以防 rough 失败时不会保留旧值
    
    RCLCPP_INFO(m_logger, "[ICP] Rough: converged=%d fitness=%.4f (thresh=%.3f) iters=%d inp=%zu tgt=%zu", 
              m_rough_icp.hasConverged(), m_last_rough_fitness, m_config.rough_score_thresh,
              m_config.rough_max_iteration, m_rough_inp->size(), m_rough_tgt->size());
    
    if (!m_rough_icp.hasConverged() || m_last_rough_fitness > m_config.rough_score_thresh)
    {
        RCLCPP_WARN(m_logger, "[ICP] Rough ICP FAILED! fitness=%.4f > thresh=%.3f",
                 m_last_rough_fitness, m_config.rough_score_thresh);
        return false;
    }
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    
    m_last_refine_fitness = m_refine_icp.getFitnessScore();
    
    RCLCPP_INFO(m_logger, "[ICP] Refine: converged=%d fitness=%.4f (thresh=%.3f) iters=%d inp=%zu tgt=%zu", 
              m_refine_icp.hasConverged(), m_last_refine_fitness, m_config.refine_score_thresh,
              m_config.refine_max_iteration, m_refine_inp->size(), m_refine_tgt->size());
    
    if (!m_refine_icp.hasConverged() || m_last_refine_fitness > m_config.refine_score_thresh)
    {
        RCLCPP_WARN(m_logger, "[ICP] Refine ICP FAILED! fitness=%.4f > thresh=%.3f",
                 m_last_refine_fitness, m_config.refine_score_thresh);
        return false;
    }
    guess = m_refine_icp.getFinalTransformation();
    
    // 计算 ICP 修正量（initial guess 到 final 的距离）
    Eigen::Matrix3f R = guess.block<3, 3>(0, 0);
    float pitch = std::asin(-R(2, 0));
    float yaw = std::atan2(R(1, 0), R(0, 0));
    float roll = std::atan2(R(2, 1), R(2, 2));
    
    RCLCPP_INFO(m_logger, "[ICP] SUCCESS! t=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)deg "
             "rough=%.4f refine=%.4f", 
             guess(0,3), guess(1,3), guess(2,3),
             roll * 180.0f / M_PI, pitch * 180.0f / M_PI, yaw * 180.0f / M_PI,
             m_last_rough_fitness, m_last_refine_fitness);
    return true;
}