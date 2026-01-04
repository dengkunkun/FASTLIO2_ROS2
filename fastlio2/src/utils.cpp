#include <algorithm>

#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    if (!msg)
    {
        return cloud;
    }

    const size_t available_points = msg->points.size();
    if (available_points == 0)
    {
        return cloud;
    }

    const int downsample = std::max(1, filter_num);
    size_t point_num = std::min<size_t>(msg->point_num, available_points);
    constexpr size_t kMaxPoints = 800000;  // livox单帧上限防止异常数据撑爆内存
    if (point_num > kMaxPoints)
    {
        std::cerr << "[Utils] Livox point_num " << point_num << " exceeds cap " << kMaxPoints << ", truncating" << std::endl;
        point_num = kMaxPoints;
    }

    cloud->reserve(point_num / static_cast<size_t>(downsample) + 1);
    for (size_t i = 0; i < point_num; i += static_cast<size_t>(downsample))
    {
        const auto &pt = msg->points[i];
        if ((pt.line < 4) && ((pt.tag & 0x30) == 0x10 || (pt.tag & 0x30) == 0x00))
        {
            float x = pt.x;
            float y = pt.y;
            float z = pt.z;
            float r2 = x * x + y * y + z * z;
            if (r2 < static_cast<float>(min_range * min_range) || r2 > static_cast<float>(max_range * max_range))
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = pt.reflectivity;
            p.curvature = pt.offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
