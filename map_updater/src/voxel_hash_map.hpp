#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

struct BoxPointType;

struct VoxelKey
{
    int32_t ix, iy, iz;

    bool operator==(const VoxelKey& o) const
    {
        return ix == o.ix && iy == o.iy && iz == o.iz;
    }
};

struct VoxelKeyHash
{
    size_t operator()(const VoxelKey& k) const
    {
        size_t h = std::hash<int32_t>{}(k.ix);
        h ^= std::hash<int32_t>{}(k.iy) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int32_t>{}(k.iz) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct VoxelData
{
    uint16_t miss_count = 0;
};

/// Per-batch statistics for diagnosing ray cast behavior
struct VoxelStats
{
    int hit_new = 0;           // observeHit: new voxels created
    int hit_existing = 0;      // observeHit: existing voxels found (miss_count decremented)
    int ray_count = 0;         // observeRay calls executed (excluding same-key skips)
    int ray_same_key = 0;      // observeRay calls skipped (start_key == end_key)
    int ray_steps = 0;         // Total intermediate voxel cells visited across all rays
    int ray_incremented = 0;   // Steps where voxel existed in map and was incremented
    int ray_not_in_map = 0;    // Steps where voxel cell had no entry in map
    int rays_productive = 0;   // Rays that incremented at least one voxel
};

/// Histogram of miss_count distribution across all voxels
struct MissHistogram
{
    int bin_0 = 0;        // miss_count == 0
    int bin_1_3 = 0;      // 1 <= miss_count <= 3
    int bin_4_6 = 0;      // 4 <= miss_count <= 6
    int bin_7_9 = 0;      // 7 <= miss_count <= 9
    int bin_ge10 = 0;     // miss_count >= 10
    uint16_t max_miss = 0;
    int total = 0;
};

class VoxelHashMap
{
public:
    explicit VoxelHashMap(float voxel_size);

    /// Register an endpoint hit; creates voxel if not existing, decrements
    /// miss_count by 1 for existing voxels (protects active surfaces)
    void observeHit(float x, float y, float z);

    /// Increment miss_count for all traversed voxels between origin and endpoint
    /// (excluding the endpoint voxel) using 3D-DDA ray traversal
    void observeRay(const Eigen::Vector3f& origin,
                    const Eigen::Vector3f& endpoint);

    /// Return BoxPointType for each voxel exceeding threshold, remove from map
    std::vector<BoxPointType> collectExpired(uint16_t threshold);

    void clear();
    size_t size() const;
    uint16_t maxMissCount() const;

    /// Reset per-batch diagnostic counters (call before each rayCastAndRemove)
    void resetStats();
    /// Get accumulated stats since last resetStats()
    const VoxelStats& getStats() const;
    /// Compute miss_count histogram over all current voxels
    MissHistogram getMissHistogram() const;

private:
    VoxelKey toKey(float x, float y, float z) const;
    BoxPointType keyToBox(const VoxelKey& key) const;

    float voxel_size_;
    float inv_voxel_size_;
    std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels_;
    VoxelStats stats_;
};
