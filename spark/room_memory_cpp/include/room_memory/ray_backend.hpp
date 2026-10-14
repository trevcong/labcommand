// The seam for ray-evidence backends. The CPU backend (OpenMP over rays) is the only one;
// an accelerator backend implements RayBackend and is passed to cast_rays / DiffOptions.
#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "room_memory/keys.hpp"

namespace room_memory {

// Everything a backend needs to walk the spec DDA for one batch of rays.
struct RayBatch {
    std::span<const Key> end_keys;    // endpoint voxel of each ray
    std::span<const float> endpoints;  // 3 floats per ray: the representative return, meters
    std::array<float, 3> q_origin{};   // origin * inverse, binary32, in voxels
    std::array<std::int64_t, 3> start{};  // floor(q_origin): the start cell
    float inverse = 0.0f;              // f32(1) / voxel_size
};

class RayBackend {
public:
    virtual ~RayBackend() = default;
    // Sorted unique keys of every cell some ray marks; with `keep` (sorted), only marks inside
    // it. The result must be exactly the spec DDA's, independent of scheduling. The caller
    // removes occupied cells.
    virtual Keys marks(const RayBatch& batch, const Keys* keep) const = 0;
};

const RayBackend& cpu_ray_backend();

}  // namespace room_memory
