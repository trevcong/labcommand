// Ray-cast lidar scans of small synthetic rooms for the rig-guard tests: the C++ twin of
// spark/room_memory/tests/rooms.py. A sensor at a pose casts a fixed fan of rays, and each return is
// the first surface it hits, with range noise. Boxes are solid (the sensor sees their outside); the
// room is the inside of its box. Points come back in the sensor's own frame, binary32.
//
// Same geometry, constants and fan as the Python module; numpy's generator cannot be reproduced, so
// the fan offsets and the range noise come from a SplitMix64 stream (Box-Muller normals).
#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "room_memory/guard.hpp"

namespace rooms {

using Box = std::pair<room_memory::rig::Vec3d, room_memory::rig::Vec3d>;  // (low, high)

// Interior of a 8 x 6 x 3 m room with the sensor 1.2 m above the floor, and solid furniture.
extern const Box kRoom;
extern const std::vector<Box> kFurniture;
// A corridor (two parallel walls and a floor, open along x) and a lone floor: degenerate views.
extern const Box kCorridor;
extern const Box kFloorOnly;

// Deterministic uniforms and normals from a seed.
class Random {
public:
    explicit Random(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next();
    double uniform();  // [0, 1)
    double normal();   // mean 0, deviation 1

private:
    std::uint64_t state_;
    std::optional<double> spare_;
};

// Unit ray directions (3 per ray): `rows` elevations from low_deg to high_deg x `columns` azimuths,
// row-major. With `seed` the whole fan is offset by a random fraction of a step on each axis.
std::vector<double> fan(int rows = 32, int columns = 720, double low_deg = -35.0, double high_deg = 30.0,
                        std::optional<std::uint64_t> seed = std::nullopt);

// The returns of a sensor at `pose` (sensor -> room frame), 3 floats per point in the sensor frame.
// A ray without a return within max_range_m gives no point.
std::vector<float> scan(const room_memory::rig::Pose& pose = {}, const Box& room = kRoom,
                        const std::vector<Box>& boxes = kFurniture, double noise_m = 0.01, std::uint64_t seed = 0,
                        double max_range_m = 30.0);

// A rig pose in the room frame: yaw about z, then pitch about y, then roll about x.
room_memory::rig::Pose rig(double dx = 0, double dy = 0, double dz = 0, double yaw_deg = 0, double roll_deg = 0,
                           double pitch_deg = 0);

// (meters, degrees) between two poses.
std::pair<double, double> pose_error(const room_memory::rig::Pose& estimate, const room_memory::rig::Pose& truth);

}  // namespace rooms
