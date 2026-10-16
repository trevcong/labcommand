// CDR sensor_msgs/PointCloud2 decoding, with the rules of PointCloud2CDR.swift and
// room_memory/core/pointcloud2.py.
#pragma once

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace room_memory {

// Truncated, malformed, or unsupported PointCloud2 bytes.
class PointCloudError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// Finite positions of one decoded frame: 3 floats per point, meters, in the message frame.
struct PointCloud {
    std::vector<float> positions;
    std::string frame_id;
    std::int32_t stamp_sec = 0;
    std::uint32_t stamp_nanosec = 0;

    std::size_t size() const { return positions.size() / 3; }
};

// Decodes a whole message, encapsulation header included. Fields x, y, z (case-insensitive,
// first occurrence) must be FLOAT32 or FLOAT64 (rounded to binary32); points with a
// non-finite coordinate are skipped. Throws PointCloudError.
PointCloud decode_pointcloud2(std::span<const std::uint8_t> data);

// CDR little-endian PointCloud2 with x, y, z, intensity float32 (point_step 16): the test
// scenes' wire format.
std::vector<std::uint8_t> encode_pointcloud2(std::span<const float> xyz, const std::string& frame_id = "test");

}  // namespace room_memory
