#include "room_memory/pointcloud2.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <optional>

namespace room_memory {

namespace {

constexpr std::uint8_t kFloat32 = 7;
constexpr std::uint8_t kFloat64 = 8;
constexpr std::uint8_t kUint8 = 2;
constexpr std::uint8_t kUint16 = 4;

std::string hex2(std::uint8_t value) {
    static const char digits[] = "0123456789abcdef";
    return {digits[value >> 4], digits[value & 15]};
}

class Reader {
public:
    explicit Reader(std::span<const std::uint8_t> data) : data_(data) {
        if (data.size() < 4) {
            throw PointCloudError("PointCloud2 CDR: truncated encapsulation header (" + std::to_string(data.size()) +
                                  " of 4 bytes)");
        }
        if (data[0] == 0x00 && data[1] == 0x01) {
            little_ = true;
        } else if (data[0] == 0x00 && data[1] == 0x00) {
            little_ = false;
        } else {
            throw PointCloudError("PointCloud2 CDR: unsupported encapsulation 0x" + hex2(data[0]) + " 0x" + hex2(data[1]) +
                                  " (expected CDR_LE 0x00 0x01 or CDR_BE 0x00 0x00)");
        }
    }

    std::size_t take(std::size_t length, const std::string& what) {
        if (length > data_.size() - std::min(offset_, data_.size())) {
            const std::size_t have = offset_ < data_.size() ? data_.size() - offset_ : 0;
            throw PointCloudError("PointCloud2 CDR: truncated " + what + " at byte " + std::to_string(offset_) +
                                  " (need " + std::to_string(length) + ", have " + std::to_string(have) + ")");
        }
        const std::size_t start = offset_;
        offset_ += length;
        return start;
    }

    std::uint8_t u8(const std::string& what) { return data_[take(1, what)]; }

    std::uint32_t u32(const std::string& what) {
        const std::size_t misalignment = (offset_ - 4) % 4;
        if (misalignment) offset_ += 4 - misalignment;
        const std::size_t start = take(4, what);
        const std::uint8_t* p = data_.data() + start;
        return little_ ? (std::uint32_t{p[0]} | std::uint32_t{p[1]} << 8 | std::uint32_t{p[2]} << 16 |
                          std::uint32_t{p[3]} << 24)
                       : (std::uint32_t{p[3]} | std::uint32_t{p[2]} << 8 | std::uint32_t{p[1]} << 16 |
                          std::uint32_t{p[0]} << 24);
    }

    std::int32_t i32(const std::string& what) { return static_cast<std::int32_t>(u32(what)); }

    // Up to the first NUL; invalid UTF-8 is kept as bytes (Python decodes with replacement).
    std::string string(const std::string& what) {
        const std::uint32_t length = u32(what + " length");
        const std::size_t start = take(length, what);
        const auto* begin = reinterpret_cast<const char*>(data_.data() + start);
        const auto* nul = static_cast<const char*>(std::memchr(begin, 0, length));
        return std::string(begin, nul ? static_cast<std::size_t>(nul - begin) : length);
    }

    const std::uint8_t* bytes() const { return data_.data(); }

private:
    std::span<const std::uint8_t> data_;
    std::size_t offset_ = 4;
    bool little_ = true;
};

struct Field {
    std::uint32_t offset;
    std::uint8_t datatype;
};

std::size_t bounded(const std::string& name, std::uint32_t offset, std::size_t size, std::uint32_t point_step) {
    if (static_cast<std::int64_t>(offset) > static_cast<std::int64_t>(point_step) - static_cast<std::int64_t>(size)) {
        throw PointCloudError("PointCloud2 CDR: field '" + name + "' (offset " + std::to_string(offset) + ", " +
                              std::to_string(size) + " bytes) extends past point_step (" + std::to_string(point_step) +
                              ")");
    }
    return offset;
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(c >= 'A' && c <= 'Z' ? c + 32 : c); });
    return text;
}

}  // namespace

PointCloud decode_pointcloud2(std::span<const std::uint8_t> data) {
    Reader reader(data);
    PointCloud cloud;
    cloud.stamp_sec = reader.i32("header.stamp.sec");
    cloud.stamp_nanosec = reader.u32("header.stamp.nanosec");
    cloud.frame_id = reader.string("header.frame_id");
    const std::uint32_t height = reader.u32("height");
    const std::uint32_t width = reader.u32("width");

    std::optional<Field> fields[4];  // x, y, z, intensity
    const char* names[4] = {"x", "y", "z", "intensity"};
    const std::uint32_t count = reader.u32("fields length");
    for (std::uint32_t index = 0; index < count; ++index) {
        const std::string prefix = "fields[" + std::to_string(index) + "]";
        const std::string name = lower(reader.string(prefix + ".name"));
        const std::uint32_t offset = reader.u32(prefix + ".offset");
        const std::uint8_t datatype = reader.u8(prefix + ".datatype");
        reader.u32(prefix + ".count");
        for (int f = 0; f < 4; ++f) {
            if (name == names[f] && !fields[f]) fields[f] = Field{offset, datatype};
        }
    }

    const bool big_endian = reader.u8("is_bigendian") != 0;
    const std::uint32_t point_step = reader.u32("point_step");
    const std::uint32_t declared_row_step = reader.u32("row_step");
    const std::uint32_t data_length = reader.u32("data length");
    const std::size_t data_start = reader.take(data_length, "data");
    reader.u8("is_dense");

    const std::uint64_t total = std::uint64_t{width} * height;
    if (total == 0) return cloud;
    if (point_step == 0) {
        throw PointCloudError("PointCloud2 CDR: point_step is 0 for a " + std::to_string(width) + "x" +
                              std::to_string(height) + " cloud");
    }
    const std::uint64_t packed_row = std::uint64_t{point_step} * width;
    const std::uint64_t row_step = declared_row_step > 0 ? declared_row_step : packed_row;
    if (packed_row > row_step) {
        throw PointCloudError("PointCloud2 CDR: point_step (" + std::to_string(point_step) + ") x width (" +
                              std::to_string(width) + ") exceeds row_step (" + std::to_string(row_step) + ")");
    }
    // row_step * height > data_length, by division: a packed_row fallback near 2^64 would
    // overflow the product (height > 0 since total > 0).
    if (row_step > data_length / height) {
        throw PointCloudError("PointCloud2 CDR: data holds " + std::to_string(data_length) + " bytes but row_step (" +
                              std::to_string(row_step) + ") x height (" + std::to_string(height) + ") needs more");
    }

    std::size_t offsets[3];
    std::size_t sizes[3];
    for (int f = 0; f < 3; ++f) {
        if (!fields[f]) throw PointCloudError(std::string("PointCloud2 CDR: missing required field '") + names[f] + "'");
        const Field field = *fields[f];
        if (field.datatype != kFloat32 && field.datatype != kFloat64) {
            throw PointCloudError(std::string("PointCloud2 CDR: field '") + names[f] + "' has datatype " +
                                  std::to_string(field.datatype) +
                                  "; only FLOAT32 (7) and FLOAT64 (8) are supported");
        }
        sizes[f] = field.datatype == kFloat32 ? 4 : 8;
        offsets[f] = bounded(names[f], field.offset, sizes[f], point_step);
    }
    if (fields[3]) {
        const std::uint8_t type = fields[3]->datatype;
        const std::size_t size = type == kFloat32 ? 4 : type == kFloat64 ? 8 : type == kUint8 ? 1 : type == kUint16 ? 2 : 0;
        if (size) bounded("intensity", fields[3]->offset, size, point_step);
    }

    // FLOAT64 rounds to binary32 once, as numpy's astype(float32) does.
    const auto read = [&](const std::uint8_t* p, std::size_t size) -> float {
        std::uint64_t bits = 0;
        for (std::size_t b = 0; b < size; ++b) bits |= std::uint64_t{p[b]} << (8 * (big_endian ? size - 1 - b : b));
        if (size == 4) return std::bit_cast<float>(static_cast<std::uint32_t>(bits));
        return static_cast<float>(std::bit_cast<double>(bits));
    };
    cloud.positions.reserve(static_cast<std::size_t>(total) * 3);
    const std::uint8_t* base = reader.bytes() + data_start;
    for (std::uint32_t row = 0; row < height; ++row) {
        const std::uint8_t* row_start = base + static_cast<std::size_t>(row) * row_step;
        for (std::uint32_t column = 0; column < width; ++column) {
            const std::uint8_t* record = row_start + static_cast<std::size_t>(column) * point_step;
            const float x = read(record + offsets[0], sizes[0]);
            const float y = read(record + offsets[1], sizes[1]);
            const float z = read(record + offsets[2], sizes[2]);
            if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) continue;
            cloud.positions.push_back(x);
            cloud.positions.push_back(y);
            cloud.positions.push_back(z);
        }
    }
    return cloud;
}

std::vector<std::uint8_t> encode_pointcloud2(std::span<const float> xyz, const std::string& frame_id) {
    std::vector<std::uint8_t> out = {0x00, 0x01, 0x00, 0x00};
    const auto align = [&](std::size_t size) {
        while ((out.size() - 4) % size) out.push_back(0);
    };
    const auto u32 = [&](std::uint32_t value) {
        align(4);
        for (int b = 0; b < 4; ++b) out.push_back(static_cast<std::uint8_t>(value >> (8 * b)));
    };
    const auto string = [&](const std::string& text) {
        u32(static_cast<std::uint32_t>(text.size() + 1));
        out.insert(out.end(), text.begin(), text.end());
        out.push_back(0);
    };
    const std::size_t points = xyz.size() / 3;
    u32(0);
    u32(0);
    string(frame_id);
    u32(1);
    u32(static_cast<std::uint32_t>(points));
    u32(4);
    const char* names[4] = {"x", "y", "z", "intensity"};
    for (std::uint32_t f = 0; f < 4; ++f) {
        string(names[f]);
        u32(f * 4);
        out.push_back(kFloat32);
        u32(1);
    }
    out.push_back(0);
    u32(16);
    u32(static_cast<std::uint32_t>(16 * points));
    u32(static_cast<std::uint32_t>(16 * points));
    const std::size_t start = out.size();
    out.resize(start + 16 * points, 0);
    for (std::size_t n = 0; n < points; ++n) {
        for (int a = 0; a < 3; ++a) {
            const std::uint32_t bits = std::bit_cast<std::uint32_t>(xyz[3 * n + a]);
            for (int b = 0; b < 4; ++b) out[start + 16 * n + 4 * a + b] = static_cast<std::uint8_t>(bits >> (8 * b));
        }
    }
    out.push_back(1);
    return out;
}

}  // namespace room_memory
