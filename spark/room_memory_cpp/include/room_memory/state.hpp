// The rig guard's files in <state_dir>/<room_id>/, next to the references: the C++ port of
// room_memory/state.py, byte for byte (either engine loads the other's files).
//
// - registration-<sensor>.bin: the sensor's registration cloud. Magic "LCRG", u32 version 1, the
//   sensor name (u32 length, bytes), u64 frame_epoch, u32 count, count x 3 float32, all little
//   endian, then the zlib CRC-32 of everything before it.
// - rig_state: ASCII, one item per line, every sensor's pose and baseline, the gravity baseline and
//   the name, size and CRC-32 of each registration file, then the CRC-32 of the lines above. It is
//   written last, so a crash between files leaves the old rig_state naming the old files, and a
//   file that does not match it fails the load.
// - prev/<seq>-<name>/: archived copies, the newest kArchiveKeep kept.
//
// Each file is written atomically (temp file, fsync, rename). The set is bound by rig_state's sizes
// and CRCs; it is not one transaction (PLAN-RIGGUARD.md, "Later": a transactional manifest).
#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include "room_memory/guard.hpp"

namespace room_memory {

inline constexpr std::uint32_t kRegistrationVersion = 1;
inline constexpr const char* kRigStateHeader = "lcrm-rig-state 1";
inline constexpr std::size_t kArchiveKeep = 3;

std::string registration_name(const std::string& sensor);
// The file's bytes; `points` holds 3 floats per point.
std::vector<std::uint8_t> registration_bytes(const std::string& sensor, std::uint64_t frame_epoch,
                                             std::span<const float> points);
struct Registration {
    std::string sensor;
    std::uint64_t frame_epoch = 0;
    std::vector<float> points;
};
// Throws std::invalid_argument ("<what>: <reason>") on any malformed byte.
Registration parse_registration(std::span<const std::uint8_t> data, const std::string& what);

struct RegistrationFile {
    std::string name;
    std::uint64_t size = 0;
    std::uint32_t crc = 0;
    friend bool operator==(const RegistrationFile&, const RegistrationFile&) = default;
};

struct SensorState {
    rig::Pose pose;
    std::optional<double> baseline;
    RegistrationFile registration;
};

struct RigState {
    std::string room_id;
    double voxel_size = 0.0;
    std::uint64_t frame_epoch = 0;
    std::int64_t n_realigned = 0;
    std::optional<rig::Vec3d> g_base;
    // In the order they are written (the engine's channel order); parse keeps the file's order.
    std::vector<std::pair<std::string, SensorState>> sensors;
};

std::vector<std::uint8_t> format_rig_state(const RigState& state);
// Throws std::invalid_argument ("<what>: <reason>") on a CRC mismatch or any malformed line.
RigState parse_rig_state(std::span<const std::uint8_t> data, const std::string& what);

// Temp file (<path>.tmp), fsync, rename, then fsync the directory, so the rename survives a crash. Creates
// missing parent directories only with make_parents. Throws std::runtime_error (or
// std::filesystem::filesystem_error) on any failure.
void write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data, bool make_parents = true);
// Copies `files` (those that exist) into room/prev/<seq>-<name>/, seq one past the highest archive's
// (four digits), then deletes `remove`; keeps the kArchiveKeep newest archives. Returns the archive.
std::filesystem::path archive(const std::filesystem::path& room, const std::string& name,
                              const std::vector<std::string>& files, const std::vector<std::string>& remove = {});

}  // namespace room_memory
