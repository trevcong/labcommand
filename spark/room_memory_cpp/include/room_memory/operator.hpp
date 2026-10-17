// Operator actions on the room memory (protocol 3.6, D46): the durable writes they need and the
// suppressions file. The engine side is RoomMemory::operate and its hooks (src/operator.cpp).
#pragma once

#include <filesystem>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "room_memory/memory.hpp"

namespace room_memory {

// A durable write that installed nothing: the old file (or none) is still what a reader sees.
class StorageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The steps of write_staged, for fault injection in tests.
enum class WriteStage { write, sync, rename, directory };
// Called before each step; returning true makes that step fail. Empty: no faults. Tests only.
using WriteFault = std::function<bool(WriteStage)>;
void set_write_fault(WriteFault fault);

// Temp file, fsync, rename, then fsync the directory, like write_atomic, but it says how far it
// got. Throws StorageError when it fails before the rename (the temp file is removed). Returns
// false when the file is installed but the directory sync failed: every reader, a restart
// included, sees the new file; only a power loss may still lose it. Creates missing parents.
bool write_staged(const std::filesystem::path& path, std::span<const std::uint8_t> data);

// <state_dir>/<room_id>/suppressed-<sensor>: one file per sensor, like its reference, so a sensor
// subset run touches only its own. Same room_id rule as reference_path.
std::filesystem::path suppressions_path(const std::string& state_dir, const std::string& room_id,
                                        const std::string& sensor);
// The file: "room_memory suppressed 1", then one line per suppression in event id order:
// "<event_id> <kind> <n> <key>... <m> <key>..." (keys as decimal packed int64, sorted; m is 0
// unless moved). `cleared` is not saved.
std::string format_suppressions(const std::vector<Suppression>& list);
// Throws std::invalid_argument when the text is not exactly what format_suppressions writes for
// some list (event ids increasing, keys sorted and unique, a pose for each kind).
std::vector<Suppression> parse_suppressions(const std::string& text);

}  // namespace room_memory
