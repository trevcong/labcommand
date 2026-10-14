// room_memory_offline: the C++ twin of `python -m room_memory.offline`.
#pragma once

#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "room_memory/change_detection.hpp"

namespace room_memory::offline {

// Nearest binary32 (ties to even) of a decimal, as Swift Float(text) and the Python
// parse_f32 do; also inf/infinity/nan. Empty when the text is not a number.
std::optional<float> parse_f32(const std::string& text);

struct Invocation {
    Config config;
    bool emit_json = false;
    std::string before;
    std::string after;
};

// change-detect arguments; `evidence` and `emit_json` are the values before any flag.
// Empty on a usage error (unknown flag, missing or malformed value, not exactly two paths).
std::optional<Invocation> parse_args(const std::vector<std::string>& args, bool evidence, bool emit_json);

// Runs the CLI: by default the canonical v2 record plus one newline, byte for byte what
// `change-detect --evidence --json` prints. Returns the exit code.
int run(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace room_memory::offline
