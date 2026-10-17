// room_memory_replay <script>: drives RoomMemory through a window sequence and prints every
// output message as CDR hex, for byte comparison with tests/replay.py (the Python oracle).
//
// Script lines (whitespace-separated, # comments):
//   config key=value ...        MemoryConfig fields; sensors=a,b; max_range=none; state_dir=none
//   start <sec> <nanosec> [generation]
//                               construct RoomMemory; the generation defaults to the start time in
//                               ms (at least 1), the allocator's wall-clock floor
//   points <sensor> <file>      add_points from raw little-endian float32 x, y, z
//   cdr <sensor> <file>         add_cdr with the file's bytes
//   hex <sensor> <hex>          add_cdr with inline bytes
//   close <sec> <nanosec>       close_window
//   status <sec> <nanosec> <period_s>
//   activity <sec> <nanosec>    the Activity message
//   defaults                    every MemoryConfig default, one "default <key> <value>" line each
//   map <sec> <nanosec>         map_update
//   sync <sec> <nanosec> <hex>  sync with a CDR RequestSync: "sync <n>", the answer's messages as
//                               "data" lines, then "end <SyncEnd hex>" ("end none" for another room)
//   imu <sec> <nanosec> <hex>   add_imu with a CDR sensor_msgs/Imu received at that time
//   rig <sec> <nanosec>         the RigPose due, as CDR hex, or none
#include <algorithm>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "config_keys.hpp"
#include "room_memory/memory.hpp"

namespace {

using room_memory::MemoryConfig;
using room_memory::RoomMemory;
namespace msg = room_memory::msg;

std::string hex(const msg::Bytes& bytes) {
    static const char digits[] = "0123456789abcdef";
    std::string out;
    out.reserve(bytes.size() * 2);
    for (const std::uint8_t b : bytes) {
        out.push_back(digits[b >> 4]);
        out.push_back(digits[b & 15]);
    }
    return out;
}

msg::Bytes unhex(const std::string& text) {
    msg::Bytes out;
    for (std::size_t n = 0; n + 1 < text.size(); n += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(n, 2), nullptr, 16)));
    }
    return out;
}

msg::Bytes read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path);
    return msg::Bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::vector<float> read_floats(const std::string& path) {
    const msg::Bytes bytes = read_file(path);
    std::vector<float> out(bytes.size() / 4);
    for (std::size_t n = 0; n < out.size(); ++n) {
        const std::uint32_t bits = std::uint32_t{bytes[4 * n]} | std::uint32_t{bytes[4 * n + 1]} << 8 |
                                   std::uint32_t{bytes[4 * n + 2]} << 16 | std::uint32_t{bytes[4 * n + 3]} << 24;
        out[n] = room_memory::bits_float(bits);
    }
    return out;
}

msg::Time time_of(std::istringstream& words) {
    msg::Time t;
    words >> t.sec >> t.nanosec;
    return t;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "room_memory_replay <script>\n";
        return 2;
    }
    std::ifstream script(argv[1]);
    if (!script) {
        std::cerr << "room_memory_replay: cannot read " << argv[1] << "\n";
        return 2;
    }
    MemoryConfig config;
    std::unique_ptr<RoomMemory> memory;
    for (std::string line; std::getline(script, line);) {
        std::istringstream words(line);
        std::string command;
        if (!(words >> command) || command[0] == '#') continue;
        if (command == "config") {
            for (std::string pair; words >> pair;) {
                const std::size_t eq = pair.find('=');
                room_memory::tools::configure(config, pair.substr(0, eq), pair.substr(eq + 1));
            }
        } else if (command == "defaults") {
            const MemoryConfig d;
            std::string sensors;
            for (const std::string& name : d.sensors) sensors += (sensors.empty() ? "" : ",") + name;
            const std::pair<const char*, std::string> fields[] = {
                {"room_id", d.room_id}, {"sensors", sensors}, {"voxel_size", std::to_string(d.voxel_size)},
                {"max_range", d.max_range ? std::to_string(*d.max_range) : "none"},
                {"tolerance", std::to_string(d.tolerance)}, {"min_voxels", std::to_string(d.min_voxels)},
                {"min_extent", std::to_string(d.min_extent)}, {"window_s", std::to_string(d.window_s)},
                {"reference_windows", std::to_string(d.reference_windows)},
                {"recent_windows", std::to_string(d.recent_windows)}, {"state_dir", d.state_dir.value_or("none")},
                {"occupied_min_hits", std::to_string(d.occupied_min_hits)},
                {"occupied_min_ratio", std::to_string(d.occupied_min_ratio)},
                {"free_min_seen", std::to_string(d.free_min_seen)}, {"free_max_ratio", std::to_string(d.free_max_ratio)},
                {"candidate_windows", std::to_string(d.candidate_windows)},
                {"expire_windows", std::to_string(d.expire_windows)},
                {"fresh_min_voxels", std::to_string(d.fresh_min_voxels)},
                {"fresh_min_fraction", std::to_string(d.fresh_min_fraction)},
                {"stable_min_fraction", std::to_string(d.stable_min_fraction)},
                {"activity_radius_m", std::to_string(d.activity_radius_m)},
                {"learn_max_offset_m", std::to_string(d.learn_max_offset_m)},
                {"learn_windows", std::to_string(d.learn_windows)}, {"map_sensor", d.map_sensor},
                {"delta_period_s", std::to_string(d.delta_period_s)},
                {"keyframe_period_s", std::to_string(d.keyframe_period_s)},
                {"keyframe_delta_ratio", std::to_string(d.keyframe_delta_ratio)},
                {"rig_guard", d.rig_guard ? "true" : "false"}, {"imu_topic", d.imu_topic},
                {"imu_shock_frac", std::to_string(d.imu_shock_frac)}, {"imu_gyro_dps", std::to_string(d.imu_gyro_dps)},
                {"imu_quiet_s", std::to_string(d.imu_quiet_s)}, {"tilt_trigger_deg", std::to_string(d.tilt_trigger_deg)},
                {"pose_trigger_m", std::to_string(d.pose_trigger_m)},
                {"guard_check_period_s", std::to_string(d.guard_check_period_s)},
                {"check_max_s", std::to_string(d.check_max_s)}, {"move_min_m", std::to_string(d.move_min_m)},
                {"move_min_deg", std::to_string(d.move_min_deg)}, {"align_max_m", std::to_string(d.align_max_m)},
                {"align_max_deg", std::to_string(d.align_max_deg)},
                {"align_inlier_ratio", std::to_string(d.align_inlier_ratio)},
                {"align_max_rms_m", std::to_string(d.align_max_rms_m)},
                {"align_stable_windows", std::to_string(d.align_stable_windows)},
                {"observe_min_eig", std::to_string(d.observe_min_eig)},
                {"gross_inlier_ratio", std::to_string(d.gross_inlier_ratio)},
                {"relearn_after_s", std::to_string(d.relearn_after_s)}, {"auto_relearn", d.auto_relearn ? "true" : "false"},
                {"reg_voxel_m", std::to_string(d.reg_voxel_m)}, {"reg_max_points", std::to_string(d.reg_max_points)},
                {"src_voxel_m", std::to_string(d.src_voxel_m)}, {"src_max_points", std::to_string(d.src_max_points)},
                {"icp_max_range_m", std::to_string(d.icp_max_range_m)},
            };
            for (const auto& [key, value] : fields) std::cout << "default " << key << " " << value << "\n";
        } else if (command == "start") {
            const msg::Time start = time_of(words);
            std::uint64_t generation =
                static_cast<std::uint64_t>(std::max<std::int64_t>(1, start.sec * 1000 + start.nanosec / 1'000'000));
            std::string text;
            // An explicit generation that is not a uint64 (negative, too large) is refused like 0.
            const bool parsed = !(words >> text) ||
                                std::from_chars(text.data(), text.data() + text.size(), generation).ec == std::errc{};
            if (!parsed) generation = 0;
            try {
                memory = std::make_unique<RoomMemory>(config, start, generation);
                std::cout << "init " << msg::name(memory->state()) << " " << memory->map_epoch() << "\n";
            } catch (const std::invalid_argument&) {
                std::cout << "init error\n";
            }
        } else if (command == "points") {
            std::string sensor, path;
            words >> sensor >> path;
            memory->add_points(sensor, read_floats(path));
        } else if (command == "cdr" || command == "hex") {
            std::string sensor, value;
            words >> sensor >> value;
            const msg::Bytes data = command == "cdr" ? read_file(value) : unhex(value);
            std::cout << "cdr " << (memory->add_cdr(sensor, data) ? 1 : 0) << "\n";
        } else if (command == "close") {
            const msg::Time stamp = time_of(words);
            const std::vector<msg::ChangeEvent> events = memory->close_window(stamp);
            std::cout << "close " << events.size() << " " << msg::name(memory->state()) << " " << memory->map_epoch()
                      << "\n";
            for (const msg::ChangeEvent& event : events) std::cout << "event " << hex(event.to_cdr()) << "\n";
        } else if (command == "status") {
            const msg::Time stamp = time_of(words);
            double period = 0;
            words >> period;
            std::cout << "status " << hex(memory->status(stamp, period).to_cdr()) << "\n";
        } else if (command == "activity") {
            std::cout << "activity " << hex(memory->activity(time_of(words)).to_cdr()) << "\n";
        } else if (command == "map") {
            const std::optional<msg::Bytes> message = memory->map_update(time_of(words));
            std::cout << "map " << (message ? hex(*message) : "none") << "\n";
        } else if (command == "sync") {
            const msg::Time stamp = time_of(words);
            std::string value;
            words >> value;
            std::optional<msg::RequestSync> request;
            try {
                request = msg::RequestSync::from_cdr(unhex(value));
            } catch (const msg::CDRError&) {
                std::cout << "sync invalid\n";
            }
            if (request) {
                const RoomMemory::SyncAnswer answer = memory->sync(*request, stamp);
                std::cout << "sync " << answer.messages.size() << "\n";
                for (const msg::Bytes& message : answer.messages) std::cout << "data " << hex(message) << "\n";
                std::cout << "end " << (answer.end ? hex(answer.end->to_cdr()) : "none") << "\n";
            }
        } else if (command == "imu") {
            const msg::Time received = time_of(words);
            std::string value;
            words >> value;
            std::cout << "imu " << (memory->add_imu(unhex(value), received) ? 1 : 0) << "\n";
        } else if (command == "rig") {
            const std::optional<msg::RigPose> message = memory->rig_pose(time_of(words));
            std::cout << "rig " << (message ? hex(message->to_cdr()) : "none") << "\n";
        } else {
            std::cerr << "room_memory_replay: unknown command " << command << "\n";
            return 2;
        }
    }
    return 0;
}
