// room_memory_replay <script>: drives RoomMemory through a window sequence and prints every
// output message as CDR hex, for byte comparison with tests/replay.py (the Python oracle).
//
// Script lines (whitespace-separated, # comments):
//   config key=value ...        MemoryConfig fields; sensors=a,b; max_range=none; state_dir=none
//   start <sec> <nanosec>       construct RoomMemory
//   points <sensor> <file>      add_points from raw little-endian float32 x, y, z
//   cdr <sensor> <file>         add_cdr with the file's bytes
//   hex <sensor> <hex>          add_cdr with inline bytes
//   close <sec> <nanosec>       close_window
//   status <sec> <nanosec> <period_s>
//   map <sec> <nanosec>         map_update
//   sync <sec> <nanosec> <hex>  sync with a CDR RequestSync
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
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
        } else if (command == "start") {
            const msg::Time start = time_of(words);
            try {
                memory = std::make_unique<RoomMemory>(config, start);
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
        } else if (command == "map") {
            const std::optional<msg::Bytes> message = memory->map_update(time_of(words));
            std::cout << "map " << (message ? hex(*message) : "none") << "\n";
        } else if (command == "sync") {
            const msg::Time stamp = time_of(words);
            std::string value;
            words >> value;
            try {
                const std::vector<msg::Bytes> answer = memory->sync(msg::RequestSync::from_cdr(unhex(value)), stamp);
                std::cout << "sync " << answer.size() << "\n";
                for (const msg::Bytes& message : answer) std::cout << "data " << hex(message) << "\n";
            } catch (const msg::CDRError&) {
                std::cout << "sync invalid\n";
            }
        } else {
            std::cerr << "room_memory_replay: unknown command " << command << "\n";
            return 2;
        }
    }
    return 0;
}
