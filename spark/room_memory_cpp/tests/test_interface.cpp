// The deployed node creates generic publishers by type name. Those names resolve only when
// ros/labcommand_msgs generates the message. Activity is created even with rhythm_publish
// false, before any subscription, so a missing Activity.msg stops detection and rhythm logging.
// This test runs without ROS: it checks the package the image builds, and that Activity.msg's
// XCDR1 layout matches the hand goldens the codec publishes.
#include <cctype>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "harness.hpp"

namespace {

const std::filesystem::path kMsgs = ROOM_MEMORY_MSGS_DIR;
const std::filesystem::path kNode = ROOM_MEMORY_NODE_SOURCE;
const std::filesystem::path kHand = std::filesystem::path(ROOM_MEMORY_FIXTURES) / "hand";

std::string trim_comment(std::string line) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line.resize(hash);
    return line;
}

std::set<std::string> types_named_by(const std::string& source) {
    std::set<std::string> names;
    const std::string key = "labcommand_msgs/msg/";
    for (std::size_t pos = 0; (pos = source.find(key, pos)) != std::string::npos;) {
        pos += key.size();
        std::size_t end = pos;
        while (end < source.size() &&
               (std::isalnum(static_cast<unsigned char>(source[end])) || source[end] == '_')) {
            ++end;
        }
        if (end == pos) throw harness::Failure("empty type name after " + key);
        names.insert(source.substr(pos, end - pos));
    }
    return names;
}

std::set<std::string> types_generated_by(const std::string& cmake) {
    std::set<std::string> names;
    std::istringstream lines(cmake);
    for (std::string line; std::getline(lines, line);) {
        line = trim_comment(std::move(line));
        const std::string key = "msg/";
        const auto at = line.find(key);
        if (at == std::string::npos) continue;
        const auto begin = at + key.size();
        const auto end = line.find(".msg", begin);
        if (end == std::string::npos) continue;
        const std::string name = line.substr(begin, end - begin);
        if (name.empty() || name.find(' ') != std::string::npos) throw harness::Failure("bad msg entry: " + line);
        names.insert(name);
    }
    return names;
}

struct Field {
    std::string type;
    std::string name;
};

std::vector<Field> fields_of(const std::string& text) {
    std::vector<Field> fields;
    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);) {
        line = trim_comment(std::move(line));
        std::istringstream tokens(line);
        std::string type;
        std::string name;
        if (!(tokens >> type >> name)) continue;
        if (type.find('=') != std::string::npos || name.find('=') != std::string::npos) continue;
        fields.push_back(Field{type, name});
    }
    return fields;
}

// XCDR1 alignment counted from the first byte after the encapsulation header.
// A string is sized as room_id "lab" (uint32 length including the NUL, then 4 bytes).
struct Layout {
    std::map<std::string, int> offset;
    int end = 0;
};

int align_size(const std::string& base, int* align) {
    if (base == "bool" || base == "uint8" || base == "int8" || base == "byte" || base == "char") {
        *align = 1;
        return 1;
    }
    if (base == "uint16" || base == "int16") {
        *align = 2;
        return 2;
    }
    if (base == "uint32" || base == "int32" || base == "float32") {
        *align = 4;
        return 4;
    }
    if (base == "uint64" || base == "int64" || base == "float64") {
        *align = 8;
        return 8;
    }
    if (base == "builtin_interfaces/Time") {
        *align = 4;
        return 8;
    }
    if (base == "string") {
        *align = 4;
        return -1;
    }
    throw harness::Failure("Activity.msg field type is not in the golden layout: " + base);
}

Layout layout_of(const std::vector<Field>& fields) {
    Layout layout;
    int at = 0;
    for (const Field& field : fields) {
        std::string base = field.type;
        int count = 1;
        const auto bracket = field.type.find('[');
        if (bracket != std::string::npos) {
            if (field.type.back() != ']') throw harness::Failure("bad array type " + field.type);
            base = field.type.substr(0, bracket);
            const std::string inside = field.type.substr(bracket + 1, field.type.size() - bracket - 2);
            if (inside.empty() || inside.find_first_not_of("0123456789") != std::string::npos) {
                throw harness::Failure("Activity.msg array is not a fixed decimal count: " + field.type);
            }
            count = std::stoi(inside);
        }
        int align = 0;
        int size = align_size(base, &align);
        if (size < 0) {
            if (field.name != "room_id" || count != 1) throw harness::Failure("only room_id is a string");
            size = 8;  // uint32 length + "lab\0"
        } else {
            size *= count;
        }
        at = (at + align - 1) & ~(align - 1);
        if (!layout.offset.emplace(field.name, at).second) throw harness::Failure("duplicate field " + field.name);
        at += size;
    }
    layout.end = at;
    return layout;
}

std::vector<std::uint8_t> hexfile(const std::filesystem::path& path) {
    const std::string text = harness::read_text(path);
    std::string digits;
    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);) {
        line = trim_comment(std::move(line));
        for (const char c : line) {
            if (std::isxdigit(static_cast<unsigned char>(c))) digits.push_back(c);
        }
    }
    if (digits.size() % 2 != 0) throw harness::Failure(path.filename().string() + " has an odd number of hex digits");
    std::vector<std::uint8_t> out;
    out.reserve(digits.size() / 2);
    for (std::size_t n = 0; n < digits.size(); n += 2) {
        out.push_back(static_cast<std::uint8_t>(std::stoul(digits.substr(n, 2), nullptr, 16)));
    }
    return out;
}

}  // namespace

TEST(node_types_are_generated) {
    const std::string source = harness::read_text(kNode);
    const auto named = types_named_by(source);
    if (!named.count("Activity")) {
        throw harness::Failure("room_memory_node no longer publishes labcommand_msgs/msg/Activity");
    }
    // Activity is created on the default path. rhythm_publish false must not skip it.
    const auto activity = source.find("\"labcommand_msgs/msg/Activity\"");
    const auto rhythm_gate = source.find("if (config.rhythm_publish)");
    if (activity == std::string::npos || rhythm_gate == std::string::npos || activity > rhythm_gate) {
        throw harness::Failure("Activity publisher must be created before the rhythm_publish gate");
    }
    const auto generated = types_generated_by(harness::read_text(kMsgs / "CMakeLists.txt"));
    for (const std::string& name : named) {
        if (!generated.count(name)) {
            throw harness::Failure("room_memory_node uses labcommand_msgs/msg/" + name +
                                   " but ros/labcommand_msgs does not generate it");
        }
        if (!std::filesystem::is_regular_file(kMsgs / "msg" / (name + ".msg"))) {
            throw harness::Failure("generated message has no file: " + name + ".msg");
        }
    }
    for (const auto& entry : std::filesystem::directory_iterator(kMsgs / "msg")) {
        if (entry.path().extension() != ".msg") continue;
        const std::string name = entry.path().stem().string();
        if (!generated.count(name)) throw harness::Failure(name + ".msg is not passed to rosidl_generate_interfaces");
    }
}

TEST(activity_msg_matches_the_hand_golden_layout) {
    const std::vector<Field> fields = fields_of(harness::read_text(kMsgs / "msg" / "Activity.msg"));
    const Layout layout = layout_of(fields);
    const std::pair<const char*, int> expect[] = {
        {"room_id", 0},
        {"map_epoch", 8},
        {"seq", 16},
        {"stamp", 24},
        {"active", 32},
        {"sensors", 33},
        {"largest_sensor", 34},
        {"n_regions", 36},
        {"voxel_count", 40},
        {"centroid", 44},
        {"aabb", 56},
        {"since_s", 80},
        // Protocol v2 (D38). The hand goldens omit it; alignment pads 4 bytes after since_s.
        {"publisher_generation", 88},
    };
    if (fields.size() != sizeof(expect) / sizeof(expect[0])) {
        throw harness::Failure("Activity.msg field count is " + std::to_string(fields.size()) + ", layout has 13");
    }
    for (std::size_t n = 0; n < fields.size(); ++n) {
        if (fields[n].name != expect[n].first) {
            throw harness::Failure(std::string("Activity.msg field ") + std::to_string(n) + " is " + fields[n].name +
                                   ", golden order is " + expect[n].first);
        }
        const auto found = layout.offset.find(fields[n].name);
        if (found == layout.offset.end() || found->second != expect[n].second) {
            throw harness::Failure(fields[n].name + " is not at golden offset " + std::to_string(expect[n].second));
        }
    }
    if (layout.end != 96) throw harness::Failure("Activity payload for room_id lab is not 96 bytes with publisher_generation");
    // The hand goldens are the legacy layout: generation omitted, payload ends on since_s.
    if (layout.offset.at("since_s") + 4 != 84) {
        throw harness::Failure("legacy Activity payload does not end at 84 bytes");
    }
    // bool, then the two uint8 sensor fields, then the pad the golden annotates before n_regions.
    if (fields[4].type != "bool" || fields[5].type != "uint8" || fields[6].type != "uint8") {
        throw harness::Failure("active, sensors and largest_sensor are not bool, uint8, uint8");
    }
    for (const char* name : {"activity_le.hex", "activity_idle_le.hex"}) {
        const auto bytes = hexfile(kHand / name);
        if (bytes.size() != 88) {
            throw harness::Failure(std::string(name) + " is " + std::to_string(bytes.size()) +
                                   " bytes, encapsulation plus 84");
        }
    }
}

TEST_MAIN
