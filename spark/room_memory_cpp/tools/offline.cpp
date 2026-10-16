#include "offline.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <ostream>

#include "room_memory/pointcloud2.hpp"

namespace room_memory::offline {

namespace {

constexpr const char* kUsage =
    "room_memory_offline [--voxel-size m] [--max-range m] [--min-points n]\n"
    "                    [--tolerance n] [--min-voxels n] [--min-extent n]\n"
    "                    [--max-move m] [--no-pair] [--evidence | --no-evidence]\n"
    "                    [--origin x,y,z]\n"
    "                    [--ray-stride n] [--unobserved] [--json | --text]\n"
    "                    <before.bin> <after.bin>\n"
    "evidence and JSON are on by default (change-detect --evidence --json)\n";

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }

std::string strip(const std::string& text) {
    std::size_t begin = 0;
    std::size_t end = text.size();
    while (begin < end && is_space(text[begin])) ++begin;
    while (end > begin && is_space(text[end - 1])) --end;
    return text.substr(begin, end - begin);
}

std::string lower(std::string text) {
    for (char& c : text) c = static_cast<char>(c >= 'A' && c <= 'Z' ? c + 32 : c);
    return text;
}

bool digits(const std::string& text, std::size_t& at) {
    const std::size_t start = at;
    while (at < text.size() && text[at] >= '0' && text[at] <= '9') ++at;
    return at > start;
}

// Python int(): optional whitespace and sign, decimal digits with single underscores between
// them. Values outside int64 are refused.
std::optional<std::int64_t> parse_int(const std::string& raw) {
    const std::string text = strip(raw);
    std::size_t at = 0;
    bool negative = false;
    if (at < text.size() && (text[at] == '+' || text[at] == '-')) negative = text[at++] == '-';
    std::string number;
    bool expect_digit = true;
    for (; at < text.size(); ++at) {
        const char c = text[at];
        if (c >= '0' && c <= '9') {
            number.push_back(c);
            expect_digit = false;
        } else if (c == '_' && !expect_digit) {
            expect_digit = true;
        } else {
            return std::nullopt;
        }
    }
    if (number.empty() || expect_digit) return std::nullopt;
    errno = 0;
    const long long value = std::strtoll((negative ? "-" + number : number).c_str(), nullptr, 10);
    if (errno == ERANGE) return std::nullopt;
    return value;
}

std::string format_fixed(double value, int places) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.*f", places, value);
    return buffer;
}

std::string point(const Vec3& values) {
    return format_fixed(values[0], 3) + "," + format_fixed(values[1], 3) + "," + format_fixed(values[2], 3);
}

std::string bounds(const Vec3& low, const Vec3& high) { return "[" + point(low) + " .. " + point(high) + "]"; }

std::vector<float> load(const std::string& path) {
    const std::string suffix = lower(std::filesystem::path(path).extension().string());
    if (suffix != ".bin") throw PointCloudError("expected a .bin file, got '" + (suffix.empty() ? path : suffix) + "'");
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("[Errno 2] No such file or directory: '" + path + "'");
    const std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return decode_pointcloud2(data).positions;
}

}  // namespace

std::optional<float> parse_f32(const std::string& raw) {
    const std::string text = strip(raw);
    std::size_t at = 0;
    if (at < text.size() && (text[at] == '+' || text[at] == '-')) ++at;
    const std::string word = lower(text.substr(at));
    if (word == "inf" || word == "infinity" || word == "nan") {
        const float magnitude = word == "nan" ? std::nanf("") : INFINITY;
        return text[0] == '-' ? -magnitude : magnitude;
    }
    const bool whole = digits(text, at);
    bool fraction = false;
    if (at < text.size() && text[at] == '.') {
        ++at;
        fraction = digits(text, at);
    }
    if (!whole && !fraction) return std::nullopt;
    if (at < text.size() && (text[at] == 'e' || text[at] == 'E')) {
        ++at;
        if (at < text.size() && (text[at] == '+' || text[at] == '-')) ++at;
        if (!digits(text, at)) return std::nullopt;
    }
    if (at != text.size()) return std::nullopt;
    // strtof rounds the exact decimal once, to nearest with ties to even.
    return std::strtof(text.c_str(), nullptr);
}

std::optional<Invocation> parse_args(const std::vector<std::string>& args, bool evidence, bool emit_json) {
    Invocation call;
    call.config.evidence_of_absence = evidence;
    call.emit_json = emit_json;
    std::vector<std::string> paths;
    for (std::size_t index = 0; index < args.size(); ++index) {
        const std::string& arg = args[index];
        const auto value = [&]() -> std::optional<std::string> {
            if (index + 1 >= args.size() || args[index + 1].starts_with("-")) return std::nullopt;
            return args[++index];
        };
        const auto real = [&]() -> std::optional<float> {
            const auto text = value();
            return text ? parse_f32(*text) : std::nullopt;
        };
        const auto integer = [&]() -> std::optional<std::int64_t> {
            const auto text = value();
            return text ? parse_int(*text) : std::nullopt;
        };
        if (arg == "--voxel-size" || arg == "--max-range" || arg == "--max-move") {
            const auto number = real();
            if (!number) return std::nullopt;
            if (arg == "--voxel-size") call.config.voxel_size = *number;
            if (arg == "--max-range") call.config.max_range = *number;
            if (arg == "--max-move") call.config.max_move = *number;
        } else if (arg == "--min-points" || arg == "--tolerance" || arg == "--min-voxels" || arg == "--min-extent" ||
                   arg == "--ray-stride") {
            const auto number = integer();
            if (!number) return std::nullopt;
            if (arg == "--min-points") call.config.min_points = *number;
            if (arg == "--tolerance") call.config.tolerance = *number;
            if (arg == "--min-voxels") call.config.min_voxels = *number;
            if (arg == "--min-extent") call.config.min_extent = *number;
            if (arg == "--ray-stride") call.config.ray_stride = *number;
        } else if (arg == "--no-pair") {
            call.config.pair_moves = false;
        } else if (arg == "--evidence") {
            call.config.evidence_of_absence = true;
        } else if (arg == "--no-evidence") {
            call.config.evidence_of_absence = false;
        } else if (arg == "--origin") {
            if (index + 1 >= args.size() || args[index + 1].starts_with("--")) return std::nullopt;
            const std::string& text = args[++index];
            std::vector<std::string> parts(1);
            for (const char c : text) {
                if (c == ',') {
                    parts.emplace_back();
                } else {
                    parts.back().push_back(c);
                }
            }
            if (parts.size() != 3) return std::nullopt;
            for (int a = 0; a < 3; ++a) {
                const auto number = parse_f32(parts[static_cast<std::size_t>(a)]);
                if (!number) return std::nullopt;
                call.config.origin[a] = *number;
            }
        } else if (arg == "--unobserved") {
            call.config.report_unobserved = true;
        } else if (arg == "--json") {
            call.emit_json = true;
        } else if (arg == "--text") {
            call.emit_json = false;
        } else if (arg == "--") {
            paths.insert(paths.end(), args.begin() + static_cast<std::ptrdiff_t>(index) + 1, args.end());
            break;
        } else if (arg.starts_with("-")) {
            return std::nullopt;
        } else {
            paths.push_back(arg);
        }
    }
    if (paths.size() != 2) return std::nullopt;
    call.before = paths[0];
    call.after = paths[1];
    return call;
}

int run(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    const auto call = parse_args(args, true, true);
    if (!call) {
        err << kUsage;
        return 1;
    }
    const Config& config = call->config;
    std::vector<float> before_points;
    std::vector<float> after_points;
    Comparison result;
    double elapsed_ms = 0;
    try {
        before_points = load(call->before);
        after_points = load(call->after);
        const auto start = std::chrono::steady_clock::now();
        result = compare(before_points, after_points, config);
        elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    } catch (const std::exception& error) {
        err << "room_memory_offline: " << error.what() << "\n";
        return 1;
    }
    const OccupancyMap& before = result.before;
    const OccupancyMap& after = result.after;
    const Report& report = result.report;
    if (call->emit_json) {
        out << to_json(report, before, after, config) << "\n";
        return 0;
    }
    const auto name = [](const std::string& path) { return std::filesystem::path(path).filename().string(); };
    out << "before: " << name(call->before) << "  points=" << before_points.size() / 3
        << "  voxels=" << before.occupied.size() << "  dropped=" << before.dropped_points << "\n";
    out << "after:  " << name(call->after) << "  points=" << after_points.size() / 3
        << "  voxels=" << after.occupied.size() << "  dropped=" << after.dropped_points << "\n";
    out << "voxel_size_m=" << format_fixed(config.voxel_size, 3) << "  tolerance=" << config.tolerance
        << "  min_points=" << config.min_points << "  min_voxels=" << config.min_voxels
        << "  min_extent=" << config.min_extent << "  pair_moves=" << (config.pair_moves ? "true" : "false") << "\n";
    out << "appeared_voxels=" << report.appeared_voxels << "  disappeared_voxels=" << report.disappeared_voxels;
    if (config.evidence_of_absence) out << "  unknown_voxels=" << *report.unknown_voxels;
    out << "\n";
    out << "elapsed_ms=" << format_fixed(elapsed_ms, 1) << "\n";
    out << "regions=" << report.regions.size() << "\n";
    for (std::size_t n = 0; n < report.regions.size(); ++n) {
        const Region& region = report.regions[n];
        out << "region " << n << " " << kind_name(region.kind) << " voxels=" << region.voxel_count
            << " volume_m3=" << format_fixed(region.volume, 6) << " centroid=" << point(region.centroid)
            << " bounds=" << bounds(region.bounds_min, region.bounds_max);
        if (region.kind == Region::Kind::moved) {
            out << " from_voxels=" << region.from_voxel_count << " from_centroid=" << point(region.from_centroid)
                << " from_bounds=" << bounds(region.from_bounds_min, region.from_bounds_max);
        }
        out << "\n";
    }
    return 0;
}

}  // namespace room_memory::offline
