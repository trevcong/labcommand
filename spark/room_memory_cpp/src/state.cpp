#include "room_memory/state.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string_view>

namespace room_memory {

namespace {

constexpr char kRegistrationMagic[4] = {'L', 'C', 'R', 'G'};

std::uint32_t crc32_of(std::span<const std::uint8_t> data) {
    return static_cast<std::uint32_t>(::crc32(0L, data.data(), static_cast<uInt>(data.size())));
}

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int n = 0; n < 4; ++n) out.push_back(static_cast<std::uint8_t>(value >> (8 * n)));
}

void put_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    for (int n = 0; n < 8; ++n) out.push_back(static_cast<std::uint8_t>(value >> (8 * n)));
}

std::uint32_t get_u32(std::span<const std::uint8_t> data, std::size_t at) {
    std::uint32_t value = 0;
    for (int n = 3; n >= 0; --n) value = (value << 8) | data[at + static_cast<std::size_t>(n)];
    return value;
}

std::uint64_t get_u64(std::span<const std::uint8_t> data, std::size_t at) {
    std::uint64_t value = 0;
    for (int n = 7; n >= 0; --n) value = (value << 8) | data[at + static_cast<std::size_t>(n)];
    return value;
}

// Strict UTF-8 (no overlongs, surrogates, or code points past U+10FFFF), as Python decodes.
bool valid_utf8(std::string_view text) {
    std::size_t n = 0;
    while (n < text.size()) {
        const auto c = static_cast<unsigned char>(text[n]);
        std::size_t extra;
        std::uint32_t point;
        if (c < 0x80) {
            ++n;
            continue;
        } else if (c >= 0xC2 && c <= 0xDF) {
            extra = 1;
            point = c & 0x1F;
        } else if (c >= 0xE0 && c <= 0xEF) {
            extra = 2;
            point = c & 0x0F;
        } else if (c >= 0xF0 && c <= 0xF4) {
            extra = 3;
            point = c & 0x07;
        } else {
            return false;
        }
        if (n + extra >= text.size()) return false;
        for (std::size_t e = 1; e <= extra; ++e) {
            const auto next = static_cast<unsigned char>(text[n + e]);
            if ((next & 0xC0) != 0x80) return false;
            point = (point << 6) | (next & 0x3F);
        }
        if ((extra == 2 && (point < 0x800 || (point >= 0xD800 && point <= 0xDFFF))) ||
            (extra == 3 && (point < 0x10000 || point > 0x10FFFF))) {
            return false;
        }
        n += extra + 1;
    }
    return true;
}

// Python's f"{value:.17g}" (every NaN is "nan").
std::string number(double value) {
    if (std::isnan(value)) return "nan";
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.17g", value);
    return buffer;
}

std::string hex32(std::uint32_t value) {
    char buffer[16];
    std::snprintf(buffer, sizeof buffer, "%08x", value);
    return buffer;
}

// Python's repr of an ASCII str.
std::string repr(std::string_view text) {
    const char quote = text.find('\'') != std::string_view::npos && text.find('"') == std::string_view::npos ? '"' : '\'';
    std::string out(1, quote);
    for (const char c : text) {
        if (c == '\\' || c == quote) {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else if (c < 0x20 || c == 0x7F) {
            out += "\\x" + hex32(static_cast<std::uint32_t>(c)).substr(6);
        } else {
            out += c;
        }
    }
    return out + quote;
}

// Python's float(): what %.17g writes, inf and nan; no hex, no NaN payloads.
double parse_float(const std::string& word) {
    const auto bad = [&] { return std::invalid_argument("could not convert string to float: " + repr(word)); };
    if (word.empty() || word.find_first_of("xX(") != std::string::npos) throw bad();
    char* end = nullptr;
    const double value = std::strtod(word.c_str(), &end);  // ERANGE is not an error: Python rounds to inf or 0
    if (end != word.c_str() + word.size()) throw bad();
    return value;
}

std::uint64_t parse_unsigned(const std::string& word, int base, std::uint64_t limit) {
    const auto bad = [&] {
        return std::invalid_argument("invalid literal for int() with base " + std::to_string(base) + ": " + repr(word));
    };
    if (word.empty() || word.find('-') != std::string::npos) throw bad();
    errno = 0;
    char* end = nullptr;
    const unsigned long long value = std::strtoull(word.c_str(), &end, base);
    if (end != word.c_str() + word.size()) throw bad();
    if (errno == ERANGE || value > limit) throw std::invalid_argument(repr(word) + " is out of range");
    return value;
}

std::int64_t parse_signed(const std::string& word) {
    const auto bad = [&] { return std::invalid_argument("invalid literal for int() with base 10: " + repr(word)); };
    if (word.empty()) throw bad();
    errno = 0;
    char* end = nullptr;
    const long long value = std::strtoll(word.c_str(), &end, 10);
    if (end != word.c_str() + word.size()) throw bad();
    if (errno == ERANGE) throw std::invalid_argument(repr(word) + " is out of range");
    return value;
}

// str.split(" "): every single space separates, empty words kept.
std::vector<std::string> split_words(std::string_view line) {
    std::vector<std::string> words;
    std::size_t start = 0;
    for (std::size_t at; (at = line.find(' ', start)) != std::string_view::npos; start = at + 1) {
        words.emplace_back(line.substr(start, at - start));
    }
    words.emplace_back(line.substr(start));
    return words;
}

// str.splitlines() of ASCII text.
std::vector<std::string_view> split_lines(std::string_view text) {
    std::vector<std::string_view> lines;
    std::size_t start = 0;
    for (std::size_t n = 0; n < text.size(); ++n) {
        const char c = text[n];
        if (c != '\n' && c != '\r' && c != '\v' && c != '\f' && c != '\x1c' && c != '\x1d' && c != '\x1e') continue;
        lines.push_back(text.substr(start, n - start));
        if (c == '\r' && n + 1 < text.size() && text[n + 1] == '\n') ++n;
        start = n + 1;
    }
    if (start < text.size()) lines.push_back(text.substr(start));
    return lines;
}

const std::vector<std::string>& field(const std::map<std::string, std::vector<std::string>>& fields,
                                      const std::string& key) {
    const auto found = fields.find(key);
    if (found == fields.end()) throw std::invalid_argument("'" + key + "'");
    return found->second;
}

const std::string& first(const std::vector<std::string>& words) {
    if (words.empty()) throw std::invalid_argument("list index out of range");
    return words[0];
}

void fsync_file(const std::filesystem::path& path) {
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) throw std::runtime_error("cannot open " + path.string());
    const bool synced = ::fsync(fd) == 0;
    ::close(fd);
    if (!synced) throw std::runtime_error("cannot sync " + path.string());
}

// re.fullmatch(r"[0-9]{4}-.*", name)
bool archive_name(const std::string& name) {
    if (name.size() < 5 || name[4] != '-' || name.find('\n') != std::string::npos) return false;
    return std::all_of(name.begin(), name.begin() + 4, [](char c) { return c >= '0' && c <= '9'; });
}

}  // namespace

std::string registration_name(const std::string& sensor) { return "registration-" + sensor + ".bin"; }

std::vector<std::uint8_t> registration_bytes(const std::string& sensor, std::uint64_t frame_epoch,
                                             std::span<const float> points) {
    if (points.size() % 3 != 0) throw std::invalid_argument("registration points are not triples");
    if (sensor.size() > UINT32_MAX || points.size() / 3 > UINT32_MAX) {
        throw std::invalid_argument("registration cloud is too large");
    }
    std::vector<std::uint8_t> out(kRegistrationMagic, kRegistrationMagic + 4);
    out.reserve(4 + 4 + 4 + sensor.size() + 8 + 4 + 4 * points.size() + 4);
    put_u32(out, kRegistrationVersion);
    put_u32(out, static_cast<std::uint32_t>(sensor.size()));
    out.insert(out.end(), sensor.begin(), sensor.end());
    put_u64(out, frame_epoch);
    put_u32(out, static_cast<std::uint32_t>(points.size() / 3));
    for (const float value : points) {
        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof bits);
        put_u32(out, bits);
    }
    put_u32(out, crc32_of(out));
    return out;
}

Registration parse_registration(std::span<const std::uint8_t> data, const std::string& what) {
    const auto bad = [&](const std::string& reason) { return std::invalid_argument(what + ": " + reason); };
    if (data.size() < 4 + 4 + 4 + 8 + 4 + 4 || std::memcmp(data.data(), kRegistrationMagic, 4) != 0) {
        throw bad("not a registration cloud");
    }
    if (get_u32(data, data.size() - 4) != crc32_of(data.first(data.size() - 4))) throw bad("CRC mismatch");
    if (get_u32(data, 4) != kRegistrationVersion) throw bad("unsupported version");
    std::uint64_t at = 12 + std::uint64_t{get_u32(data, 8)};
    if (at + 12 + 4 > data.size()) throw bad("truncated");
    Registration out;
    out.sensor.assign(data.begin() + 12, data.begin() + static_cast<std::ptrdiff_t>(at));
    if (!valid_utf8(out.sensor)) throw bad("sensor name is not UTF-8");
    out.frame_epoch = get_u64(data, at);
    const std::uint64_t count = get_u32(data, at + 8);
    at += 12;
    if (at + 12 * count + 4 != data.size()) throw bad("point count does not match the size");
    out.points.resize(3 * count);
    for (std::size_t n = 0; n < out.points.size(); ++n) {
        const std::uint32_t bits = get_u32(data, at + 4 * n);
        std::memcpy(&out.points[n], &bits, sizeof bits);
    }
    return out;
}

std::vector<std::uint8_t> format_rig_state(const RigState& state) {
    std::string body = std::string(kRigStateHeader) + "\n";
    body += "room " + state.room_id + "\n";
    body += "voxel_size " + number(state.voxel_size) + "\n";
    body += "frame_epoch " + std::to_string(state.frame_epoch) + "\n";
    body += "n_realigned " + std::to_string(state.n_realigned) + "\n";
    body += "g_base";
    if (state.g_base) {
        for (const double value : *state.g_base) body += " " + number(value);
    } else {
        body += " none";
    }
    body += "\n";
    for (const auto& [name, sensor] : state.sensors) {
        const RegistrationFile& file = sensor.registration;
        body += "sensor " + name + " pose " + sensor.pose.text() + " baseline " +
                (sensor.baseline ? number(*sensor.baseline) : "none") + " registration " + file.name + " " +
                std::to_string(file.size) + " " + hex32(file.crc) + "\n";
    }
    if (std::any_of(body.begin(), body.end(), [](char c) { return static_cast<unsigned char>(c) >= 0x80; })) {
        throw std::invalid_argument("rig state: not ASCII");
    }
    const auto bytes = [](const std::string& text) {
        return std::span(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
    };
    body += "crc " + hex32(crc32_of(bytes(body))) + "\n";
    return std::vector<std::uint8_t>(body.begin(), body.end());
}

RigState parse_rig_state(std::span<const std::uint8_t> data, const std::string& what) {
    const auto bad = [&](const std::string& reason) { return std::invalid_argument(what + ": " + reason); };
    if (std::any_of(data.begin(), data.end(), [](std::uint8_t c) { return c >= 0x80; })) throw bad("not ASCII");
    const std::string_view text(reinterpret_cast<const char*>(data.data()), data.size());
    // (.*\n)crc ([0-9a-f]{8})\n
    constexpr std::size_t kTrailer = 4 + 8 + 1;
    if (text.size() < kTrailer + 1 || text[text.size() - kTrailer - 1] != '\n' ||
        text.substr(text.size() - kTrailer, 4) != "crc " || text.back() != '\n') {
        throw bad("CRC mismatch");
    }
    const std::string_view digits = text.substr(text.size() - 9, 8);
    if (!std::all_of(digits.begin(), digits.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); })) {
        throw bad("CRC mismatch");
    }
    const std::string_view body = text.substr(0, text.size() - kTrailer);
    if (std::strtoul(std::string(digits).c_str(), nullptr, 16) != crc32_of(data.first(body.size()))) {
        throw bad("CRC mismatch");
    }
    const std::vector<std::string_view> lines = split_lines(body);
    if (lines.empty() || lines[0] != kRigStateHeader) throw bad("not a rig state");
    std::map<std::string, std::vector<std::string>> fields;
    RigState state;
    std::optional<std::vector<double>> g_base;
    try {
        for (std::size_t n = 1; n < lines.size(); ++n) {
            std::vector<std::string> words = split_words(lines[n]);
            if (words[0] != "sensor") {
                fields[words[0]] = std::vector<std::string>(words.begin() + 1, words.end());
                continue;
            }
            if (words.size() != 21 || words[2] != "pose" || words[15] != "baseline" || words[17] != "registration") {
                throw bad("malformed sensor line " + repr(lines[n]));
            }
            const std::optional<double> baseline =
                words[16] == "none" ? std::nullopt : std::optional<double>(parse_float(words[16]));
            std::array<double, 12> values;
            for (std::size_t v = 0; v < values.size(); ++v) values[v] = parse_float(words[3 + v]);
            SensorState sensor{rig::Pose::from_values(values), baseline,
                               RegistrationFile{words[18], parse_unsigned(words[19], 10, UINT64_MAX),
                                                static_cast<std::uint32_t>(parse_unsigned(words[20], 16, UINT32_MAX))}};
            // A repeated name keeps its place and takes the later line (a Python dict).
            const auto found = std::find_if(state.sensors.begin(), state.sensors.end(),
                                            [&](const auto& entry) { return entry.first == words[1]; });
            if (found != state.sensors.end()) {
                found->second = std::move(sensor);
            } else {
                state.sensors.emplace_back(words[1], std::move(sensor));
            }
        }
        const std::vector<std::string>& g_words = field(fields, "g_base");
        if (!(g_words.size() == 1 && g_words[0] == "none")) {
            g_base.emplace();
            for (const std::string& word : g_words) g_base->push_back(parse_float(word));
        }
        state.room_id = first(field(fields, "room"));
        state.voxel_size = parse_float(first(field(fields, "voxel_size")));
        state.frame_epoch = parse_unsigned(first(field(fields, "frame_epoch")), 10, UINT64_MAX);
        state.n_realigned = parse_signed(first(field(fields, "n_realigned")));
    } catch (const std::invalid_argument& error) {
        throw bad(std::string("malformed (") + error.what() + ")");
    }
    if (g_base) {
        if (g_base->size() != 3 || !std::all_of(g_base->begin(), g_base->end(), [](double v) { return std::isfinite(v); })) {
            throw bad("g_base is not three finite numbers");
        }
        state.g_base = rig::Vec3d{(*g_base)[0], (*g_base)[1], (*g_base)[2]};
    }
    return state;
}

// Temp file, fsync, rename, then fsync the directory.
void write_atomic(const std::filesystem::path& path, std::span<const std::uint8_t> data, bool make_parents) {
    const std::filesystem::path parent = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
    if (make_parents) std::filesystem::create_directories(parent);
    const std::filesystem::path scratch = path.string() + ".tmp";
    const int fd = ::open(scratch.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) throw std::runtime_error("cannot write " + scratch.string());
    std::size_t written = 0;
    while (written < data.size()) {
        const ssize_t n = ::write(fd, data.data() + written, data.size() - written);
        if (n < 0) {
            ::close(fd);
            throw std::runtime_error("cannot write " + scratch.string());
        }
        written += static_cast<std::size_t>(n);
    }
    const bool synced = ::fsync(fd) == 0;
    ::close(fd);
    if (!synced) throw std::runtime_error("cannot sync " + scratch.string());
    std::filesystem::rename(scratch, path);
    const int directory = ::open(parent.c_str(), O_RDONLY);
    if (directory < 0) throw std::runtime_error("cannot open " + parent.string() + " to sync it");
    const bool dir_synced = ::fsync(directory) == 0;
    ::close(directory);
    if (!dir_synced) throw std::runtime_error("cannot sync " + parent.string());
}

std::filesystem::path archive(const std::filesystem::path& room, const std::string& name,
                              const std::vector<std::string>& files, const std::vector<std::string>& remove) {
    const std::filesystem::path prev = room / "prev";
    std::filesystem::create_directories(prev);
    std::vector<std::filesystem::path> existing;
    for (const auto& entry : std::filesystem::directory_iterator(prev)) {
        if (entry.is_directory() && archive_name(entry.path().filename().string())) existing.push_back(entry.path());
    }
    std::sort(existing.begin(), existing.end());
    const int seq = existing.empty() ? 1 : std::stoi(existing.back().filename().string().substr(0, 4)) + 1;
    char prefix[16];
    std::snprintf(prefix, sizeof prefix, "%04d-", seq);
    const std::filesystem::path target = prev / (prefix + name);
    if (!std::filesystem::create_directory(target)) throw std::runtime_error(target.string() + " already exists");
    for (const std::string& file : files) {
        const std::filesystem::path source = room / file;
        if (!std::filesystem::exists(source)) continue;
        std::filesystem::copy_file(source, target / file, std::filesystem::copy_options::overwrite_existing);
        fsync_file(target / file);
    }
    for (const std::string& file : remove) {
        const std::filesystem::path path = room / file;
        if (::unlink(path.c_str()) != 0 && errno != ENOENT) throw std::runtime_error("cannot remove " + path.string());
    }
    existing.push_back(target);
    for (std::size_t n = 0; n + kArchiveKeep < existing.size(); ++n) std::filesystem::remove_all(existing[n]);
    return target;
}

}  // namespace room_memory
