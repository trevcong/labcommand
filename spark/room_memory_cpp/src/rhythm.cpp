#include "room_memory/rhythm.hpp"

#include "room_memory/messages.hpp"

#include <zlib.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <mutex>
#include <pwd.h>
#include <string_view>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <utility>

namespace room_memory {
namespace {

constexpr int kInt16Min = -32768;
constexpr int kInt16Max = 32767;
constexpr std::uint32_t kU16Max = 65535;
constexpr std::uint32_t kU32Max = 0xFFFFFFFFu;

void log_line(const std::string& message) { std::fprintf(stderr, "rhythm: %s\n", message.c_str()); }

std::string error_text(int err) { return std::generic_category().message(err); }

bool plain_name(const std::string& text) {
    const auto ok = [](char c, bool first) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
               (!first && (c == '.' || c == '_' || c == '-'));
    };
    if (text.empty() || !ok(text[0], true)) return false;
    for (std::size_t n = 1; n < text.size(); ++n) {
        if (!ok(text[n], false)) return false;
    }
    return true;
}

std::filesystem::path expand_user(const std::string& path) {
    if (path.empty() || path[0] != '~') return path;
    const std::size_t slash = path.find('/');
    const std::string user = path.substr(1, slash == std::string::npos ? std::string::npos : slash - 1);
    std::string home;
    if (user.empty()) {
        if (const char* env = std::getenv("HOME")) home = env;
        else if (const passwd* entry = getpwuid(getuid())) home = entry->pw_dir;
    } else if (const passwd* entry = getpwnam(user.c_str())) {
        home = entry->pw_dir;
    }
    if (home.empty()) return path;
    while (home.size() > 1 && home.back() == '/') home.pop_back();
    return home + (slash == std::string::npos ? std::string() : path.substr(slash));
}

std::uint32_t f32_bits(double value) { return std::bit_cast<std::uint32_t>(static_cast<float>(value)); }

void put_u16(std::vector<std::uint8_t>& buf, std::size_t at, std::uint16_t value) {
    buf[at] = static_cast<std::uint8_t>(value);
    buf[at + 1] = static_cast<std::uint8_t>(value >> 8);
}
void put_u32(std::vector<std::uint8_t>& buf, std::size_t at, std::uint32_t value) {
    for (int b = 0; b < 4; ++b) buf[at + static_cast<std::size_t>(b)] = static_cast<std::uint8_t>(value >> (8 * b));
}
void put_u64(std::vector<std::uint8_t>& buf, std::size_t at, std::uint64_t value) {
    for (int b = 0; b < 8; ++b) buf[at + static_cast<std::size_t>(b)] = static_cast<std::uint8_t>(value >> (8 * b));
}
void put_i16(std::vector<std::uint8_t>& buf, std::size_t at, std::int16_t value) {
    put_u16(buf, at, static_cast<std::uint16_t>(value));
}
void put_i64(std::vector<std::uint8_t>& buf, std::size_t at, std::int64_t value) {
    put_u64(buf, at, static_cast<std::uint64_t>(value));
}

std::uint16_t get_u16(std::span<const std::uint8_t> data, std::size_t at) {
    return static_cast<std::uint16_t>(data[at] | (data[at + 1] << 8));
}
std::uint32_t get_u32(std::span<const std::uint8_t> data, std::size_t at) {
    std::uint32_t value = 0;
    for (int b = 0; b < 4; ++b) value |= std::uint32_t{data[at + static_cast<std::size_t>(b)]} << (8 * b);
    return value;
}
std::uint64_t get_u64(std::span<const std::uint8_t> data, std::size_t at) {
    std::uint64_t value = 0;
    for (int b = 0; b < 8; ++b) value |= std::uint64_t{data[at + static_cast<std::size_t>(b)]} << (8 * b);
    return value;
}
std::int16_t get_i16(std::span<const std::uint8_t> data, std::size_t at) {
    return static_cast<std::int16_t>(get_u16(data, at));
}
std::int64_t get_i64(std::span<const std::uint8_t> data, std::size_t at) {
    return static_cast<std::int64_t>(get_u64(data, at));
}

std::uint32_t sat_u32(std::uint32_t total, std::uint32_t extra) {
    return extra > kU32Max - total ? kU32Max : total + extra;
}
int sat_u16(int value) { return value >= static_cast<int>(kU16Max) ? static_cast<int>(kU16Max) : value; }

std::int64_t floor_div(std::int64_t a, std::int64_t b) {
    std::int64_t q = a / b;
    const std::int64_t r = a % b;
    if (r != 0 && ((a < 0) != (b < 0))) --q;
    return q;
}

int sensor_code(const std::string& name) {
    if (name == "mid360") return 1;
    if (name == "jt128") return 2;
    throw std::invalid_argument("rhythm sensor '" + name + "' is not mid360 or jt128");
}
std::string sensor_name_of(int code) {
    if (code == 2) return "jt128";
    return "mid360";
}

struct Tm {
    int year;
    int month;
    int day;
};
Tm utc_tm(std::int64_t unix_seconds) {
    const std::time_t sec = static_cast<std::time_t>(unix_seconds);
    std::tm out{};
    gmtime_r(&sec, &out);
    return {out.tm_year + 1900, out.tm_mon + 1, out.tm_mday};
}
int yyyymmdd_now() {
    const Tm now = utc_tm(static_cast<std::int64_t>(std::time(nullptr)));
    return now.year * 10000 + now.month * 100 + now.day;
}
std::string day_string(std::int64_t minute_unix) {
    const Tm day = utc_tm(minute_unix);
    char text[16];
    std::snprintf(text, sizeof text, "%04d%02d%02d", day.year, day.month, day.day);
    return text;
}
int shift_yyyymmdd(int yyyymmdd, int days_back) {
    const std::chrono::year_month_day ymd{std::chrono::year{yyyymmdd / 10000},
                                          std::chrono::month{static_cast<unsigned>((yyyymmdd / 100) % 100)},
                                          std::chrono::day{static_cast<unsigned>(yyyymmdd % 100)}};
    const std::chrono::year_month_day shifted{std::chrono::sys_days{ymd} - std::chrono::days{days_back}};
    return static_cast<int>(shifted.year()) * 10000 + static_cast<unsigned>(shifted.month()) * 100 +
           static_cast<unsigned>(shifted.day());
}

struct ParsedName {
    std::string sensor;
    int date = 0;
    int index = 1;
};
bool all_digits(std::string_view text) {
    return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
}
std::optional<ParsedName> parse_segment_name(const std::string& name) {
    constexpr std::string_view kPrefix = "rhythm-";
    constexpr std::string_view kSuffix = ".rhy";
    if (name.size() <= kPrefix.size() + kSuffix.size() || name.compare(0, kPrefix.size(), kPrefix) != 0 ||
        name.compare(name.size() - kSuffix.size(), kSuffix.size(), kSuffix) != 0) {
        return std::nullopt;
    }
    const std::string body = name.substr(kPrefix.size(), name.size() - kPrefix.size() - kSuffix.size());
    std::string sensor;
    if (body.rfind("mid360-", 0) == 0) sensor = "mid360";
    else if (body.rfind("jt128-", 0) == 0) sensor = "jt128";
    else return std::nullopt;
    const std::string rest = body.substr(sensor.size() + 1);
    try {
        if (rest.size() == 8 && all_digits(rest)) return ParsedName{sensor, std::stoi(rest), 1};
        if (rest.size() > 9 && rest[8] == '-' && all_digits(std::string_view(rest).substr(0, 8)) &&
            all_digits(std::string_view(rest).substr(9))) {
            return ParsedName{sensor, std::stoi(rest.substr(0, 8)), std::stoi(rest.substr(9))};
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return std::nullopt;
}
std::optional<ParsedName> parse_quarantine_name(const std::string& name) {
    const auto match = [&](const char* kind) -> std::optional<ParsedName> {
        const std::string token = std::string(".") + kind + "-";
        const auto at = name.rfind(token);
        if (at == std::string::npos) return std::nullopt;
        const std::string tail = name.substr(at + token.size());
        if (!all_digits(tail)) return std::nullopt;
        return parse_segment_name(name.substr(0, at));
    };
    if (auto parsed = match("superseded")) return parsed;
    return match("bad");
}
std::string segment_name(const std::string& sensor, std::int64_t minute, int index) {
    const std::string day = day_string(minute);
    if (index <= 1) return "rhythm-" + sensor + "-" + day + ".rhy";
    return "rhythm-" + sensor + "-" + day + "-" + std::to_string(index) + ".rhy";
}
std::pair<int, int> segment_sort_key(const std::string& name) {
    if (const auto parsed = parse_segment_name(name)) return {parsed->date, parsed->index};
    return {0, 0};
}

struct Cell {
    std::int16_t ix = 0;
    std::int16_t iy = 0;
    std::int16_t iz = 0;
    std::uint64_t count = 0;
};

// 4096 open-addressed slots. Allocated once. clear() does not allocate.
struct Table {
    explicit Table(int slots) : slots(slots), mask(slots - 1) {
        if (slots < 1 || (slots & (slots - 1)) != 0) throw std::invalid_argument("rhythm table length must be a power of two");
        used.assign(static_cast<std::size_t>(slots), 0);
        ix.assign(static_cast<std::size_t>(slots), 0);
        iy.assign(static_cast<std::size_t>(slots), 0);
        iz.assign(static_cast<std::size_t>(slots), 0);
        hits.assign(static_cast<std::size_t>(slots), 0);
        sums.assign(static_cast<std::size_t>(slots), 0);
        cix.assign(static_cast<std::size_t>(slots), 0);
        ciy.assign(static_cast<std::size_t>(slots), 0);
        ciz.assign(static_cast<std::size_t>(slots), 0);
        cslot.assign(static_cast<std::size_t>(slots), 0);
    }
    void clear() {
        // A minute seal. Keep a cache that lists the whole table so the next identical
        // window writes those slots back. A partial cache is not a valid empty-table
        // layout: a key stored past a neighbor would sit behind an empty slot.
        // count == 0 means this table was already cleared; dropping the cache then
        // would make the next minute probe the full table again.
        if (count != 0 && cached != count) cached = 0;
        std::fill(used.begin(), used.end(), 0);
        std::fill(hits.begin(), hits.end(), 0);
        std::fill(sums.begin(), sums.end(), 0);
        count = 0;
    }
    int insert_window(std::span<const Cell> cells) {
        const int total = static_cast<int>(cells.size());
        if (total == 0) return 0;
        int begin = 0;
        if (cached > 0 && total >= cached && same_cache(cells)) {
            if (count == 0) {
                reoccupy(cells);
            } else if (count == cached) {
                bump(cells);
            } else {
                cached = 0;
            }
            if (cached > 0) {
                if (total == cached) return 0;
                // The prefix is the entire table and it is full. Every later cell misses.
                if (count >= slots) return total - cached;
                begin = cached;
            }
        }
        for (int n = begin; n < total; ++n) {
            // Unique cells, and every occupied slot is one of this window's first n.
            // The next cell is absent. Searching for it would scan the whole table.
            if (count >= slots && n == count) {
                remember(cells, n);
                return total - n;
            }
            const Cell& cell = cells[static_cast<std::size_t>(n)];
            const int slot = probe(cell.ix, cell.iy, cell.iz, rhythm_hash_slot(cell.ix, cell.iy, cell.iz, slots));
            if (slot < 0) {
                if (n > 0) remember(cells, n);
                return total - n;
            }
            if (n < slots) cslot[static_cast<std::size_t>(n)] = slot;
            const std::uint32_t updated = hits[static_cast<std::size_t>(slot)] + 1;
            hits[static_cast<std::size_t>(slot)] = std::min(kU16Max, updated);
            sums[static_cast<std::size_t>(slot)] += cell.count;
        }
        if (total <= slots) remember(cells, total);
        else cached = 0;
        return 0;
    }
    bool holds(int slot, std::int16_t x, std::int16_t y, std::int16_t z) const {
        if (slot < 0 || slot >= slots || !used[static_cast<std::size_t>(slot)]) return false;
        return ix[static_cast<std::size_t>(slot)] == x && iy[static_cast<std::size_t>(slot)] == y &&
               iz[static_cast<std::size_t>(slot)] == z;
    }
    std::vector<RhythmRegion> regions(int with_points) const {
        if (with_points <= 0) return {};
        std::vector<RhythmRegion> out;
        for (int n = 0; n < slots; ++n) {
            if (!used[static_cast<std::size_t>(n)] || hits[static_cast<std::size_t>(n)] == 0) continue;
            const std::uint64_t denom = static_cast<std::uint64_t>(with_points);
            std::uint64_t mean = (sums[static_cast<std::size_t>(n)] + denom / 2) / denom;
            if (mean > kU16Max) mean = kU16Max;
            RhythmRegion region;
            region.ix = ix[static_cast<std::size_t>(n)];
            region.iy = iy[static_cast<std::size_t>(n)];
            region.iz = iz[static_cast<std::size_t>(n)];
            region.hits = static_cast<std::uint16_t>(std::min(kU16Max, hits[static_cast<std::size_t>(n)]));
            region.mean = static_cast<std::uint16_t>(mean);
            out.push_back(region);
        }
        std::sort(out.begin(), out.end(), [](const RhythmRegion& a, const RhythmRegion& b) {
            if (a.ix != b.ix) return a.ix < b.ix;
            if (a.iy != b.iy) return a.iy < b.iy;
            return a.iz < b.iz;
        });
        return out;
    }

    int slots = 0;
    int mask = 0;
    int count = 0;
    int cached = 0;
    std::vector<std::uint8_t> used;
    std::vector<std::int16_t> ix, iy, iz, cix, ciy, ciz;
    std::vector<std::uint32_t> hits;
    std::vector<std::uint64_t> sums;
    std::vector<int> cslot;

private:
    bool same_cache(std::span<const Cell> cells) const {
        for (int n = 0; n < cached; ++n) {
            const Cell& cell = cells[static_cast<std::size_t>(n)];
            if (cell.ix != cix[static_cast<std::size_t>(n)] || cell.iy != ciy[static_cast<std::size_t>(n)] ||
                cell.iz != ciz[static_cast<std::size_t>(n)]) {
                return false;
            }
        }
        return true;
    }
    void reoccupy(std::span<const Cell> cells) {
        for (int n = 0; n < cached; ++n) {
            const std::size_t index = static_cast<std::size_t>(n);
            const std::size_t slot = static_cast<std::size_t>(cslot[index]);
            used[slot] = 1;
            ix[slot] = cix[index];
            iy[slot] = ciy[index];
            iz[slot] = ciz[index];
            hits[slot] = 1;
            sums[slot] = cells[index].count;
        }
        count = cached;
    }
    void bump(std::span<const Cell> cells) {
        for (int n = 0; n < cached; ++n) {
            const std::size_t index = static_cast<std::size_t>(n);
            const std::size_t slot = static_cast<std::size_t>(cslot[index]);
            const std::uint32_t updated = hits[slot] + 1;
            hits[slot] = std::min(kU16Max, updated);
            sums[slot] += cells[index].count;
        }
    }
    void remember(std::span<const Cell> cells, int ncells) {
        if (ncells <= 0 || ncells > slots) {
            cached = 0;
            return;
        }
        for (int n = 0; n < ncells; ++n) {
            const std::size_t index = static_cast<std::size_t>(n);
            cix[index] = cells[index].ix;
            ciy[index] = cells[index].iy;
            ciz[index] = cells[index].iz;
        }
        cached = ncells;
    }
    int probe(std::int16_t x, std::int16_t y, std::int16_t z, int start) {
        for (int step = 0; step < slots; ++step) {
            const int slot = (start + step) & mask;
            if (!used[static_cast<std::size_t>(slot)]) {
                if (count >= slots) return -1;
                used[static_cast<std::size_t>(slot)] = 1;
                ix[static_cast<std::size_t>(slot)] = x;
                iy[static_cast<std::size_t>(slot)] = y;
                iz[static_cast<std::size_t>(slot)] = z;
                hits[static_cast<std::size_t>(slot)] = 0;
                sums[static_cast<std::size_t>(slot)] = 0;
                ++count;
                return slot;
            }
            if (ix[static_cast<std::size_t>(slot)] == x && iy[static_cast<std::size_t>(slot)] == y &&
                iz[static_cast<std::size_t>(slot)] == z) {
                return slot;
            }
        }
        return -1;
    }
};

struct Bin {
    std::int64_t minute = 0;
    std::uint64_t map_epoch = 0;
    int closed = 0;
    int with_points = 0;
    int unavailable = 0;
    std::uint32_t clipped = 0;
    std::uint32_t regions_dropped = 0;
    bool pose_bad = false;
    bool discontinuity = false;
    bool overflow = false;
};

std::vector<Cell> cells_of(std::span<const Key> keys, int voxels, int dims, std::uint32_t& clipped) {
    clipped = 0;
    if (keys.empty()) return {};
    std::vector<Key> ordered(keys.begin(), keys.end());
    if (!std::is_sorted(ordered.begin(), ordered.end())) std::sort(ordered.begin(), ordered.end());
    std::vector<Cell> cells;
    cells.reserve(ordered.size());
    for (const Key key : ordered) {
        const Index3 v = unpack(key);
        const std::int64_t ix = floor_div(v.i, voxels);
        const std::int64_t iy = floor_div(v.j, voxels);
        const std::int64_t iz = dims == 2 ? 0 : floor_div(v.k, voxels);
        if (ix < kInt16Min || ix > kInt16Max || iy < kInt16Min || iy > kInt16Max || iz < kInt16Min || iz > kInt16Max) {
            clipped = sat_u32(clipped, 1);
            continue;
        }
        cells.push_back(Cell{static_cast<std::int16_t>(ix), static_cast<std::int16_t>(iy), static_cast<std::int16_t>(iz), 1});
    }
    if (cells.size() > 1) {
        std::sort(cells.begin(), cells.end(), [](const Cell& a, const Cell& b) {
            if (a.ix != b.ix) return a.ix < b.ix;
            if (a.iy != b.iy) return a.iy < b.iy;
            return a.iz < b.iz;
        });
        std::vector<Cell> merged;
        merged.reserve(cells.size());
        for (const Cell& cell : cells) {
            if (!merged.empty() && merged.back().ix == cell.ix && merged.back().iy == cell.iy && merged.back().iz == cell.iz) {
                merged.back().count += cell.count;
            } else {
                merged.push_back(cell);
            }
        }
        cells = std::move(merged);
    }
    return cells;
}

struct SensorSlot {
    std::string name;
    std::optional<Bin> bin;
    Table table;
    std::optional<std::pair<std::int64_t, std::uint64_t>> loss;
    bool force_disc = false;
    bool explode = false;
    explicit SensorSlot(std::string name, int slots) : name(std::move(name)), table(slots) {}
};

}  // namespace

int rhythm_hash_slot(std::int16_t ix, std::int16_t iy, std::int16_t iz, int slots) {
    // Widen before the shift. A uint16 shifted by 32 is zero.
    const std::uint64_t packed = (std::uint64_t{static_cast<std::uint16_t>(ix)} << 32) |
                                 (std::uint64_t{static_cast<std::uint16_t>(iy)} << 16) |
                                 std::uint64_t{static_cast<std::uint16_t>(iz)};
    return static_cast<int>(splitmix64(packed) & static_cast<std::uint64_t>(slots - 1));
}

std::uint32_t rhythm_crc32(std::span<const std::uint8_t> data) {
    const auto* bytes = reinterpret_cast<const Bytef*>(data.data());
    return static_cast<std::uint32_t>(::crc32(0L, bytes, static_cast<uInt>(data.size())));
}

int voxels_per_cell(double cell_m, double voxel_size) {
    const float cell = static_cast<float>(cell_m);
    const float voxel = static_cast<float>(voxel_size);
    if (!std::isfinite(cell) || !std::isfinite(voxel) || !(cell > 0) || !(voxel > 0)) {
        throw std::invalid_argument("rhythm_cell_m and voxel_size must be finite and > 0");
    }
    const float ratio = cell / voxel;
    if (!std::isfinite(ratio)) throw std::invalid_argument("rhythm_cell_m / voxel_size is not a finite float32");
    const float nearest = std::nearbyint(ratio);
    const int whole = static_cast<int>(nearest);
    if (ratio != nearest || whole < 1 || whole > 64) {
        throw std::invalid_argument("rhythm_cell_m / voxel_size must be an integer in 1...64 as float32, got " +
                                    std::to_string(ratio));
    }
    return whole;
}

int windows_per_period(std::int64_t period_s, double window_s) {
    if (period_s < 1 || period_s > 65535) throw std::invalid_argument("rhythm_period_s must be an integer in 1...65535");
    if (!(std::isfinite(window_s) && window_s > 0)) throw std::invalid_argument("window_s must be a finite number > 0 s");
    const double quotient = static_cast<double>(period_s) / window_s;
    const int windows = static_cast<int>(std::llround(quotient));
    if (windows < 1 || windows > 3600 || std::abs(quotient - windows) > 1e-9) {
        throw std::invalid_argument("rhythm_period_s / window_s must be an integer in 1...3600");
    }
    return windows;
}

void validate_rhythm(const RhythmSettings& settings) {
    if (settings.dims != 2 && settings.dims != 3) throw std::invalid_argument("rhythm_dims must be 2 or 3");
    // The cell and the period have to divide the detection grid only while the logger is on.
    // A disabled logger must not refuse a voxel_size or window_s that already started the node.
    // The period and the cell still have to be in range on their own.
    if (settings.enable) {
        voxels_per_cell(settings.cell_m, settings.voxel_size);
        windows_per_period(settings.period_s, settings.window_s);
    } else {
        if (settings.period_s < 1 || settings.period_s > 65535) {
            throw std::invalid_argument("rhythm_period_s must be an integer in 1...65535");
        }
        const float cell = static_cast<float>(settings.cell_m);
        if (!std::isfinite(cell) || !(cell > 0)) {
            throw std::invalid_argument("rhythm_cell_m must be finite and > 0");
        }
    }
    if (settings.keep_days < 0) throw std::invalid_argument("rhythm_keep_days must be an integer >= 0 (0: no age cap)");
    if (settings.max_bytes < 0) throw std::invalid_argument("rhythm_max_bytes must be an integer >= 0 (0: no byte cap)");
    if (settings.max_file_bytes < 0) throw std::invalid_argument("rhythm_max_file_bytes must be an integer >= 0");
    if (settings.max_file_bytes != 0 && settings.max_file_bytes < 65536) {
        throw std::invalid_argument("rhythm_max_file_bytes must be 0 or at least 65536");
    }
    if (settings.max_bytes > 0 && settings.max_file_bytes > settings.max_bytes) {
        throw std::invalid_argument("rhythm_max_file_bytes must be <= rhythm_max_bytes");
    }
    if (settings.fsync_s < 0) throw std::invalid_argument("rhythm_fsync_s must be an integer >= 0 (0: no periodic fsync)");
    if ((!settings.dir.empty() || settings.state_dir) && !plain_name(settings.room_id)) {
        throw std::invalid_argument("room_id '" + settings.room_id +
                                    "' is not a plain file name ([A-Za-z0-9._-], no leading dot)");
    }
}

std::optional<std::filesystem::path> rhythm_directory(const RhythmSettings& settings) {
    const std::string base = !settings.dir.empty() ? settings.dir : settings.state_dir.value_or("");
    if (base.empty()) return std::nullopt;
    if (!plain_name(settings.room_id)) {
        throw std::invalid_argument("room_id '" + settings.room_id +
                                    "' is not a plain file name ([A-Za-z0-9._-], no leading dot)");
    }
    return expand_user(base) / settings.room_id / "rhythm";
}

bool rhythm_active(const RhythmSettings& settings) {
    if (!settings.enable) return false;
    return rhythm_directory(settings).has_value() || settings.publish;
}

std::vector<std::uint8_t> encode_header(int sensor, int dims, int voxels, double voxel_size, double cell_m,
                                        std::uint32_t period_s, const std::string& room_id) {
    if (room_id.size() > kU16Max) throw std::invalid_argument("room_id is longer than uint16");
    const std::size_t prefix = 26 + room_id.size();
    const std::size_t padded = (prefix + 3) & ~std::size_t{3};
    const std::size_t header_bytes = padded + 4;
    std::vector<std::uint8_t> buf(header_bytes, 0);
    buf[0] = 'R';
    buf[1] = 'H';
    buf[2] = 'Y';
    buf[3] = '1';
    put_u16(buf, 4, 1);
    put_u16(buf, 6, static_cast<std::uint16_t>(header_bytes));
    buf[8] = static_cast<std::uint8_t>(sensor);
    buf[9] = static_cast<std::uint8_t>(dims);
    put_u16(buf, 10, static_cast<std::uint16_t>(voxels));
    put_u32(buf, 12, f32_bits(voxel_size));
    put_u32(buf, 16, f32_bits(cell_m));
    put_u32(buf, 20, period_s);
    put_u16(buf, 24, static_cast<std::uint16_t>(room_id.size()));
    std::memcpy(buf.data() + 26, room_id.data(), room_id.size());
    put_u32(buf, padded, rhythm_crc32(std::span<const std::uint8_t>(buf.data(), padded)));
    return buf;
}

std::optional<RhythmIdentity> header_identity(std::span<const std::uint8_t> data) {
    if (data.size() < 28 || data[0] != 'R' || data[1] != 'H' || data[2] != 'Y' || data[3] != '1') return std::nullopt;
    const int version = get_u16(data, 4);
    const int header_bytes = get_u16(data, 6);
    if (version != 1 || header_bytes < 28 || header_bytes % 4 || static_cast<std::size_t>(header_bytes) > data.size()) {
        return std::nullopt;
    }
    if (rhythm_crc32(data.first(static_cast<std::size_t>(header_bytes) - 4)) !=
        get_u32(data, static_cast<std::size_t>(header_bytes) - 4)) {
        return std::nullopt;
    }
    const int id_len = get_u16(data, 24);
    if (26 + id_len > header_bytes - 4) return std::nullopt;
    RhythmIdentity parsed;
    parsed.header_bytes = header_bytes;
    parsed.sensor = data[8];
    parsed.dims = data[9];
    parsed.voxels_per_cell = get_u16(data, 10);
    parsed.voxel_bits = get_u32(data, 12);
    parsed.cell_bits = get_u32(data, 16);
    parsed.period_s = get_u32(data, 20);
    parsed.room_id.assign(reinterpret_cast<const char*>(data.data() + 26), static_cast<std::size_t>(id_len));
    return parsed;
}

bool identity_matches(const RhythmIdentity& parsed, const RhythmIdentity& expected) {
    return parsed.room_id == expected.room_id && parsed.sensor == expected.sensor && parsed.dims == expected.dims &&
           parsed.voxels_per_cell == expected.voxels_per_cell && parsed.voxel_bits == expected.voxel_bits &&
           parsed.cell_bits == expected.cell_bits && parsed.period_s == expected.period_s;
}

std::vector<std::uint8_t> encode_record(std::int64_t minute, std::uint64_t map_epoch, int closed, int with_points,
                                        int unavailable, std::uint32_t clipped, std::uint32_t dropped, int pose_ok,
                                        int flags, std::span<const RhythmRegion> regions) {
    if (regions.size() > kU16Max) throw std::invalid_argument("n_regions does not fit in uint16");
    const std::size_t body = kEnvelopeBytes + regions.size() * static_cast<std::size_t>(kRegionBytes);
    std::vector<std::uint8_t> buf(body, 0);
    put_u32(buf, 0, static_cast<std::uint32_t>(body));
    put_i64(buf, 4, minute);
    put_u64(buf, 12, map_epoch);
    put_u16(buf, 20, static_cast<std::uint16_t>(closed));
    put_u16(buf, 22, static_cast<std::uint16_t>(with_points));
    put_u16(buf, 24, static_cast<std::uint16_t>(unavailable));
    put_u32(buf, 28, clipped);
    put_u32(buf, 32, dropped);
    buf[36] = static_cast<std::uint8_t>(pose_ok);
    buf[37] = static_cast<std::uint8_t>(flags);
    put_u16(buf, 38, static_cast<std::uint16_t>(regions.size()));
    std::optional<std::array<int, 3>> previous;
    std::size_t offset = kEnvelopeBytes;
    for (const RhythmRegion& region : regions) {
        const std::array<int, 3> current{region.ix, region.iy, region.iz};
        if (previous && current <= *previous) throw std::invalid_argument("regions must be strictly increasing");
        previous = current;
        put_i16(buf, offset, region.ix);
        put_i16(buf, offset + 2, region.iy);
        put_i16(buf, offset + 4, region.iz);
        put_u16(buf, offset + 6, region.hits);
        put_u16(buf, offset + 8, region.mean);
        offset += kRegionBytes;
    }
    const std::uint32_t crc = rhythm_crc32(std::span<const std::uint8_t>(buf.data() + 4, body - 4));
    buf.resize(body + 4);
    put_u32(buf, body, crc);
    return buf;
}

std::vector<std::uint8_t> encode_marker(std::int64_t minute, std::uint64_t map_epoch) {
    return encode_record(minute, map_epoch, 0, 0, 0, 0, 0, 0, kFlagDiscontinuity | kFlagLost, {});
}

std::optional<RhythmRecord> parse_record(std::span<const std::uint8_t> data) {
    if (data.size() < 4 + kEnvelopeBytes) return std::nullopt;
    const std::uint32_t body = get_u32(data, 0);
    if (body < kEnvelopeBytes || 4u + body != data.size() || (body - kEnvelopeBytes) % kRegionBytes) return std::nullopt;
    if (rhythm_crc32(data.subspan(4, data.size() - 8)) != get_u32(data, data.size() - 4)) return std::nullopt;
    RhythmRecord record;
    record.minute = get_i64(data, 4);
    record.map_epoch = get_u64(data, 12);
    record.closed = get_u16(data, 20);
    record.with_points = get_u16(data, 22);
    record.unavailable = get_u16(data, 24);
    record.pad = get_u16(data, 26);
    record.clipped = get_u32(data, 28);
    record.regions_dropped = get_u32(data, 32);
    record.pose_ok = data[36];
    record.flags = data[37];
    const int count = get_u16(data, 38);
    if (record.pad != 0 || kEnvelopeBytes + count * kRegionBytes != static_cast<int>(body)) return std::nullopt;
    std::optional<std::array<int, 3>> previous;
    std::size_t offset = kEnvelopeBytes;
    record.regions.reserve(static_cast<std::size_t>(count));
    for (int n = 0; n < count; ++n) {
        RhythmRegion region;
        region.ix = get_i16(data, offset);
        region.iy = get_i16(data, offset + 2);
        region.iz = get_i16(data, offset + 4);
        region.hits = get_u16(data, offset + 6);
        region.mean = get_u16(data, offset + 8);
        if (region.hits == 0) return std::nullopt;
        const std::array<int, 3> current{region.ix, region.iy, region.iz};
        if (previous && current <= *previous) return std::nullopt;
        previous = current;
        record.regions.push_back(region);
        offset += kRegionBytes;
    }
    return record;
}

std::optional<std::pair<std::size_t, std::optional<std::int64_t>>> scan_records(std::span<const std::uint8_t> data) {
    const auto parsed = header_identity(data);
    if (!parsed) return std::nullopt;
    std::size_t offset = static_cast<std::size_t>(parsed->header_bytes);
    std::optional<std::int64_t> last;
    while (offset + 4 <= data.size()) {
        const std::uint32_t body = get_u32(data, offset);
        const std::size_t total = 4u + body;
        if (body < kEnvelopeBytes || offset + total > data.size()) break;
        const auto record = parse_record(data.subspan(offset, total));
        if (!record) break;
        last = record->minute;
        offset += total;
    }
    return std::pair<std::size_t, std::optional<std::int64_t>>{offset, last};
}

struct RhythmAccumulator::State {
    int voxels = 1;
    int dims = 2;
    int period_s = 60;
    int windows_per = 60;
    int slots = kRegionSlots;
    int hook_errors = 0;
    std::vector<SensorSlot> sensors;
    std::vector<RhythmFragment> finished;

    SensorSlot& find(const std::string& name) {
        for (SensorSlot& sensor : sensors) {
            if (sensor.name == name) return sensor;
        }
        throw std::out_of_range("rhythm sensor '" + name + "' is not configured");
    }
    const SensorSlot& find(const std::string& name) const { return const_cast<State*>(this)->find(name); }

    std::optional<std::int64_t> minute_of(std::int64_t sec) const {
        if (sec < 0) return std::nullopt;
        return sec - sec % period_s;
    }

    void seal(SensorSlot& sensor) {
        if (!sensor.bin) return;
        Bin opened = *sensor.bin;
        if (opened.closed + opened.unavailable == 0) {
            sensor.bin.reset();
            sensor.table.clear();
            return;
        }
        int flags = 0;
        if (opened.closed < windows_per) flags |= kFlagPartial;
        if (opened.discontinuity) flags |= kFlagDiscontinuity;
        if (opened.unavailable) flags |= kFlagUnavailable;
        if (opened.overflow || opened.regions_dropped) flags |= kFlagOverflow;
        const int pose_ok = opened.with_points == 0 || opened.pose_bad ? 0 : 1;
        const std::vector<RhythmRegion> regions = sensor.table.regions(opened.with_points);
        // Build the record before dropping the bin. A failure leaves it for discard.
        auto data = encode_record(opened.minute, opened.map_epoch, opened.closed, opened.with_points, opened.unavailable,
                                  opened.clipped, opened.regions_dropped, pose_ok, flags, regions);
        sensor.bin.reset();
        sensor.table.clear();
        finished.push_back(RhythmFragment{sensor.name, opened.minute, opened.map_epoch, static_cast<std::uint32_t>(flags),
                                          std::move(data)});
    }

    Bin* advance(SensorSlot& sensor, std::int64_t sec, std::uint64_t map_epoch) {
        const auto minute = minute_of(sec);
        if (!minute) return nullptr;
        if (!sensor.bin) {
            sensor.bin = Bin{*minute, map_epoch, 0, 0, 0, 0, 0, false, sensor.force_disc, false};
            sensor.force_disc = false;
            return &*sensor.bin;
        }
        Bin& opened = *sensor.bin;
        if (*minute == opened.minute ||
            (*minute + period_s == opened.minute && sec < opened.minute && opened.minute - sec <= kGraceSeconds)) {
            opened.map_epoch = map_epoch;
            return &opened;
        }
        const bool gap = *minute > opened.minute + period_s || *minute < opened.minute;
        seal(sensor);
        sensor.bin = Bin{*minute, map_epoch, 0, 0, 0, 0, 0, false, gap || sensor.force_disc, false};
        sensor.force_disc = false;
        return &*sensor.bin;
    }
};

RhythmAccumulator::RhythmAccumulator(std::vector<std::string> sensors, int voxels, int dims, int period_s, int windows_per,
                                     int slots)
    : state_(std::make_unique<State>()) {
    state_->voxels = voxels;
    state_->dims = dims;
    state_->period_s = period_s;
    state_->windows_per = windows_per;
    state_->slots = slots;
    for (std::string& name : sensors) state_->sensors.emplace_back(std::move(name), slots);
}
RhythmAccumulator::~RhythmAccumulator() = default;
RhythmAccumulator::RhythmAccumulator(RhythmAccumulator&&) noexcept = default;
RhythmAccumulator& RhythmAccumulator::operator=(RhythmAccumulator&&) noexcept = default;

void RhythmAccumulator::add(const std::string& sensor, std::int64_t sec, std::span<const Key> keys, std::uint64_t map_epoch,
                            bool tracking, std::optional<double> residual, std::optional<double> learn_max_offset_m) {
    SensorSlot& slot = state_->find(sensor);
    Bin* opened = state_->advance(slot, sec, map_epoch);
    if (!opened) return;
    const bool pose_ok = tracking && residual && learn_max_offset_m && *residual <= *learn_max_offset_m;
    if (!pose_ok) opened->pose_bad = true;
    opened->closed = sat_u16(opened->closed + 1);
    opened->with_points = sat_u16(opened->with_points + 1);
    if (slot.explode) {
        slot.explode = false;
        throw std::runtime_error("rhythm insert failed");
    }
    std::uint32_t clipped = 0;
    const std::vector<Cell> cells = cells_of(keys, state_->voxels, state_->dims, clipped);
    opened->clipped = sat_u32(opened->clipped, clipped);
    const int missed = slot.table.insert_window(cells);
    if (missed) {
        opened->overflow = true;
        opened->regions_dropped = sat_u32(opened->regions_dropped, static_cast<std::uint32_t>(missed));
    }
}
void RhythmAccumulator::add_empty(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch) {
    SensorSlot& slot = state_->find(sensor);
    Bin* opened = state_->advance(slot, sec, map_epoch);
    if (!opened) return;
    opened->closed = sat_u16(opened->closed + 1);
}
void RhythmAccumulator::note_unavailable(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch) {
    SensorSlot& slot = state_->find(sensor);
    Bin* opened = state_->advance(slot, sec, map_epoch);
    if (!opened) return;
    opened->unavailable = sat_u16(opened->unavailable + 1);
}
void RhythmAccumulator::discard(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch) {
    SensorSlot& slot = state_->find(sensor);
    const auto minute = slot.bin ? std::optional<std::int64_t>(slot.bin->minute) : state_->minute_of(sec);
    const std::uint64_t epoch = slot.bin ? slot.bin->map_epoch : map_epoch;
    slot.bin.reset();
    slot.table.clear();
    if (minute) slot.loss = std::pair<std::int64_t, std::uint64_t>{*minute, epoch};
    slot.force_disc = true;
    ++state_->hook_errors;
}
void RhythmAccumulator::note_epoch(std::uint64_t map_epoch) {
    for (SensorSlot& sensor : state_->sensors) {
        if (sensor.bin) sensor.bin->map_epoch = map_epoch;
    }
}
void RhythmAccumulator::fail_next_insert(const std::string& sensor) { state_->find(sensor).explode = true; }
void RhythmAccumulator::set_force_disc(const std::string& sensor) { state_->find(sensor).force_disc = true; }
RhythmSensorView RhythmAccumulator::view(const std::string& sensor) const {
    const SensorSlot& slot = state_->find(sensor);
    RhythmSensorView out;
    out.open = slot.bin.has_value();
    out.with_points = slot.bin ? slot.bin->with_points : 0;
    out.loss = slot.loss.has_value();
    return out;
}
bool RhythmAccumulator::table_holds(const std::string& sensor, int slot, std::int16_t ix, std::int16_t iy,
                                    std::int16_t iz) const {
    return state_->find(sensor).table.holds(slot, ix, iy, iz);
}
std::vector<RhythmFragment> RhythmAccumulator::take() {
    std::vector<RhythmFragment> done;
    done.swap(state_->finished);
    return done;
}
std::vector<RhythmFragment> RhythmAccumulator::flush() {
    for (SensorSlot& sensor : state_->sensors) state_->seal(sensor);
    return take();
}
int RhythmAccumulator::hook_errors() const { return state_->hook_errors; }
std::optional<std::pair<std::int64_t, std::uint64_t>> RhythmAccumulator::loss_of(const std::string& sensor) const {
    return state_->find(sensor).loss;
}
void RhythmAccumulator::remember_loss(const std::string& sensor, std::int64_t minute, std::uint64_t epoch) {
    state_->find(sensor).loss = std::pair<std::int64_t, std::uint64_t>{minute, epoch};
}
void RhythmAccumulator::clear_loss(const std::string& sensor) { state_->find(sensor).loss.reset(); }

namespace {

struct OpenSegment {
    std::string sensor;
    std::filesystem::path path;
    int fd = -1;
    int day = 0;
    int index = 1;
    std::optional<std::int64_t> last_minute;
    bool abandoned = false;
};

struct BudgetFile {
    std::filesystem::path path;
    std::uintmax_t size = 0;
    bool quarantine = false;
    std::filesystem::file_time_type mtime{};
};

class Writer : public std::enable_shared_from_this<Writer> {
public:
    Writer(RhythmSettings settings, std::optional<std::filesystem::path> directory, RhythmStore store, Rhythm::Publish publish,
           std::function<bool()> would_block)
        : settings_(std::move(settings)),
          directory_(std::move(directory)),
          store_(std::move(store)),
          publish_(std::move(publish)),
          would_block_(std::move(would_block)),
          voxels_(voxels_per_cell(settings_.cell_m, settings_.voxel_size)) {
        if (!store_.write) {
            store_.write = [](int fd, const void* data, std::size_t n) { return ::write(fd, data, n); };
        }
        if (!store_.fsync) store_.fsync = [](int fd) { return ::fsync(fd); };
        if (!store_.truncate) {
            store_.truncate = [](int fd, std::int64_t length) { return ::ftruncate(fd, static_cast<off_t>(length)); };
        }
        if (!would_block_) would_block_ = [] { return false; };
        for (auto& slot : slots_) slot.resize(static_cast<std::size_t>(kMaxRecordBytes));
        for (const std::string& name : settings_.sensors) force_disc_[name] = false;
        if (directory_) prepare();
    }
    ~Writer() { close(); }

    void start() {
        if (thread_.joinable()) return;
        auto self = shared_from_this();
        thread_ = std::thread([self] { self->run(); });
    }
    bool force_disc(const std::string& sensor) const {
        std::lock_guard lock(mu_);
        const auto it = force_disc_.find(sensor);
        return it != force_disc_.end() && it->second;
    }
    void note_discontinuity(const std::string& sensor) {
        std::lock_guard lock(mu_);
        force_disc_[sensor] = true;
        ++disc_gen_[sensor];
    }
    void set_publish(Rhythm::Publish publish, std::function<bool()> would_block) {
        std::lock_guard lock(pub_mu_);
        publish_ = std::move(publish);
        if (would_block) would_block_ = std::move(would_block);
    }
    bool try_enqueue(const std::string& sensor, const std::vector<std::uint8_t>& data) {
        if (static_cast<int>(data.size()) > kMaxRecordBytes) {
            std::lock_guard lock(mu_);
            ++queue_dropped_;
            return false;
        }
        const int code = sensor_code(sensor);
        {
            std::lock_guard lock(mu_);
            if (count_ >= kQueueSlots) {
                ++queue_dropped_;
                return false;
            }
            std::memcpy(slots_[static_cast<std::size_t>(tail_)].data(), data.data(), data.size());
            lengths_[static_cast<std::size_t>(tail_)] = static_cast<int>(data.size());
            codes_[static_cast<std::size_t>(tail_)] = code;
            tail_ = (tail_ + 1) % kQueueSlots;
            ++count_;
        }
        cv_.notify_all();
        return true;
    }
    void close() {
        if (closed_.exchange(true)) return;
        shutting_down_.store(true);
        stop_.store(true);
        cv_.notify_all();
        if (!thread_.joinable()) return;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline && !finished_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (finished_.load()) thread_.join();
        else thread_.detach();
    }
    int budget_dropped() const { return budget_dropped_.load(); }

private:
    void prepare() {
        try {
            std::filesystem::create_directories(*directory_);
            recover();
            age_prune();
            last_age_day_ = yyyymmdd_now();
        } catch (const std::exception& error) {
            ++io_errors_;
            log_once("rhythm directory " + directory_->string() + " is unusable (" + error.what() +
                     "); detection will still start");
        }
    }
    void log_once(const std::string& message) {
        if (logged_.exchange(true)) return;
        log_line(message);
    }
    RhythmIdentity identity(const std::string& sensor) const {
        RhythmIdentity out;
        out.room_id = settings_.room_id;
        out.sensor = sensor_code(sensor);
        out.dims = settings_.dims;
        out.voxels_per_cell = voxels_;
        out.voxel_bits = f32_bits(settings_.voxel_size);
        out.cell_bits = f32_bits(settings_.cell_m);
        out.period_s = static_cast<std::uint32_t>(settings_.period_s);
        return out;
    }
    std::pair<std::optional<std::filesystem::path>, int> latest(const std::string& sensor, int day) const {
        std::optional<std::filesystem::path> best;
        int best_index = 0;
        std::error_code error;
        std::filesystem::directory_iterator entries(*directory_, error);
        if (error) return {std::nullopt, 1};
        for (const auto& entry : entries) {
            const auto parsed = parse_segment_name(entry.path().filename().string());
            if (!parsed || parsed->sensor != sensor || parsed->date != day) continue;
            if (parsed->index >= best_index) {
                best = entry.path();
                best_index = parsed->index;
            }
        }
        return {best, best_index};
    }
    void quarantine(const std::filesystem::path& path, const char* kind) {
        const auto stamp = static_cast<std::int64_t>(std::time(nullptr));
        std::filesystem::path target = path.parent_path() / (path.filename().string() + "." + kind + "-" + std::to_string(stamp));
        int step = 0;
        while (std::filesystem::exists(target)) {
            ++step;
            target = path.parent_path() / (path.filename().string() + "." + kind + "-" + std::to_string(stamp) + "-" +
                                           std::to_string(step));
        }
        std::filesystem::rename(path, target);
    }
    void recover() {
        const int today = yyyymmdd_now();
        for (const std::string& sensor : settings_.sensors) {
            std::optional<std::filesystem::path> path;
            int index = 0;
            try {
                std::tie(path, index) = latest(sensor, today);
            } catch (const std::exception& error) {
                ++io_errors_;
                log_once("rhythm recovery skipped (" + std::string(error.what()) + ")");
                force_disc_[sensor] = true;
                continue;
            }
            if (!path) continue;
            std::error_code error;
            const auto size = std::filesystem::file_size(*path, error);
            if (error) {
                ++io_errors_;
                log_once("rhythm recovery skipped " + path->string() + " (" + error.message() + ")");
                force_disc_[sensor] = true;
                continue;
            }
            if (size > static_cast<std::uintmax_t>(kRecoveryBytes)) {
                quarantine(*path, "superseded");
                force_disc_[sensor] = true;
                continue;
            }
            std::ifstream in(*path, std::ios::binary);
            std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            if (!in && data.empty()) {
                ++io_errors_;
                log_once("rhythm recovery skipped " + path->string() + " (read failed)");
                force_disc_[sensor] = true;
                continue;
            }
            const auto parsed = header_identity(data);
            if (!parsed) {
                quarantine(*path, "bad");
                force_disc_[sensor] = true;
                continue;
            }
            if (!identity_matches(*parsed, identity(sensor))) {
                quarantine(*path, "superseded");
                force_disc_[sensor] = true;
                continue;
            }
            const auto scanned = scan_records(data);
            if (!scanned) {
                quarantine(*path, "bad");
                force_disc_[sensor] = true;
                continue;
            }
            if (scanned->first < data.size()) {
                std::error_code truncate_error;
                std::filesystem::resize_file(*path, scanned->first, truncate_error);
                if (truncate_error) {
                    ++io_errors_;
                    log_once("rhythm could not truncate " + path->string() + " (" + truncate_error.message() + ")");
                    force_disc_[sensor] = true;
                    continue;
                }
                force_disc_[sensor] = true;
            }
            last_minute_[sensor] = scanned->second;
            resume_[sensor] = Resume{*path, index == 0 ? 1 : index, today};
        }
    }
    void run() {
        try {
            while (true) {
                {
                    std::unique_lock lock(mu_);
                    cv_.wait_for(lock, idle_timeout(), [&] { return stop_.load() || count_ > 0; });
                    if (stop_.load() && count_ == 0) break;
                }
                drain(!stop_.load());
                if (stop_.load()) continue;
                maybe_fsync();
                maybe_age_prune();
            }
            drain(false);
            fsync_open();
        } catch (const std::exception& error) {
            ++io_errors_;
            log_once(std::string("rhythm writer stopped (") + error.what() + ")");
        }
        finished_.store(true);
    }
    std::chrono::milliseconds idle_timeout() const {
        if (settings_.fsync_s <= 0) return std::chrono::seconds(30);
        const auto elapsed = std::chrono::steady_clock::now() - last_fsync_;
        const auto interval = std::chrono::seconds(settings_.fsync_s);
        if (elapsed + std::chrono::milliseconds(50) >= interval) return std::chrono::milliseconds(50);
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(interval - elapsed);
        if (remaining > std::chrono::seconds(30)) return std::chrono::seconds(30);
        return remaining;
    }
    struct Item {
        int code = 0;
        std::vector<std::uint8_t> data;
    };
    std::optional<Item> pop() {
        std::lock_guard lock(mu_);
        if (count_ == 0) return std::nullopt;
        Item item;
        item.code = codes_[static_cast<std::size_t>(head_)];
        const int length = lengths_[static_cast<std::size_t>(head_)];
        item.data.assign(slots_[static_cast<std::size_t>(head_)].begin(),
                         slots_[static_cast<std::size_t>(head_)].begin() + length);
        head_ = (head_ + 1) % kQueueSlots;
        --count_;
        return item;
    }
    void drain(bool prune) {
        while (true) {
            const auto item = pop();
            if (!item) return;
            append(sensor_name_of(item->code), item->data, prune);
        }
    }
    bool take_force(const std::string& sensor, std::uint64_t& generation) {
        std::lock_guard lock(mu_);
        generation = disc_gen_[sensor];
        return force_disc_[sensor];
    }
    void clear_force_if(const std::string& sensor, std::uint64_t generation) {
        std::lock_guard lock(mu_);
        if (disc_gen_[sensor] == generation) force_disc_[sensor] = false;
    }
    void set_force(const std::string& sensor) {
        std::lock_guard lock(mu_);
        force_disc_[sensor] = true;
        ++disc_gen_[sensor];
    }
    void append(const std::string& sensor, std::vector<std::uint8_t> data, bool prune) {
        auto record = parse_record(data);
        if (!record) {
            ++io_errors_;
            return;
        }
        if (!directory_) {
            publish(sensor, *record);
            return;
        }
        if (const auto segment = open_.find(sensor); segment != open_.end() && segment->second.abandoned) return;
        const auto last = last_minute_[sensor];
        if (last && record->minute == *last) set_force(sensor);
        std::uint64_t generation = 0;
        if (take_force(sensor, generation) && (record->flags & kFlagDiscontinuity) == 0) {
            record->flags = static_cast<std::uint8_t>(record->flags | kFlagDiscontinuity);
            data = encode_record(record->minute, record->map_epoch, record->closed, record->with_points, record->unavailable,
                                 record->clipped, record->regions_dropped, record->pose_ok, record->flags, record->regions);
            record = parse_record(data);
            if (!record) {
                ++io_errors_;
                return;
            }
        }
        if (!ready(sensor, *record, data.size(), prune)) return;
        if (!write_record(open_.at(sensor), data, *record)) return;
        clear_force_if(sensor, generation);
        last_minute_[sensor] = record->minute;
        open_.at(sensor).last_minute = record->minute;
        publish(sensor, *record);
    }
    bool ready(const std::string& sensor, const RhythmRecord& record, std::size_t size, bool prune) {
        const int day = std::stoi(day_string(record.minute));
        auto found = open_.find(sensor);
        if (found == open_.end()) {
            take_resume(sensor, day, size);
            found = open_.find(sensor);
        }
        bool rotate = found != open_.end() && found->second.day != day;
        if (found != open_.end() && !rotate && settings_.max_file_bytes) {
            const off_t used = ::lseek(found->second.fd, 0, SEEK_END);
            if (used >= 0 && static_cast<std::uint64_t>(used) + size > static_cast<std::uint64_t>(settings_.max_file_bytes)) {
                rotate = true;
            }
        }
        if (rotate && found != open_.end()) {
            sync_fd(found->second.fd);
            close_segment(sensor, false);
            found = open_.end();
        }
        const std::size_t header = found == open_.end() ? header_size(sensor) : 0;
        if (!fits(size + header, prune)) {
            ++budget_dropped_;
            return false;
        }
        if (found == open_.end() && !open_new(sensor, record.minute, day)) return false;
        return true;
    }
    void take_resume(const std::string& sensor, int day, std::size_t size) {
        const auto resumed = resume_.find(sensor);
        if (resumed == resume_.end()) return;
        const Resume saved = resumed->second;
        if (saved.day != day) {
            resume_.erase(resumed);
            return;
        }
        std::error_code error;
        const auto used = std::filesystem::file_size(saved.path, error);
        if (!error && settings_.max_file_bytes && used + size > static_cast<std::uint64_t>(settings_.max_file_bytes)) {
            resume_.erase(sensor);
            return;
        }
        resume_.erase(sensor);
        const int fd = ::open(saved.path.c_str(), O_RDWR);
        if (fd < 0 || ::lseek(fd, 0, SEEK_END) < 0) {
            if (fd >= 0) ::close(fd);
            ++io_errors_;
            log_once("rhythm cannot resume " + saved.path.string() + " (" + error_text(errno) + ")");
            set_force(sensor);
            return;
        }
        OpenSegment segment;
        segment.sensor = sensor;
        segment.path = saved.path;
        segment.fd = fd;
        segment.day = day;
        segment.index = saved.index;
        segment.last_minute = last_minute_[sensor];
        open_[sensor] = std::move(segment);
    }
    std::size_t header_size(const std::string& sensor) const {
        return encode_header(sensor_code(sensor), settings_.dims, voxels_, settings_.voxel_size, settings_.cell_m,
                             static_cast<std::uint32_t>(settings_.period_s), settings_.room_id)
            .size();
    }
    bool open_new(const std::string& sensor, std::int64_t minute, int day) {
        try {
            std::filesystem::create_directories(*directory_);
        } catch (const std::exception& error) {
            ++io_errors_;
            log_once("rhythm cannot create " + directory_->string() + " (" + error.what() + ")");
            return false;
        }
        int index = 0;
        try {
            index = latest(sensor, day).second;
        } catch (const std::exception&) {
            index = 0;
        }
        int next = index ? index + 1 : 1;
        std::filesystem::path path = *directory_ / segment_name(sensor, minute, next);
        if (std::filesystem::exists(path)) {
            next = std::max(index, 1) + 1;
            path = *directory_ / segment_name(sensor, minute, next);
        }
        const int fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_RDWR, 0644);
        if (fd < 0) {
            ++io_errors_;
            log_once("rhythm cannot open " + path.string() + " (" + error_text(errno) + ")");
            return false;
        }
        const auto header = encode_header(sensor_code(sensor), settings_.dims, voxels_, settings_.voxel_size, settings_.cell_m,
                                          static_cast<std::uint32_t>(settings_.period_s), settings_.room_id);
        bool ok = false;
        try {
            const auto written = store_.write(fd, header.data(), header.size());
            if (written != static_cast<std::ptrdiff_t>(header.size())) throw std::runtime_error("short header");
            if (store_.fsync(fd) != 0) throw std::runtime_error(error_text(errno));
            ok = true;
        } catch (const std::exception& error) {
            ++io_errors_;
            log_once("rhythm header failed for " + path.string() + " (" + error.what() + ")");
        }
        if (!ok) {
            ::close(fd);
            std::error_code ignore;
            std::filesystem::remove(path, ignore);
            return false;
        }
        OpenSegment segment;
        segment.sensor = sensor;
        segment.path = path;
        segment.fd = fd;
        segment.day = day;
        segment.index = next;
        open_[sensor] = std::move(segment);
        return true;
    }
    enum class Attempt { ok, retry, stop };
    bool write_record(OpenSegment& segment, const std::vector<std::uint8_t>& data, const RhythmRecord& record) {
        const off_t offset = ::lseek(segment.fd, 0, SEEK_END);
        if (offset < 0) {
            abandon(segment, error_text(errno));
            return false;
        }
        Attempt status = attempt(segment, data, offset);
        if (status == Attempt::ok) return true;
        if (status != Attempt::retry) return false;
        status = attempt(segment, data, offset);
        if (status == Attempt::ok) return true;
        if (status != Attempt::retry) return false;
        const auto marker = encode_marker(record.minute, record.map_epoch);
        status = attempt(segment, marker, offset);
        if (status == Attempt::ok) {
            last_minute_[segment.sensor] = record.minute;
            segment.last_minute = record.minute;
            {
                std::lock_guard lock(mu_);
                force_disc_[segment.sensor] = false;
            }
            if (const auto parsed = parse_record(marker)) publish(segment.sensor, *parsed);
            ++io_errors_;
            return false;
        }
        if (status == Attempt::retry && !segment.abandoned) {
            set_force(segment.sensor);
            ++io_errors_;
        }
        return false;
    }
    Attempt attempt(OpenSegment& segment, const std::vector<std::uint8_t>& data, off_t offset) {
        std::ptrdiff_t written = -1;
        int err = 0;
        try {
            written = store_.write(segment.fd, data.data(), data.size());
            if (written < 0) err = errno;
        } catch (const std::system_error& error) {
            err = error.code().value();
            written = -1;
        } catch (const std::exception& error) {
            log_once("rhythm write failed on " + segment.path.string() + " (" + error.what() + ")");
            if (!restore(segment, offset)) return Attempt::stop;
            ++io_errors_;
            set_force(segment.sensor);
            return Attempt::stop;
        }
        if (written < 0) {
            if (err == ENOSPC || err == EDQUOT) {
                written = 0;
            } else {
                log_once("rhythm write failed on " + segment.path.string() + " (" + error_text(err) + ")");
                if (!restore(segment, offset)) return Attempt::stop;
                ++io_errors_;
                set_force(segment.sensor);
                return Attempt::stop;
            }
        }
        if (written == static_cast<std::ptrdiff_t>(data.size())) return Attempt::ok;
        if (!restore(segment, offset)) return Attempt::stop;
        return Attempt::retry;
    }
    bool restore(OpenSegment& segment, off_t offset) {
        if (store_.truncate(segment.fd, offset) != 0 || ::lseek(segment.fd, offset, SEEK_SET) < 0) {
            abandon(segment, error_text(errno));
            return false;
        }
        return true;
    }
    void abandon(OpenSegment& segment, const std::string& error) {
        segment.abandoned = true;
        ++io_errors_;
        log_once("rhythm abandoned " + segment.path.string() + " (" + error + ")");
        set_force(segment.sensor);
        if (segment.fd >= 0) ::close(segment.fd);
        segment.fd = -1;
    }
    bool fits(std::size_t incoming, bool prune) {
        if (!directory_ || settings_.max_bytes == 0) return true;
        std::vector<BudgetFile> files;
        try {
            files = budget_files();
        } catch (const std::exception& error) {
            ++io_errors_;
            log_once("rhythm cannot list " + directory_->string() + " (" + error.what() + ")");
            return false;
        }
        std::uintmax_t total = 0;
        for (const BudgetFile& file : files) total += file.size;
        if (total + incoming <= static_cast<std::uintmax_t>(settings_.max_bytes)) return true;
        if (!prune || shutting_down_.load()) return false;
        std::vector<std::filesystem::path> open_paths;
        for (const auto& [name, segment] : open_) {
            (void)name;
            if (!segment.abandoned) open_paths.push_back(segment.path);
        }
        const auto is_open = [&](const std::filesystem::path& path) {
            return std::find(open_paths.begin(), open_paths.end(), path) != open_paths.end();
        };
        std::vector<BudgetFile> quarantine;
        std::vector<BudgetFile> closed;
        for (const BudgetFile& file : files) {
            if (is_open(file.path)) continue;
            if (file.quarantine) quarantine.push_back(file);
            else closed.push_back(file);
        }
        std::sort(quarantine.begin(), quarantine.end(),
                  [](const BudgetFile& a, const BudgetFile& b) { return a.mtime < b.mtime; });
        std::sort(closed.begin(), closed.end(), [](const BudgetFile& a, const BudgetFile& b) {
            return segment_sort_key(a.path.filename().string()) < segment_sort_key(b.path.filename().string());
        });
        for (const std::vector<BudgetFile>* group : {&quarantine, &closed}) {
            for (const BudgetFile& file : *group) {
                if (total + incoming <= static_cast<std::uintmax_t>(settings_.max_bytes)) break;
                std::error_code error;
                std::filesystem::remove(file.path, error);
                if (error) {
                    log_once("rhythm could not delete " + file.path.string() + " (" + error.message() + ")");
                    continue;
                }
                total -= file.size;
            }
            if (total + incoming <= static_cast<std::uintmax_t>(settings_.max_bytes)) return true;
        }
        return false;
    }
    std::vector<BudgetFile> budget_files() const {
        std::vector<BudgetFile> found;
        for (const auto& entry : std::filesystem::directory_iterator(*directory_)) {
            if (!entry.is_regular_file()) continue;
            const std::string name = entry.path().filename().string();
            const bool segment = parse_segment_name(name).has_value();
            const bool quarantine = parse_quarantine_name(name).has_value() || name.find(".superseded-") != std::string::npos ||
                                    name.find(".bad-") != std::string::npos;
            if (!segment && !quarantine) continue;
            std::error_code error;
            const auto status = entry.status(error);
            (void)status;
            const auto size = entry.file_size(error);
            if (error) continue;
            found.push_back(BudgetFile{entry.path(), size, quarantine && !segment, entry.last_write_time()});
        }
        return found;
    }
    void age_prune() {
        if (!directory_ || settings_.keep_days == 0) return;
        const int limit = shift_yyyymmdd(yyyymmdd_now(), static_cast<int>(settings_.keep_days));
        std::vector<std::filesystem::path> open_paths;
        for (const auto& [name, segment] : open_) {
            (void)name;
            open_paths.push_back(segment.path);
        }
        std::error_code list_error;
        std::filesystem::directory_iterator it(*directory_, list_error);
        if (list_error) {
            log_once("rhythm age prune skipped (" + list_error.message() + ")");
            return;
        }
        for (const auto& entry : it) {
            if (std::find(open_paths.begin(), open_paths.end(), entry.path()) != open_paths.end()) continue;
            std::error_code error;
            if (!entry.is_regular_file(error)) continue;
            const std::string name = entry.path().filename().string();
            const auto parsed = parse_segment_name(name) ? parse_segment_name(name) : parse_quarantine_name(name);
            if (!parsed || parsed->date >= limit) continue;
            std::filesystem::remove(entry.path(), error);
            if (error) log_once("rhythm could not delete " + entry.path().string() + " (" + error.message() + ")");
        }
    }
    void maybe_age_prune() {
        if (settings_.keep_days == 0) return;
        const int day = yyyymmdd_now();
        if (last_age_day_ && *last_age_day_ == day) return;
        last_age_day_ = day;
        age_prune();
    }
    void maybe_fsync() {
        if (settings_.fsync_s == 0) return;
        if (std::chrono::steady_clock::now() - last_fsync_ < std::chrono::seconds(settings_.fsync_s)) return;
        fsync_open();
    }
    void fsync_open() {
        for (auto& [name, segment] : open_) {
            (void)name;
            sync_fd(segment.fd);
        }
        last_fsync_ = std::chrono::steady_clock::now();
    }
    void sync_fd(int fd) {
        if (fd < 0) return;
        if (store_.fsync(fd) != 0) {
            ++io_errors_;
            log_once("rhythm fsync failed (" + error_text(errno) + ")");
        }
    }
    void close_segment(const std::string& sensor, bool abandon_segment) {
        const auto found = open_.find(sensor);
        if (found == open_.end()) return;
        if (found->second.fd >= 0) {
            ::close(found->second.fd);
            found->second.fd = -1;
        }
        if (abandon_segment) found->second.abandoned = true;
        else open_.erase(found);
    }
    void publish(const std::string& sensor, const RhythmRecord& record) {
        Rhythm::Publish publish;
        std::function<bool()> would_block;
        {
            std::lock_guard lock(pub_mu_);
            publish = publish_;
            would_block = would_block_;
        }
        if (!settings_.publish || !publish) return;
        if (would_block && would_block()) {
            ++publish_dropped_;
            return;
        }
        std::unique_lock lock(pub_mu_, std::try_to_lock);
        if (!lock.owns_lock()) {
            ++publish_dropped_;
            return;
        }
        try {
            msg::RhythmMinute message;
            message.room_id = settings_.room_id;
            message.sensor = sensor_code(sensor);
            message.map_epoch = record.map_epoch;
            message.seq = seq_++;
            message.stamp = msg::Time{record.minute, 0};
            message.windows_closed = record.closed;
            message.windows_with_points = record.with_points;
            message.windows_unavailable = record.unavailable;
            message.clipped = record.clipped;
            message.regions_dropped = record.regions_dropped;
            message.pose_ok = record.pose_ok;
            message.flags = record.flags;
            for (const RhythmRegion& region : record.regions) {
                message.ix.push_back(region.ix);
                message.iy.push_back(region.iy);
                message.iz.push_back(region.iz);
                message.hit_windows.push_back(region.hits);
                message.mean_voxels.push_back(region.mean);
            }
            message.publisher_generation = settings_.publisher_generation;
            publish_(message.to_cdr());
        } catch (const std::exception&) {
            ++publish_dropped_;
            --seq_;
        }
    }

    struct Resume {
        std::filesystem::path path;
        int index = 1;
        int day = 0;
    };

    RhythmSettings settings_;
    std::optional<std::filesystem::path> directory_;
    RhythmStore store_;
    Rhythm::Publish publish_;
    std::function<bool()> would_block_;
    int voxels_ = 1;
    std::atomic<int> budget_dropped_{0};
    std::atomic<int> queue_dropped_{0};
    std::atomic<int> io_errors_{0};
    std::atomic<int> publish_dropped_{0};
    std::atomic<bool> logged_{false};
    std::atomic<bool> stop_{false};
    std::atomic<bool> shutting_down_{false};
    std::atomic<bool> finished_{false};
    std::atomic<bool> closed_{false};
    std::uint64_t seq_ = 0;
    mutable std::mutex mu_;
    std::mutex pub_mu_;
    std::condition_variable cv_;
    std::array<std::vector<std::uint8_t>, kQueueSlots> slots_{};
    std::array<int, kQueueSlots> lengths_{};
    std::array<int, kQueueSlots> codes_{};
    int head_ = 0;
    int tail_ = 0;
    int count_ = 0;
    std::map<std::string, OpenSegment> open_;
    std::map<std::string, Resume> resume_;
    std::map<std::string, bool> force_disc_;
    std::map<std::string, std::uint64_t> disc_gen_;
    std::map<std::string, std::optional<std::int64_t>> last_minute_;
    std::optional<int> last_age_day_;
    std::chrono::steady_clock::time_point last_fsync_ = std::chrono::steady_clock::now();
    std::thread thread_;
};

}  // namespace

struct Rhythm::Impl {
    RhythmSettings settings;
    RhythmAccumulator accumulator;
    std::shared_ptr<Writer> writer;
    std::chrono::steady_clock::time_point last_hook{};
    bool closed = false;

    Impl(RhythmSettings settings, RhythmStore store, Publish publish, std::function<bool()> would_block)
        : settings(std::move(settings)),
          accumulator(this->settings.sensors, voxels_per_cell(this->settings.cell_m, this->settings.voxel_size),
                      this->settings.dims, static_cast<int>(this->settings.period_s),
                      windows_per_period(this->settings.period_s, this->settings.window_s)),
          writer(std::make_shared<Writer>(this->settings, rhythm_directory(this->settings), std::move(store),
                                          std::move(publish), std::move(would_block))) {}
};

Rhythm::Rhythm(RhythmSettings settings, RhythmStore store, Publish publish, std::function<bool()> publish_would_block) {
    validate_rhythm(settings);
    impl_ = std::make_unique<Impl>(std::move(settings), std::move(store), std::move(publish), std::move(publish_would_block));
    for (const std::string& sensor : impl_->settings.sensors) {
        if (impl_->writer->force_disc(sensor)) impl_->accumulator.set_force_disc(sensor);
    }
    impl_->writer->start();
}
Rhythm::~Rhythm() { close(); }
void Rhythm::add(const std::string& sensor, std::int64_t sec, std::span<const Key> keys, std::uint64_t map_epoch, bool tracking,
                 std::optional<double> residual, std::optional<double> learn_max_offset_m) {
    impl_->accumulator.add(sensor, sec, keys, map_epoch, tracking, residual, learn_max_offset_m);
}
void Rhythm::add_empty(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch) {
    impl_->accumulator.add_empty(sensor, sec, map_epoch);
}
void Rhythm::note_unavailable(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch) {
    impl_->accumulator.note_unavailable(sensor, sec, map_epoch);
}
void Rhythm::discard(const std::string& sensor, std::int64_t sec, std::uint64_t map_epoch) {
    impl_->accumulator.discard(sensor, sec, map_epoch);
    const auto now = std::chrono::steady_clock::now();
    if (now - impl_->last_hook >= std::chrono::seconds(60)) {
        impl_->last_hook = now;
        log_line("rhythm hook failed for " + sensor + "; dropped its open bin");
    }
}
void Rhythm::note_epoch(std::uint64_t map_epoch) { impl_->accumulator.note_epoch(map_epoch); }
void Rhythm::fail_next_insert(const std::string& sensor) { impl_->accumulator.fail_next_insert(sensor); }
void Rhythm::push_fragment(const RhythmFragment& fragment) {
    const auto loss = impl_->accumulator.loss_of(fragment.sensor);
    if (loss && !fragment.lost()) {
        if (!impl_->writer->try_enqueue(fragment.sensor, encode_marker(loss->first, loss->second))) {
            impl_->accumulator.remember_loss(fragment.sensor, fragment.minute, fragment.map_epoch);
            impl_->writer->note_discontinuity(fragment.sensor);
            return;
        }
        impl_->accumulator.clear_loss(fragment.sensor);
    }
    if (impl_->writer->try_enqueue(fragment.sensor, fragment.data)) return;
    impl_->accumulator.remember_loss(fragment.sensor, fragment.minute, fragment.map_epoch);
    impl_->writer->note_discontinuity(fragment.sensor);
}
void Rhythm::drain_loss_markers() {
    for (const std::string& sensor : impl_->settings.sensors) {
        const auto loss = impl_->accumulator.loss_of(sensor);
        if (!loss) continue;
        impl_->accumulator.clear_loss(sensor);
        impl_->writer->try_enqueue(sensor, encode_marker(loss->first, loss->second));
    }
}
void Rhythm::enqueue() {
    if (!impl_ || impl_->closed) return;
    try {
        for (const RhythmFragment& fragment : impl_->accumulator.take()) push_fragment(fragment);
    } catch (const std::exception& error) {
        log_line(std::string("rhythm enqueue failed (") + error.what() + ")");
    } catch (...) {
        log_line("rhythm enqueue failed");
    }
}
void Rhythm::close() {
    if (!impl_ || impl_->closed) return;
    impl_->closed = true;
    try {
        for (const RhythmFragment& fragment : impl_->accumulator.flush()) push_fragment(fragment);
        drain_loss_markers();
    } catch (const std::exception& error) {
        log_line(std::string("rhythm close failed (") + error.what() + ")");
    }
    impl_->writer->close();
}
void Rhythm::set_publish(Publish publish, std::function<bool()> publish_would_block) {
    impl_->writer->set_publish(std::move(publish), std::move(publish_would_block));
}
RhythmSensorView Rhythm::view(const std::string& sensor) const { return impl_->accumulator.view(sensor); }
bool Rhythm::table_holds(const std::string& sensor, int slot, std::int16_t ix, std::int16_t iy, std::int16_t iz) const {
    return impl_->accumulator.table_holds(sensor, slot, ix, iy, iz);
}
int Rhythm::budget_dropped() const { return impl_->writer->budget_dropped(); }

}  // namespace room_memory
