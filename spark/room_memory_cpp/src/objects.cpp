#include "room_memory/objects.hpp"

#include "room_memory/memory.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <utility>

namespace room_memory {
namespace {

constexpr char kMagic[4] = {'L', 'C', 'N', 'O'};
constexpr std::uint32_t kVersion = 1;
constexpr std::uint64_t kSha256K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

constexpr std::uint32_t rotr(std::uint32_t value, std::uint32_t bits) {
    return (value >> bits) | (value << (32 - bits));
}

bool valid_utf8(std::string_view text) {
    std::size_t n = 0;
    while (n < text.size()) {
        const auto c = static_cast<unsigned char>(text[n]);
        std::size_t extra = 0;
        std::uint32_t point = c;
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

bool has_control(std::string_view text) {
    std::size_t n = 0;
    while (n < text.size()) {
        const auto c = static_cast<unsigned char>(text[n]);
        std::uint32_t point = c;
        std::size_t len = 1;
        if (c >= 0x80) {
            if (c <= 0xDF && n + 1 < text.size()) {
                point = (c & 0x1Fu) << 6 | (static_cast<unsigned char>(text[n + 1]) & 0x3Fu);
                len = 2;
            } else if (c <= 0xEF && n + 2 < text.size()) {
                point = (c & 0x0Fu) << 12 | (static_cast<unsigned char>(text[n + 1]) & 0x3Fu) << 6 |
                        (static_cast<unsigned char>(text[n + 2]) & 0x3Fu);
                len = 3;
            } else if (n + 3 < text.size()) {
                point = (c & 0x07u) << 18 | (static_cast<unsigned char>(text[n + 1]) & 0x3Fu) << 12 |
                        (static_cast<unsigned char>(text[n + 2]) & 0x3Fu) << 6 |
                        (static_cast<unsigned char>(text[n + 3]) & 0x3Fu);
                len = 4;
            }
        }
        if (point < 0x20 || (point >= 0x7F && point <= 0x9F)) return true;
        n += len;
    }
    return false;
}

bool one_sensor(std::uint8_t sensor) {
    return sensor == static_cast<std::uint8_t>(msg::kSensorMid360) || sensor == static_cast<std::uint8_t>(msg::kSensorJT128);
}

bool sensor_in_set(std::uint8_t sensor, std::uint8_t set) { return one_sensor(sensor) && (set & sensor) == sensor; }

void canonicalize(Keys& keys) { sort_unique(keys); }

class Out {
public:
    void u8(std::uint8_t value) { bytes_.push_back(value); }
    void u32(std::uint32_t value) {
        for (int n = 0; n < 4; ++n) bytes_.push_back(static_cast<std::uint8_t>(value >> (8 * n)));
    }
    void u64(std::uint64_t value) {
        for (int n = 0; n < 8; ++n) bytes_.push_back(static_cast<std::uint8_t>(value >> (8 * n)));
    }
    void i64(std::int64_t value) { u64(std::bit_cast<std::uint64_t>(value)); }
    void f64(double value) { u64(std::bit_cast<std::uint64_t>(value)); }
    void str(std::string_view text) {
        u32(static_cast<std::uint32_t>(text.size()));
        bytes_.insert(bytes_.end(), text.begin(), text.end());
    }
    void keys(const Keys& keys) {
        u32(static_cast<std::uint32_t>(keys.size()));
        for (const Key key : keys) i64(key);
    }
    std::vector<std::uint8_t>& bytes() { return bytes_; }

private:
    std::vector<std::uint8_t> bytes_;
};

class In {
public:
    explicit In(std::span<const std::uint8_t> bytes) : bytes_(bytes) {}
    std::size_t left() const { return at_ < bytes_.size() ? bytes_.size() - at_ : 0; }
    void need(std::size_t n, const char* what) {
        if (n > left()) throw RegistryError(std::string("registry file truncated at ") + what);
    }
    std::uint8_t u8() {
        need(1, "u8");
        return bytes_[at_++];
    }
    std::uint32_t u32() {
        need(4, "u32");
        std::uint32_t value = 0;
        for (int n = 0; n < 4; ++n) value |= std::uint32_t{bytes_[at_++]} << (8 * n);
        return value;
    }
    std::uint64_t u64() {
        need(8, "u64");
        std::uint64_t value = 0;
        for (int n = 0; n < 8; ++n) value |= std::uint64_t{bytes_[at_++]} << (8 * n);
        return value;
    }
    std::int64_t i64() { return std::bit_cast<std::int64_t>(u64()); }
    double f64() { return std::bit_cast<double>(u64()); }
    std::string str(std::size_t cap, const char* what) {
        const std::uint32_t n = u32();
        if (n > cap) throw RegistryError(std::string(what) + " is too long");
        need(n, what);
        std::string text(reinterpret_cast<const char*>(bytes_.data() + at_), n);
        at_ += n;
        return text;
    }
    Keys keys() {
        const std::uint32_t n = u32();
        if (static_cast<std::uint64_t>(n) > left() / 8) throw RegistryError("registry file truncated at keys");
        Keys out;
        out.reserve(n);
        for (std::uint32_t i = 0; i < n; ++i) out.push_back(i64());
        canonicalize(out);
        return out;
    }
    bool done() const { return at_ == bytes_.size(); }

private:
    std::span<const std::uint8_t> bytes_;
    std::size_t at_ = 0;
};

std::uint32_t crc_of(std::span<const std::uint8_t> data) {
    return static_cast<std::uint32_t>(::crc32(0L, data.data(), static_cast<uInt>(data.size())));
}

msg::EventKind event_kind(std::uint8_t value) {
    if (value > static_cast<std::uint8_t>(msg::EventKind::moved)) throw RegistryError("bad event kind");
    return static_cast<msg::EventKind>(value);
}

msg::Baseline baseline_of(std::uint8_t value) {
    if (value > static_cast<std::uint8_t>(msg::Baseline::retired)) throw RegistryError("bad baseline");
    return static_cast<msg::Baseline>(value);
}

msg::NameOutcome outcome_of(std::uint8_t value) {
    if (value > static_cast<std::uint8_t>(msg::NameOutcome::unknown_object)) throw RegistryError("bad outcome");
    return static_cast<msg::NameOutcome>(value);
}

void put_change(Out& out, const PublishedChange& change) {
    out.u8(change.sensor);
    out.u64(change.frame_epoch);
    out.u8(static_cast<std::uint8_t>(change.kind));
    out.keys(change.keys);
    out.keys(change.from_keys);
}

PublishedChange get_change(In& in) {
    PublishedChange change;
    change.sensor = in.u8();
    change.frame_epoch = in.u64();
    change.kind = event_kind(in.u8());
    change.keys = in.keys();
    change.from_keys = in.keys();
    return change;
}

double mean_lane(const Keys& keys, int lane) {
    double sum = 0;
    for (const Key key : keys) {
        const Index3 v = unpack(key);
        const std::int64_t lanes[3] = {v.i, v.j, v.k};
        sum += static_cast<double>(lanes[lane]);
    }
    return sum / static_cast<double>(keys.size());
}

}  // namespace

bool install_registry_file(const std::filesystem::path& path, std::span<const std::uint8_t> bytes) {
    const std::filesystem::path parent = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
    std::filesystem::create_directories(parent);
    const std::filesystem::path scratch = path.string() + ".tmp";
    const int fd = ::open(scratch.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) throw StorageError("cannot write " + scratch.string());
    std::size_t written = 0;
    while (written < bytes.size()) {
        const ssize_t n = ::write(fd, bytes.data() + written, bytes.size() - written);
        if (n < 0) {
            ::close(fd);
            ::unlink(scratch.c_str());
            throw StorageError("cannot write " + scratch.string());
        }
        written += static_cast<std::size_t>(n);
    }
    const bool synced = ::fsync(fd) == 0;
    ::close(fd);
    if (!synced) {
        ::unlink(scratch.c_str());
        throw StorageError("cannot sync " + scratch.string());
    }
    std::filesystem::rename(scratch, path);
    const int directory = ::open(parent.c_str(), O_RDONLY | O_CLOEXEC);
    if (directory < 0) return false;
    const bool dir_synced = ::fsync(directory) == 0;
    ::close(directory);
    return dir_synced;
}

std::array<std::uint8_t, 32> sha256(std::span<const std::uint8_t> data) {
    std::uint32_t h0 = 0x6a09e667, h1 = 0xbb67ae85, h2 = 0x3c6ef372, h3 = 0xa54ff53a;
    std::uint32_t h4 = 0x510e527f, h5 = 0x9b05688c, h6 = 0x1f83d9ab, h7 = 0x5be0cd19;
    std::vector<std::uint8_t> padded(data.begin(), data.end());
    const std::uint64_t bits = static_cast<std::uint64_t>(data.size()) * 8;
    padded.push_back(0x80);
    while (padded.size() % 64 != 56) padded.push_back(0);
    for (int shift = 56; shift >= 0; shift -= 8) padded.push_back(static_cast<std::uint8_t>(bits >> shift));
    for (std::size_t offset = 0; offset < padded.size(); offset += 64) {
        std::uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (std::uint32_t{padded[offset + static_cast<std::size_t>(i) * 4]} << 24) |
                   (std::uint32_t{padded[offset + static_cast<std::size_t>(i) * 4 + 1]} << 16) |
                   (std::uint32_t{padded[offset + static_cast<std::size_t>(i) * 4 + 2]} << 8) |
                   std::uint32_t{padded[offset + static_cast<std::size_t>(i) * 4 + 3]};
        }
        for (int i = 16; i < 64; ++i) {
            const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        std::uint32_t a = h0, b = h1, c = h2, d = h3, e = h4, f = h5, g = h6, h = h7;
        for (int i = 0; i < 64; ++i) {
            const std::uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const std::uint32_t ch = (e & f) ^ ((~e) & g);
            const std::uint32_t t1 = h + S1 + ch + static_cast<std::uint32_t>(kSha256K[i]) + w[i];
            const std::uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t t2 = S0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
        h5 += f;
        h6 += g;
        h7 += h;
    }
    const std::uint32_t words[8] = {h0, h1, h2, h3, h4, h5, h6, h7};
    std::array<std::uint8_t, 32> out{};
    for (int i = 0; i < 8; ++i) {
        out[static_cast<std::size_t>(i) * 4] = static_cast<std::uint8_t>(words[i] >> 24);
        out[static_cast<std::size_t>(i) * 4 + 1] = static_cast<std::uint8_t>(words[i] >> 16);
        out[static_cast<std::size_t>(i) * 4 + 2] = static_cast<std::uint8_t>(words[i] >> 8);
        out[static_cast<std::size_t>(i) * 4 + 3] = static_cast<std::uint8_t>(words[i]);
    }
    return out;
}

std::array<std::uint8_t, 32> request_digest(const msg::NameObjectRequest& request) {
    Out out;
    out.str(request.header.room_id);
    out.u64(request.object_id);
    out.u64(request.expected_revision);
    out.u64(request.event_generation);
    out.u64(request.event_id);
    out.str(request.name);
    out.str(request.operator_id);
    return sha256(out.bytes());
}

bool accept_matches(const Keys& region, const Keys& published, std::int64_t tolerance, double fraction, bool seen_now) {
    if (!seen_now) return false;
    Keys left = region;
    Keys right = published;
    canonicalize(left);
    canonicalize(right);
    return same_shape(left, right, tolerance, fraction);
}

bool Registry::text_ok(const std::string& text, std::size_t low, std::size_t high) {
    return valid_utf8(text) && text.find('\0') == std::string::npos && text.size() >= low && text.size() <= high &&
           !has_control(text);
}

bool Registry::within_ratio(std::size_t side, std::size_t anchor, double ratio) {
    if (side == 0 || anchor == 0) return false;
    const double hi = static_cast<double>(std::max(side, anchor));
    const double lo = static_cast<double>(std::min(side, anchor));
    return hi / lo <= ratio;
}

bool Registry::translated_match(const Keys& destination, const Keys& from, const Keys& anchor) const {
    if (destination.empty() || from.empty() || anchor.empty()) return false;
    std::int64_t offset[3];
    for (int lane = 0; lane < 3; ++lane) {
        offset[lane] = std::llround(mean_lane(destination, lane) - mean_lane(from, lane));
    }
    Keys shifted;
    shifted.reserve(anchor.size());
    for (const Key key : anchor) {
        const Index3 v = unpack(key);
        const std::int64_t i = v.i + offset[0];
        const std::int64_t j = v.j + offset[1];
        const std::int64_t k = v.k + offset[2];
        if (!in_range(i, j, k)) return false;
        shifted.push_back(pack(i, j, k));
    }
    canonicalize(shifted);
    Keys dest = destination;
    canonicalize(dest);
    return same_shape(dest, shifted, config_.tolerance, config_.name_min_fraction);
}

msg::NameObjectResponse Registry::respond(msg::NameOutcome outcome, std::uint64_t object_id, std::uint64_t revision,
                                          bool repeat_differs) const {
    msg::NameObjectResponse response;
    response.outcome = outcome;
    response.object_id = object_id;
    response.revision = revision;
    response.repeat_differs = repeat_differs;
    return response;
}

bool Registry::commit_draft(Registry draft, bool required) {
    const std::vector<std::uint8_t> bytes = draft.encode();
    bool durable = true;
    try {
        durable = write_ ? write_(path_, bytes) : install_registry_file(path_, bytes);
    } catch (const StorageError&) {
        if (required) persist_blocked_ = true;
        throw;
    }
    draft.storage_error_ = !durable;
    draft.persist_blocked_ = false;
    *this = std::move(draft);
    return durable;
}

std::vector<std::uint8_t> Registry::encode() const {
    Out out;
    out.u8(static_cast<std::uint8_t>(kMagic[0]));
    out.u8(static_cast<std::uint8_t>(kMagic[1]));
    out.u8(static_cast<std::uint8_t>(kMagic[2]));
    out.u8(static_cast<std::uint8_t>(kMagic[3]));
    out.u32(kVersion);
    out.str(room_id_);
    out.f64(voxel_size_);
    out.u8(sensors_);
    out.u64(frame_epoch_);
    out.u64(next_id_);
    out.u32(static_cast<std::uint32_t>(objects_.size()));
    for (const Object& obj : objects_) {
        out.u64(obj.id);
        out.u64(obj.revision);
        out.str(obj.name);
        out.u8(obj.sensor);
        out.u8(static_cast<std::uint8_t>(obj.baseline));
        out.u8(obj.anchor ? 1 : 0);
        if (obj.anchor) {
            out.u64(obj.anchor->frame_epoch);
            out.u64(obj.anchor->set_by_generation);
            out.u64(obj.anchor->set_by_id);
            out.keys(obj.anchor->keys);
        }
        out.u32(static_cast<std::uint32_t>(obj.links.size()));
        for (const ObjectLinkRecord& link : obj.links) {
            out.u64(link.event_generation);
            out.u64(link.event_id);
            out.u8(static_cast<std::uint8_t>(link.kind));
            out.u8(link.recovered ? 1 : 0);
        }
    }
    out.u32(static_cast<std::uint32_t>(seeds_.size()));
    for (const Seed& seed : seeds_) {
        out.u64(seed.event_generation);
        out.u64(seed.event_id);
        out.u64(seed.frame_epoch);
        out.u8(seed.sensor);
        out.keys(seed.keys);
    }
    out.u32(static_cast<std::uint32_t>(dedupe_.size()));
    for (const DedupeEntry& entry : dedupe_) {
        out.str(entry.action_id);
        for (const std::uint8_t byte : entry.digest) out.u8(byte);
        out.u8(static_cast<std::uint8_t>(entry.outcome));
        out.u64(entry.object_id);
        out.u64(entry.revision);
        out.u64(entry.stamp_ns);
    }
    out.u8(record_ ? 1 : 0);
    if (record_) {
        out.u64(record_->frame_epoch);
        out.u8(record_->redo ? 1 : 0);
        if (record_->redo) put_change(out, *record_->redo);
        out.u8(static_cast<std::uint8_t>(record_->effect.kind));
        out.u64(record_->effect.object_id);
        if (record_->effect.kind == SpatialEffect::Kind::move) out.keys(record_->effect.keys);
        out.u32(static_cast<std::uint32_t>(record_->effect.detach_ids.size()));
        for (const std::uint64_t id : record_->effect.detach_ids) out.u64(id);
        out.u8(record_->finalize ? 1 : 0);
        if (record_->finalize) {
            out.u64(record_->finalize->event_generation);
            out.u64(record_->finalize->event_id);
            out.u8(static_cast<std::uint8_t>(record_->finalize->kind));
            out.u64(record_->finalize->object_id);
        }
    }
    out.u32(static_cast<std::uint32_t>(receipts_.size()));
    for (const Receipt& receipt : receipts_) {
        out.u64(receipt.event_generation);
        out.u64(receipt.event_id);
        out.u8(static_cast<std::uint8_t>(receipt.kind));
    }
    const std::uint32_t crc = crc_of(out.bytes());
    out.u32(crc);
    return out.bytes();
}

Registry Registry::decode(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < 8) throw RegistryError("registry file is too short");
    const std::uint32_t stored = crc_of(bytes.first(bytes.size() - 4));
    In tail(bytes.last(4));
    if (tail.u32() != stored) throw RegistryError("registry file crc mismatch");
    In in(bytes.first(bytes.size() - 4));
    if (in.u8() != kMagic[0] || in.u8() != kMagic[1] || in.u8() != kMagic[2] || in.u8() != kMagic[3]) {
        throw RegistryError("registry file magic mismatch");
    }
    if (in.u32() != kVersion) throw RegistryError("registry file version mismatch");
    Registry reg;
    reg.room_id_ = in.str(1024, "room_id");
    reg.voxel_size_ = in.f64();
    reg.sensors_ = in.u8();
    reg.frame_epoch_ = in.u64();
    reg.next_id_ = in.u64();
    const std::uint32_t n_objects = in.u32();
    if (static_cast<std::uint64_t>(n_objects) > in.left()) throw RegistryError("registry file truncated at objects");
    reg.objects_.reserve(n_objects);
    for (std::uint32_t n = 0; n < n_objects; ++n) {
        Object obj;
        obj.id = in.u64();
        obj.revision = in.u64();
        obj.name = in.str(msg::kMaxNameBytes, "name");
        obj.sensor = in.u8();
        obj.baseline = baseline_of(in.u8());
        if (in.u8()) {
            Anchor anchor;
            anchor.frame_epoch = in.u64();
            anchor.set_by_generation = in.u64();
            anchor.set_by_id = in.u64();
            anchor.keys = in.keys();
            obj.anchor = std::move(anchor);
        }
        const std::uint32_t n_links = in.u32();
        if (n_links > kRegistryLinkCap) throw RegistryError("too many links");
        for (std::uint32_t link = 0; link < n_links; ++link) {
            ObjectLinkRecord row;
            row.event_generation = in.u64();
            row.event_id = in.u64();
            row.kind = event_kind(in.u8());
            const std::uint8_t recovered = in.u8();
            if (recovered > 1) throw RegistryError("bad recovered flag");
            row.recovered = recovered == 1;
            obj.links.push_back(row);
        }
        if (obj.baseline == msg::Baseline::gone) {
            if (obj.anchor) throw RegistryError("GONE object has an anchor");
        } else if (!obj.anchor || obj.anchor->keys.empty()) {
            throw RegistryError("object is missing its anchor");
        }
        reg.objects_.push_back(std::move(obj));
    }
    const std::uint32_t n_seeds = in.u32();
    if (n_seeds > kRegistrySeedCap) throw RegistryError("too many seeds");
    for (std::uint32_t n = 0; n < n_seeds; ++n) {
        Seed seed;
        seed.event_generation = in.u64();
        seed.event_id = in.u64();
        seed.frame_epoch = in.u64();
        seed.sensor = in.u8();
        seed.keys = in.keys();
        reg.seeds_.push_back(std::move(seed));
    }
    const std::uint32_t n_dedupe = in.u32();
    if (n_dedupe > kRegistryDedupeCap) throw RegistryError("dedupe ring is over the cap");
    for (std::uint32_t n = 0; n < n_dedupe; ++n) {
        DedupeEntry entry;
        entry.action_id = in.str(msg::kMaxNameBytes, "action_id");
        for (std::uint8_t& byte : entry.digest) byte = in.u8();
        entry.outcome = outcome_of(in.u8());
        entry.object_id = in.u64();
        entry.revision = in.u64();
        entry.stamp_ns = in.u64();
        reg.dedupe_.push_back(std::move(entry));
    }
    if (in.u8()) {
        CommitRecord record;
        record.frame_epoch = in.u64();
        if (in.u8()) record.redo = get_change(in);
        const std::uint8_t effect = in.u8();
        if (effect > static_cast<std::uint8_t>(SpatialEffect::Kind::gone)) throw RegistryError("bad effect");
        record.effect.kind = static_cast<SpatialEffect::Kind>(effect);
        record.effect.object_id = in.u64();
        if (record.effect.kind == SpatialEffect::Kind::move) record.effect.keys = in.keys();
        const std::uint32_t n_detach = in.u32();
        for (std::uint32_t n = 0; n < n_detach; ++n) record.effect.detach_ids.push_back(in.u64());
        if (in.u8()) {
            Finalize finalize;
            finalize.event_generation = in.u64();
            finalize.event_id = in.u64();
            finalize.kind = event_kind(in.u8());
            finalize.object_id = in.u64();
            record.finalize = finalize;
        }
        reg.record_ = std::move(record);
    }
    const std::uint32_t n_receipts = in.u32();
    if (n_receipts > kRegistryReceiptCap) throw RegistryError("too many receipts");
    for (std::uint32_t n = 0; n < n_receipts; ++n) {
        Receipt receipt;
        receipt.event_generation = in.u64();
        receipt.event_id = in.u64();
        receipt.kind = event_kind(in.u8());
        reg.receipts_.push_back(receipt);
    }
    if (!in.done()) throw RegistryError("registry file has trailing bytes");
    return reg;
}

Registry Registry::open(const std::filesystem::path& state_dir, const std::string& room_id, double voxel_size,
                        std::uint8_t sensors, std::uint64_t frame_epoch, WriteStaged write, Absorb absorb, Occupied occupied,
                        RegistryConfig config) {
    if (room_id.empty() || room_id.find('/') != std::string::npos || room_id.find('\0') != std::string::npos) {
        throw RegistryError("room_id is empty or contains a slash");
    }
    if (!(std::isfinite(voxel_size) && voxel_size > 0)) throw RegistryError("voxel_size must be finite and > 0");
    const auto allowed = static_cast<std::uint8_t>(msg::kSensorsAll);
    if (sensors == 0 || (sensors & static_cast<std::uint8_t>(~allowed)) != 0) throw RegistryError("sensor set is empty or has a foreign bit");
    const std::filesystem::path path = state_dir / room_id / "objects";
    auto finish = [&](Registry reg) {
        reg.path_ = path;
        reg.write_ = std::move(write);
        reg.occupied_ = std::move(occupied);
        reg.config_ = config;
        return reg;
    };
    if (!std::filesystem::exists(path)) {
        Registry reg;
        reg.room_id_ = room_id;
        reg.voxel_size_ = voxel_size;
        reg.sensors_ = sensors;
        reg.frame_epoch_ = frame_epoch;
        reg.next_id_ = 1;
        reg = finish(std::move(reg));
        reg.commit_draft(reg, true);
        reg.snapshot_seq_ = 0;
        return reg;
    }
    std::vector<std::uint8_t> bytes;
    {
        const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) throw RegistryError("cannot read " + path.string());
        const off_t size = ::lseek(fd, 0, SEEK_END);
        if (size < 0) {
            ::close(fd);
            throw RegistryError("cannot read " + path.string());
        }
        ::lseek(fd, 0, SEEK_SET);
        bytes.resize(static_cast<std::size_t>(size));
        std::size_t got = 0;
        while (got < bytes.size()) {
            const ssize_t n = ::read(fd, bytes.data() + got, bytes.size() - got);
            if (n <= 0) {
                ::close(fd);
                throw RegistryError("cannot read " + path.string());
            }
            got += static_cast<std::size_t>(n);
        }
        ::close(fd);
    }
    Registry reg = finish(decode(bytes));
    if (reg.room_id_ != room_id || std::bit_cast<std::uint64_t>(reg.voxel_size_) != std::bit_cast<std::uint64_t>(voxel_size) ||
        reg.sensors_ != sensors) {
        throw RegistryError("registry header does not match this room");
    }
    const bool record_disagrees = reg.record_ && reg.record_->frame_epoch != reg.frame_epoch_;
    if (reg.frame_epoch_ != frame_epoch || record_disagrees) {
        reg.commit_frame_change(frame_epoch);
    } else if (reg.record_) {
        reg.replay(absorb);
    }
    reg.snapshot_seq_ = 0;
    return reg;
}

bool Registry::key_occupied(Key key) const { return !occupied_ || occupied_(key); }

bool Registry::enough(const Keys& keys) const {
    if (keys.empty()) return false;
    std::size_t n = 0;
    for (const Key key : keys) {
        if (key_occupied(key)) ++n;
    }
    return static_cast<double>(n) >= config_.name_min_fraction * static_cast<double>(keys.size());
}

bool Registry::overlaps_anchored(const Keys& keys) const {
    for (const Object& obj : objects_) {
        if (obj.baseline == msg::Baseline::anchored && obj.anchor && intersects(obj.anchor->keys, keys)) return true;
    }
    return false;
}

Object* Registry::find_object(std::uint64_t id) {
    for (Object& obj : objects_) {
        if (obj.id == id) return &obj;
    }
    return nullptr;
}

const Object* Registry::find_object(std::uint64_t id) const {
    for (const Object& obj : objects_) {
        if (obj.id == id) return &obj;
    }
    return nullptr;
}

Seed* Registry::find_seed(std::uint64_t generation, std::uint64_t event_id) {
    for (Seed& seed : seeds_) {
        if (seed.event_generation == generation && seed.event_id == event_id) return &seed;
    }
    return nullptr;
}

bool Registry::expired(const DedupeEntry& entry, std::uint64_t now_ns) const {
    return now_ns >= entry.stamp_ns && now_ns - entry.stamp_ns > kRegistryDedupeTtlNs;
}

void Registry::evict_dedupe(std::uint64_t now_ns) {
    std::erase_if(dedupe_, [&](const DedupeEntry& entry) { return expired(entry, now_ns); });
}

const DedupeEntry* Registry::find_dedupe(const std::string& action_id) const {
    for (const DedupeEntry& entry : dedupe_) {
        if (entry.action_id == action_id) return &entry;
    }
    return nullptr;
}

std::size_t Registry::live_dedupe(std::uint64_t now_ns) const {
    std::size_t n = 0;
    for (const DedupeEntry& entry : dedupe_) {
        if (!expired(entry, now_ns)) ++n;
    }
    return n;
}

bool Registry::add_receipt(const Finalize& finalize) {
    for (const Receipt& receipt : receipts_) {
        if (receipt.event_generation == finalize.event_generation && receipt.event_id == finalize.event_id) return false;
    }
    if (receipts_.size() == kRegistryReceiptCap) receipts_.erase(receipts_.begin());
    receipts_.push_back(Receipt{finalize.event_generation, finalize.event_id, finalize.kind});
    return true;
}

void Registry::mark_links_recovered(const Finalize& finalize, bool bump) {
    for (Object& obj : objects_) {
        bool flipped = false;
        for (ObjectLinkRecord& link : obj.links) {
            if (link.event_generation == finalize.event_generation && link.event_id == finalize.event_id && !link.recovered) {
                link.recovered = true;
                flipped = true;
            }
        }
        if (flipped && bump) ++obj.revision;
    }
}

bool Registry::apply_frame_change(std::uint64_t new_frame) {
    if (frame_epoch_ == new_frame && !record_) return false;
    const std::optional<Finalize> finalize = record_ ? record_->finalize : std::nullopt;
    record_.reset();
    for (Object& obj : objects_) {
        const bool retire = obj.anchor && obj.anchor->frame_epoch != new_frame &&
                            (obj.baseline == msg::Baseline::anchored || obj.baseline == msg::Baseline::detached);
        if (retire) obj.baseline = msg::Baseline::retired;
        bool recovered = false;
        if (finalize) {
            for (ObjectLinkRecord& link : obj.links) {
                if (link.event_generation == finalize->event_generation && link.event_id == finalize->event_id && !link.recovered) {
                    link.recovered = true;
                    recovered = true;
                }
            }
        }
        if (retire || recovered) ++obj.revision;
        obj.association = msg::Association::none;
        obj.association_generation = 0;
        obj.association_event_id = 0;
    }
    if (finalize) add_receipt(*finalize);
    std::erase_if(seeds_, [&](const Seed& seed) { return seed.frame_epoch != new_frame; });
    frame_epoch_ = new_frame;
    return true;
}

void Registry::commit_frame_change(std::uint64_t frame_epoch) {
    Registry draft = *this;
    if (!draft.apply_frame_change(frame_epoch)) return;
    ++draft.snapshot_seq_;
    commit_draft(std::move(draft), true);
}

void Registry::apply_stored_effect() {
    if (!record_) return;
    const SpatialEffect& effect = record_->effect;
    if (effect.kind == SpatialEffect::Kind::move || effect.kind == SpatialEffect::Kind::gone) {
        if (Object* obj = find_object(effect.object_id)) {
            if (effect.kind == SpatialEffect::Kind::move) {
                Anchor anchor = obj->anchor.value_or(Anchor{});
                anchor.keys = effect.keys;
                anchor.frame_epoch = frame_epoch_;
                if (record_->finalize) {
                    anchor.set_by_generation = record_->finalize->event_generation;
                    anchor.set_by_id = record_->finalize->event_id;
                }
                obj->anchor = std::move(anchor);
                obj->baseline = msg::Baseline::anchored;
            } else {
                obj->anchor.reset();
                obj->baseline = msg::Baseline::gone;
            }
            ++obj->revision;
            obj->association = msg::Association::none;
            obj->association_generation = 0;
            obj->association_event_id = 0;
        }
    }
    for (const std::uint64_t id : effect.detach_ids) {
        if (Object* obj = find_object(id); obj && obj->baseline == msg::Baseline::anchored) {
            obj->baseline = msg::Baseline::detached;
            ++obj->revision;
            obj->association = msg::Association::none;
            obj->association_generation = 0;
            obj->association_event_id = 0;
        }
    }
}

void Registry::recompute_validity() {
    for (Object& obj : objects_) {
        if (obj.baseline != msg::Baseline::anchored || !obj.anchor) continue;
        if (!enough(obj.anchor->keys)) {
            obj.baseline = msg::Baseline::detached;
            ++obj.revision;
            obj.association = msg::Association::none;
            obj.association_generation = 0;
            obj.association_event_id = 0;
        }
    }
}

void Registry::replay(const Absorb& absorb) {
    if (!record_) return;
    if (record_->redo && absorb) absorb(*record_->redo);
    Registry draft = *this;
    draft.apply_stored_effect();
    draft.recompute_validity();
    if (draft.record_->finalize) {
        draft.add_receipt(*draft.record_->finalize);
        draft.mark_links_recovered(*draft.record_->finalize, true);
    }
    draft.record_.reset();
    commit_draft(std::move(draft), true);
}

bool Registry::add_seed(std::uint64_t generation, std::uint64_t event_id, std::uint8_t sensor, Keys keys) {
    if (persist_blocked_) throw RegistryError("a required write is not installed");
    if (generation == 0 || event_id == 0) throw RegistryError("a seed event id is 0");
    if (!sensor_in_set(sensor, sensors_)) throw RegistryError("seed sensor is outside the registry set");
    canonicalize(keys);
    if (keys.empty()) throw RegistryError("a seed has no voxels");
    if (find_seed(generation, event_id)) return true;
    Registry draft = *this;
    if (draft.seeds_.size() == kRegistrySeedCap) draft.seeds_.erase(draft.seeds_.begin());
    draft.seeds_.push_back(Seed{generation, event_id, frame_epoch_, sensor, std::move(keys)});
    ++draft.snapshot_seq_;
    return commit_draft(std::move(draft), false);
}

bool Registry::stage_pending_accept(const PublishedChange& published, std::optional<Finalize> finalize, bool store_redo) {
    if (persist_blocked_) throw RegistryError("a required write is not installed");
    if (record_) throw RegistryError("a commit record is already outstanding");
    if (!sensor_in_set(published.sensor, sensors_)) throw RegistryError("published sensor is outside the registry set");
    if (finalize && (finalize->event_generation == 0 || finalize->event_id == 0)) throw RegistryError("finalize event id is 0");
    Registry draft = *this;
    CommitRecord record;
    record.frame_epoch = frame_epoch_;
    PublishedChange change = published;
    canonicalize(change.keys);
    canonicalize(change.from_keys);
    if (store_redo) record.redo = change;
    bool snapshot = false;
    if (finalize && finalize->object_id != 0) {
        Object* obj = draft.find_object(finalize->object_id);
        if (!obj) throw RegistryError("finalize names an unknown object");
        const bool linked = obj->association == msg::Association::pending &&
                            obj->association_generation == finalize->event_generation &&
                            obj->association_event_id == finalize->event_id;
        if (linked && change.kind == msg::EventKind::moved) {
            record.effect.kind = SpatialEffect::Kind::move;
            record.effect.object_id = obj->id;
            record.effect.keys = change.keys;
        } else if (linked && change.kind == msg::EventKind::disappeared) {
            record.effect.kind = SpatialEffect::Kind::gone;
            record.effect.object_id = obj->id;
        }
        const bool have = std::any_of(obj->links.begin(), obj->links.end(), [&](const ObjectLinkRecord& link) {
            return link.event_generation == finalize->event_generation && link.event_id == finalize->event_id;
        });
        if (!have) {
            if (obj->links.size() == kRegistryLinkCap) throw RegistryError("an object already has 32 links");
            obj->links.push_back(ObjectLinkRecord{finalize->event_generation, finalize->event_id, finalize->kind, false});
            ++obj->revision;
            snapshot = true;
        }
    }
    for (const Object& obj : draft.objects_) {
        if (obj.sensor != change.sensor || obj.baseline != msg::Baseline::anchored || !obj.anchor) continue;
        if (record.effect.object_id == obj.id && record.effect.kind != SpatialEffect::Kind::none) continue;
        if (!draft.enough(obj.anchor->keys)) record.effect.detach_ids.push_back(obj.id);
    }
    std::sort(record.effect.detach_ids.begin(), record.effect.detach_ids.end());
    const Keys changed = set_union(change.keys, change.from_keys);
    const std::size_t before = draft.seeds_.size();
    std::erase_if(draft.seeds_, [&](const Seed& seed) { return intersects(seed.keys, changed); });
    if (draft.seeds_.size() != before) snapshot = true;
    record.finalize = std::move(finalize);
    draft.record_ = std::move(record);
    if (snapshot) ++draft.snapshot_seq_;
    return commit_draft(std::move(draft), false);
}

void Registry::settle() {
    if (persist_blocked_) throw RegistryError("a required write is not installed");
    if (record_) throw RegistryError("a commit record is already outstanding");
    Registry draft = *this;
    bool changed = false;
    for (Object& obj : draft.objects_) {
        if (obj.baseline != msg::Baseline::anchored || !obj.anchor) continue;
        if (draft.enough(obj.anchor->keys)) continue;
        obj.baseline = msg::Baseline::detached;
        ++obj.revision;
        obj.association = msg::Association::none;
        obj.association_generation = 0;
        obj.association_event_id = 0;
        changed = true;
    }
    if (!changed) return;
    ++draft.snapshot_seq_;
    commit_draft(std::move(draft), false);
}

void Registry::link_window(std::uint8_t sensor, const std::vector<LinkEvent>& events) {
    struct Eligible {
        std::uint64_t generation;
        std::uint64_t id;
        msg::EventKind kind;
        Keys removed;
        Keys destination;
    };
    std::vector<Eligible> eligible;
    for (LinkEvent event : events) {
        if (event.missed != 0) continue;
        if (event.kind != msg::EventKind::moved && event.kind != msg::EventKind::disappeared) continue;
        if (event.event_generation == 0 || event.event_id == 0) continue;
        canonicalize(event.keys);
        canonicalize(event.from_keys);
        Eligible row;
        row.generation = event.event_generation;
        row.id = event.event_id;
        row.kind = event.kind;
        row.removed = event.kind == msg::EventKind::disappeared ? event.keys : event.from_keys;
        row.destination = event.keys;
        if (!row.removed.empty()) eligible.push_back(std::move(row));
    }
    std::vector<Object*> anchors;
    for (Object& obj : objects_) {
        if (obj.sensor == sensor && obj.baseline == msg::Baseline::anchored && obj.anchor && !obj.anchor->keys.empty()) {
            anchors.push_back(&obj);
        }
    }
    bool changed = false;
    for (Object* obj : anchors) {
        std::vector<const Eligible*> hits;
        for (const Eligible& event : eligible) {
            if (intersects(event.removed, obj->anchor->keys)) hits.push_back(&event);
        }
        msg::Association association = msg::Association::none;
        std::uint64_t generation = 0;
        std::uint64_t id = 0;
        if (!hits.empty()) {
            const Eligible& event = *hits.front();
            generation = event.generation;
            id = event.id;
            bool exclusive = hits.size() == 1;
            if (exclusive) {
                for (const Object* other : anchors) {
                    if (other != obj && intersects(event.removed, other->anchor->keys)) exclusive = false;
                }
            }
            const bool shape = exclusive && same_shape(event.removed, obj->anchor->keys, config_.tolerance, config_.name_min_fraction);
            bool destination = true;
            if (event.kind == msg::EventKind::moved) {
                destination = within_ratio(event.destination.size(), obj->anchor->keys.size(), config_.name_count_ratio) &&
                              within_ratio(event.removed.size(), obj->anchor->keys.size(), config_.name_count_ratio) &&
                              translated_match(event.destination, event.removed, obj->anchor->keys);
            }
            association = shape && destination ? msg::Association::pending : msg::Association::unresolved;
        }
        if (obj->association != association || obj->association_generation != generation || obj->association_event_id != id) {
            obj->association = association;
            obj->association_generation = generation;
            obj->association_event_id = id;
            changed = true;
        }
    }
    if (changed) ++snapshot_seq_;
}

msg::NameObjectResponse Registry::store_and_commit(Registry draft, const msg::NameObjectRequest& request,
                                                   msg::NameObjectResponse response, std::uint64_t now_ns, bool bump_snapshot) {
    DedupeEntry entry;
    entry.action_id = request.action_id;
    entry.digest = request_digest(request);
    entry.outcome = response.outcome;
    entry.object_id = response.object_id;
    entry.revision = response.revision;
    entry.stamp_ns = now_ns;
    draft.dedupe_.push_back(std::move(entry));
    if (bump_snapshot) ++draft.snapshot_seq_;
    try {
        commit_draft(std::move(draft), false);
    } catch (const StorageError&) {
        return respond(msg::NameOutcome::storage_failed, 0, 0);
    }
    return response;
}

msg::NameObjectResponse Registry::name(const msg::NameObjectRequest& request, std::uint64_t now_ns) {
    if (const DedupeEntry* hit = find_dedupe(request.action_id); hit && !expired(*hit, now_ns)) {
        msg::NameObjectResponse response = respond(hit->outcome, hit->object_id, hit->revision);
        response.repeat_differs = hit->digest != request_digest(request);
        return response;
    }
    if (live_dedupe(now_ns) >= kRegistryDedupeCap) return respond(msg::NameOutcome::storage_failed, 0, 0);
    if (persist_blocked_) return respond(msg::NameOutcome::storage_failed, 0, 0);

    const bool action_ok = text_ok(request.action_id, 1, msg::kMaxNameBytes);
    const bool fields_ok = action_ok && text_ok(request.name, 1, msg::kMaxNameBytes) &&
                           text_ok(request.operator_id, 1, msg::kMaxNameBytes);
    const bool create = request.object_id == 0 && request.expected_revision == 0 && request.event_generation != 0 &&
                        request.event_id != 0;
    const bool rename = request.object_id != 0 && request.event_generation == 0 && request.event_id == 0;
    const bool attach = request.object_id != 0 && request.event_generation != 0 && request.event_id != 0;
    if (!fields_ok || (!create && !rename && !attach)) {
        if (!action_ok) return respond(msg::NameOutcome::invalid, 0, 0);
        Registry draft = *this;
        draft.evict_dedupe(now_ns);
        return store_and_commit(std::move(draft), request, respond(msg::NameOutcome::invalid, 0, 0), now_ns, false);
    }
    if (request.header.room_id != room_id_) {
        Registry draft = *this;
        draft.evict_dedupe(now_ns);
        return store_and_commit(std::move(draft), request, respond(msg::NameOutcome::wrong_room, 0, 0), now_ns, false);
    }

    Registry draft = *this;
    draft.evict_dedupe(now_ns);
    auto fail = [&](msg::NameOutcome outcome, std::uint64_t object_id, std::uint64_t revision) {
        return store_and_commit(draft, request, respond(outcome, object_id, revision), now_ns, false);
    };

    if (create || attach) {
        Object* obj = nullptr;
        if (attach) {
            obj = draft.find_object(request.object_id);
            if (!obj) return fail(msg::NameOutcome::unknown_object, request.object_id, 0);
            if (obj->revision != request.expected_revision) return fail(msg::NameOutcome::stale_revision, obj->id, obj->revision);
            if (obj->baseline == msg::Baseline::anchored) return fail(msg::NameOutcome::state_mismatch, obj->id, obj->revision);
        }
        Seed* seed = draft.find_seed(request.event_generation, request.event_id);
        if (!seed || seed->frame_epoch != draft.frame_epoch_) {
            return fail(msg::NameOutcome::unknown_event, obj ? obj->id : 0, obj ? obj->revision : 0);
        }
        if (obj && seed->sensor != obj->sensor) return fail(msg::NameOutcome::state_mismatch, obj->id, obj->revision);
        if (!draft.enough(seed->keys) || draft.overlaps_anchored(seed->keys)) {
            return fail(msg::NameOutcome::state_mismatch, obj ? obj->id : 0, obj ? obj->revision : 0);
        }
        const bool have_link = obj && std::any_of(obj->links.begin(), obj->links.end(), [&](const ObjectLinkRecord& link) {
            return link.event_generation == request.event_generation && link.event_id == request.event_id;
        });
        if (obj && !have_link && obj->links.size() == kRegistryLinkCap) {
            return fail(msg::NameOutcome::state_mismatch, obj->id, obj->revision);
        }
        const Keys anchor_keys = seed->keys;
        const std::uint8_t sensor = seed->sensor;
        std::erase_if(draft.seeds_, [&](const Seed& row) {
            return row.event_generation == request.event_generation && row.event_id == request.event_id;
        });
        std::uint64_t id = 0;
        std::uint64_t revision = 0;
        if (!obj) {
            Object created;
            created.id = draft.next_id_++;
            created.revision = 1;
            created.name = request.name;
            created.sensor = sensor;
            created.baseline = msg::Baseline::anchored;
            created.anchor = Anchor{draft.frame_epoch_, anchor_keys, request.event_generation, request.event_id};
            created.links.push_back(ObjectLinkRecord{request.event_generation, request.event_id, msg::EventKind::appeared, false});
            id = created.id;
            revision = created.revision;
            draft.objects_.push_back(std::move(created));
        } else {
            obj->baseline = msg::Baseline::anchored;
            obj->anchor = Anchor{draft.frame_epoch_, anchor_keys, request.event_generation, request.event_id};
            ++obj->revision;
            if (!have_link) {
                obj->links.push_back(ObjectLinkRecord{request.event_generation, request.event_id, msg::EventKind::appeared, false});
            }
            obj->association = msg::Association::none;
            obj->association_generation = 0;
            obj->association_event_id = 0;
            id = obj->id;
            revision = obj->revision;
        }
        return store_and_commit(std::move(draft), request, respond(msg::NameOutcome::applied, id, revision), now_ns, true);
    }

    Object* obj = draft.find_object(request.object_id);
    if (!obj) return fail(msg::NameOutcome::unknown_object, request.object_id, 0);
    if (obj->revision != request.expected_revision) return fail(msg::NameOutcome::stale_revision, obj->id, obj->revision);
    obj->name = request.name;
    ++obj->revision;
    const std::uint64_t id = obj->id;
    const std::uint64_t revision = obj->revision;
    return store_and_commit(std::move(draft), request, respond(msg::NameOutcome::applied, id, revision), now_ns, true);
}

msg::ObjectRegistry Registry::to_message(const msg::Header& header, std::uint64_t publisher_generation) const {
    msg::ObjectRegistry message;
    message.header = header;
    message.snapshot_seq = snapshot_seq_;
    message.publisher_generation = publisher_generation;
    std::vector<const Object*> ordered;
    ordered.reserve(objects_.size());
    for (const Object& obj : objects_) ordered.push_back(&obj);
    std::sort(ordered.begin(), ordered.end(), [](const Object* a, const Object* b) { return a->id < b->id; });
    for (const Object* src : ordered) {
        msg::RegistryObject obj;
        obj.object_id = src->id;
        obj.revision = src->revision;
        obj.name = src->name;
        obj.baseline = src->baseline;
        obj.association = src->association;
        obj.association_generation = src->association_generation;
        obj.association_event_id = src->association_event_id;
        obj.sensor = src->sensor;
        if (src->baseline == msg::Baseline::gone || !src->anchor) {
            obj.frame_epoch = 0;
        } else {
            const Region region = region_from_poses(Region::Kind::appeared, src->anchor->keys, {}, voxel_size_);
            obj.frame_epoch = src->anchor->frame_epoch;
            obj.centroid = region.centroid;
            obj.aabb = std::array<float, 6>{region.bounds_min[0], region.bounds_min[1], region.bounds_min[2],
                                             region.bounds_max[0], region.bounds_max[1], region.bounds_max[2]};
        }
        for (const ObjectLinkRecord& link : src->links) {
            obj.links.push_back(msg::ObjectLink{link.event_generation, link.event_id, link.kind, link.recovered});
        }
        message.objects.push_back(std::move(obj));
    }
    for (const Seed& seed : seeds_) {
        const Region region = region_from_poses(Region::Kind::appeared, seed.keys, {}, voxel_size_);
        msg::NameableSeed row;
        row.event_generation = seed.event_generation;
        row.event_id = seed.event_id;
        row.sensor = seed.sensor;
        row.centroid = region.centroid;
        row.aabb = std::array<float, 6>{region.bounds_min[0], region.bounds_min[1], region.bounds_min[2], region.bounds_max[0],
                                         region.bounds_max[1], region.bounds_max[2]};
        row.voxel_count = static_cast<std::uint32_t>(seed.keys.size());
        message.seeds.push_back(std::move(row));
    }
    for (const DedupeEntry& entry : dedupe_) {
        if (entry.outcome == msg::NameOutcome::applied) message.recent_action_ids.push_back(entry.action_id);
    }
    for (const Receipt& receipt : receipts_) {
        message.recovered_accepts.push_back(msg::RecoveredAccept{receipt.event_generation, receipt.event_id, receipt.kind});
    }
    return message;
}

}  // namespace room_memory
