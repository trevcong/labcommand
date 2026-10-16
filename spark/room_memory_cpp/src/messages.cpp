#include "room_memory/messages.hpp"

#include <zlib.h>

#include <bit>
#include <cmath>
#include <cstring>

#if defined(ROOM_MEMORY_APPLE_LZFSE)
#include <compression.h>
#endif

namespace room_memory::msg {

namespace {

constexpr std::uint32_t kNanBits = 0x7FC00000u;

std::string str(std::int64_t value) { return std::to_string(value); }
std::string str(std::uint64_t value) { return std::to_string(value); }

void require_int(std::int64_t value, std::int64_t low, std::int64_t high, const std::string& what) {
    if (value < low || value > high) throw Invalid(what + " " + str(value) + " is outside " + str(low) + "..." + str(high));
}

// A publisher_generation, when present, is 1 ... 2^64 - 1.
void require_generation(const std::optional<std::uint64_t>& value, const std::string& what) {
    if (value && *value == kGenerationUnknown) {
        throw Invalid(what + " (0 is GENERATION_UNKNOWN) 0 is outside 1..." + str(kUint64Max));
    }
}

void require_finite(const std::array<float, 3>& values, const std::string& what) {
    for (const float value : values) {
        if (!std::isfinite(value)) throw Invalid(what + " is not finite");
    }
}

void require_fraction(const std::optional<float>& value, const std::string& what) {
    if (value && !(*value >= 0 && *value <= 1)) throw Invalid(what + " is outside 0...1");
}

template <typename E>
void require_enum(E value, int count, const std::string& what) {
    if (static_cast<int>(value) >= count) {
        throw Invalid(what + " " + str(static_cast<std::int64_t>(value)) + " is not a valid value");
    }
}

template <typename E>
E read_enum(std::uint8_t value, int count, const std::string& what) {
    if (value >= count) throw Invalid(what + " " + str(static_cast<std::int64_t>(value)) + " is not a valid value");
    return static_cast<E>(value);
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

// MARK: Wire

class Writer {
public:
    Writer() : data_{0x00, 0x01, 0x00, 0x00} {}

    void align(std::size_t size) {
        while ((data_.size() - 4) % size) data_.push_back(0);
    }
    void u8(std::uint8_t value) { data_.push_back(value); }
    void put(std::uint64_t value, std::size_t size) {
        align(size);
        for (std::size_t b = 0; b < size; ++b) data_.push_back(static_cast<std::uint8_t>(value >> (8 * b)));
    }
    void u16(std::uint64_t value) { put(value, 2); }
    void u32(std::uint64_t value) { put(value, 4); }
    void u64(std::uint64_t value) { put(value, 8); }
    void f32(std::optional<float> value) {
        u32(!value || std::isnan(*value) ? kNanBits : std::bit_cast<std::uint32_t>(*value));
    }
    void floats(const std::array<float, 3>& values) {
        for (const float value : values) f32(value);
    }
    void string(const std::string& text) {
        u32(text.size() + 1);
        data_.insert(data_.end(), text.begin(), text.end());
        data_.push_back(0);
    }
    void time(const Time& value) {
        u32(static_cast<std::uint64_t>(value.sec) & 0xFFFFFFFFu);
        u32(static_cast<std::uint64_t>(value.nanosec));
    }
    void header(const Header& value) {
        string(value.room_id);
        u64(value.map_epoch);
        u64(value.seq);
        time(value.stamp);
    }
    void bytes(const Bytes& value) { data_.insert(data_.end(), value.begin(), value.end()); }
    void token(const Token& value) { data_.insert(data_.end(), value.begin(), value.end()); }
    void ranges(const std::vector<SeqRange>& values) {
        u32(values.size());
        for (const SeqRange& range : values) {
            u64(range.first);
            u64(range.last);
        }
    }
    Bytes take() { return std::move(data_); }

private:
    Bytes data_;
};

std::string hex2(std::uint8_t value) {
    static const char digits[] = "0123456789abcdef";
    return {digits[value >> 4], digits[value & 15]};
}

class Reader {
public:
    Reader(std::span<const std::uint8_t> data, std::string message) : data_(data), message_(std::move(message)) {
        if (data.size() < 4) {
            throw Truncated(message_ + " CDR: truncated encapsulation header (" + str(std::uint64_t{data.size()}) +
                            " of 4 bytes)");
        }
        if (data[0] == 0x00 && data[1] == 0x01) {
            little_ = true;
        } else if (data[0] == 0x00 && data[1] == 0x00) {
            little_ = false;
        } else {
            throw Unsupported(message_ + " CDR: unsupported encapsulation 0x" + hex2(data[0]) + " 0x" + hex2(data[1]) +
                              " (expected CDR_LE 0x00 0x01 or CDR_BE 0x00 0x00)");
        }
    }

    std::size_t available() const { return offset_ < data_.size() ? data_.size() - offset_ : 0; }

    std::size_t take(std::size_t length, const std::string& what) {
        if (offset_ > data_.size() || length > data_.size() - offset_) {
            throw Truncated(message_ + " CDR: truncated " + what + " at byte " + str(std::uint64_t{offset_}) + " (need " +
                            str(std::uint64_t{length}) + ", have " + str(std::uint64_t{available()}) + ")");
        }
        const std::size_t start = offset_;
        offset_ += length;
        return start;
    }

    void align(std::size_t size) {
        const std::size_t misalignment = (offset_ - 4) % size;
        if (misalignment) offset_ += size - misalignment;
    }

    std::uint64_t get(std::size_t size, const std::string& what) {
        align(size);
        const std::uint8_t* p = data_.data() + take(size, what);
        std::uint64_t value = 0;
        for (std::size_t b = 0; b < size; ++b) {
            value |= std::uint64_t{p[little_ ? b : size - 1 - b]} << (8 * b);
        }
        return value;
    }

    std::uint8_t u8(const std::string& what) { return data_[take(1, what)]; }
    std::uint16_t u16(const std::string& what) { return static_cast<std::uint16_t>(get(2, what)); }
    std::uint32_t u32(const std::string& what) { return static_cast<std::uint32_t>(get(4, what)); }
    std::int32_t i32(const std::string& what) { return static_cast<std::int32_t>(u32(what)); }
    std::uint64_t u64(const std::string& what) { return get(8, what); }
    float f32(const std::string& what) { return std::bit_cast<float>(u32(what)); }

    std::array<float, 3> floats(const std::string& what) {
        std::array<float, 3> out{};
        for (int n = 0; n < 3; ++n) out[n] = f32(what + "[" + str(std::int64_t{n}) + "]");
        return out;
    }

    std::string string(const std::string& what) {
        const std::uint32_t length = u32(what + " length");
        const std::size_t start = take(length, what);
        if (length < 1 || data_[start + length - 1] != 0) throw Invalid(message_ + " CDR: " + what + " is not NUL-terminated");
        const std::string text(reinterpret_cast<const char*>(data_.data() + start), length - 1);
        if (text.find('\0') != std::string::npos || !valid_utf8(text)) {
            throw Invalid(message_ + " CDR: " + what + " is not NUL-free UTF-8");
        }
        return text;
    }

    Time time(const std::string& what) {
        Time value;
        value.sec = i32(what + ".sec");
        value.nanosec = u32(what + ".nanosec");
        return value;
    }

    Header header(const std::string& path) {
        Header value;
        value.room_id = string(path + "room_id");
        value.map_epoch = u64(path + "map_epoch");
        value.seq = u64(path + "seq");
        value.stamp = time(path + "stamp");
        return value;
    }

    Bytes byte_sequence(const std::string& what) {
        const std::uint32_t count = u32(what + " length");
        if (count > available()) {
            throw Truncated(message_ + " CDR: truncated " + what + " at byte " + str(std::uint64_t{offset_}) + " (" +
                            str(std::uint64_t{count}) + " bytes, have " + str(std::uint64_t{available()}) + ")");
        }
        const std::size_t start = take(count, what);
        return Bytes(data_.begin() + static_cast<std::ptrdiff_t>(start),
                     data_.begin() + static_cast<std::ptrdiff_t>(start + count));
    }

    Token token(const std::string& what) {
        Token value;
        std::memcpy(value.data(), data_.data() + take(kTokenBytes, what), kTokenBytes);
        return value;
    }

    // u32 count, then (first, last) u64 pairs. A count the remaining bytes cannot hold is
    // refused before anything is allocated; the first u64 aligns after the count.
    std::vector<SeqRange> ranges(const std::string& what) {
        const std::uint32_t count = u32(what + " length");
        if (count > available() / 16) {
            throw Truncated(message_ + " CDR: truncated " + what + " at byte " + str(std::uint64_t{offset_}) + " (" +
                            str(std::uint64_t{count}) + " ranges, have " + str(std::uint64_t{available()}) + " bytes)");
        }
        std::vector<SeqRange> out(count);
        for (std::uint32_t n = 0; n < count; ++n) {
            const std::string at = what + "[" + str(std::uint64_t{n}) + "]";
            out[n].first = u64(at + ".first");
            out[n].last = u64(at + ".last");
        }
        return out;
    }

    // Decides the layout after the last legacy field: false when at most 3 bytes are left
    // (legacy), true when padding to 8 and then exactly `size` bytes (and at most 3) are left
    // (v2). Throws Invalid otherwise. Pad bytes are not checked, as with every alignment.
    bool is_v2(std::size_t size, const std::string& what) const {
        const std::size_t unread = available();
        if (unread <= kMaxTrailingPadding) return false;
        const std::size_t pad = (8 - (offset_ - 4) % 8) % 8;
        if (unread >= pad && unread - pad >= size && unread - pad <= size + kMaxTrailingPadding) return true;
        throw Invalid(message_ + " CDR: " + str(std::uint64_t{unread}) + " bytes after " + what +
                      " are neither padding (at most " + str(std::uint64_t{kMaxTrailingPadding}) + ") nor the " +
                      str(std::uint64_t{size}) + "-byte v2 fields");
    }

    void finish() const {
        const std::size_t trailing = available();
        if (trailing > kMaxTrailingPadding) {
            throw Invalid(message_ + " CDR: " + str(std::uint64_t{trailing}) + " bytes after the message (at most " +
                          str(std::uint64_t{kMaxTrailingPadding}) + " of padding)");
        }
    }

private:
    std::span<const std::uint8_t> data_;
    std::string message_;
    std::size_t offset_ = 4;
    bool little_ = true;
};

// MARK: MapBricks payload

Bytes pack_payload(const std::vector<Brick>& bricks, const std::vector<std::int64_t>& removed) {
    Bytes raw;
    raw.reserve(payload_size(bricks.size(), removed.size()));
    for (const Brick& brick : bricks) {
        for (const std::int64_t value : brick.coordinate) {
            const auto bits = static_cast<std::uint16_t>(static_cast<std::int16_t>(value));
            raw.push_back(static_cast<std::uint8_t>(bits));
            raw.push_back(static_cast<std::uint8_t>(bits >> 8));
        }
    }
    for (const Brick& brick : bricks) raw.insert(raw.end(), brick.occupied.begin(), brick.occupied.end());
    for (const Brick& brick : bricks) raw.insert(raw.end(), brick.known.begin(), brick.known.end());
    for (const std::int64_t value : removed) {
        for (int b = 0; b < 4; ++b) raw.push_back(static_cast<std::uint8_t>(static_cast<std::uint64_t>(value) >> (8 * b)));
    }
    return raw;
}

void unpack_payload(const Bytes& raw, std::size_t brick_count, std::size_t removed_count, std::vector<Brick>& bricks,
                    std::vector<std::int64_t>& removed) {
    const std::size_t occupied_start = brick_count * kCoordinateBytes;
    const std::size_t known_start = occupied_start + brick_count * kMaskBytes;
    const std::size_t removed_start = known_start + brick_count * kMaskBytes;
    bricks.resize(brick_count);
    for (std::size_t n = 0; n < brick_count; ++n) {
        for (int a = 0; a < 3; ++a) {
            const std::size_t at = n * kCoordinateBytes + 2 * static_cast<std::size_t>(a);
            bricks[n].coordinate[a] = static_cast<std::int16_t>(raw[at] | raw[at + 1] << 8);
        }
        std::memcpy(bricks[n].occupied.data(), raw.data() + occupied_start + n * kMaskBytes, kMaskBytes);
        std::memcpy(bricks[n].known.data(), raw.data() + known_start + n * kMaskBytes, kMaskBytes);
    }
    removed.resize(removed_count);
    for (std::size_t n = 0; n < removed_count; ++n) {
        const std::uint8_t* p = raw.data() + removed_start + 4 * n;
        removed[n] = static_cast<std::int64_t>(std::uint32_t{p[0]} | std::uint32_t{p[1]} << 8 | std::uint32_t{p[2]} << 16 |
                                               std::uint32_t{p[3]} << 24);
    }
}

// Raw DEFLATE, level 6, 32 KiB window, memLevel 8, default strategy, fed like Python's
// zlib.compressobj(6, DEFLATED, -15): all input without flushing, then Z_FINISH.
Bytes deflate_raw(const Bytes& raw) {
    z_stream stream{};
    if (deflateInit2(&stream, 6, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY) != Z_OK) throw std::bad_alloc();
    Bytes out(deflateBound(&stream, static_cast<uLong>(raw.size())));
    stream.next_in = const_cast<Bytef*>(raw.data());
    stream.avail_in = static_cast<uInt>(raw.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    deflate(&stream, Z_NO_FLUSH);
    const int status = deflate(&stream, Z_FINISH);
    out.resize(stream.total_out);
    deflateEnd(&stream);
    if (status != Z_STREAM_END) throw std::runtime_error("zlib deflate did not finish");
    return out;
}

Bytes compress(const Bytes& raw, Compression compression) {
    if (compression == Compression::none) return raw;
    if (compression == Compression::zlib) return deflate_raw(raw);
    throw Unsupported("MapBricks: this encoder writes COMPRESSION_NONE or COMPRESSION_ZLIB; the Spark sends zlib");
}

[[noreturn]] void not_one_stream(const std::string& what, std::size_t length, const char* codec, std::size_t expected) {
    throw Invalid(what + ": " + str(std::uint64_t{length}) + " " + codec + " bytes are not one complete stream of " +
                  str(std::uint64_t{expected}) + " bytes");
}

#if defined(ROOM_MEMORY_APPLE_LZFSE)
// compression_decode_buffer (and Apple's stream decoder) stop at the first end-of-stream block and
// ignore whatever follows it. A complete stream ends in its 4-byte "bvx$", so it must not decode
// without its last byte: if it does, the payload carries trailing bytes or a second stream.
Bytes inflate_lzfse_apple(const Bytes& payload, std::size_t expected, const std::string& what) {
    Bytes out(expected + 1);
    const auto decodes = [&](std::size_t length) {
        return compression_decode_buffer(out.data(), out.size(), payload.data(), length, nullptr, COMPRESSION_LZFSE) ==
               expected;
    };
    // compression_decode_buffer returns 0 on failure, so an empty result is only accepted from a bare "bvx$" (handled by the caller).
    // The caller has seen a block magic, so the payload holds at least 4 bytes.
    if (expected == 0 || decodes(payload.size() - 1) || !decodes(payload.size())) {
        not_one_stream(what, payload.size(), "LZFSE", expected);
    }
    out.resize(expected);
    return out;
}
#endif

// Uncompressed (bvx-) blocks are read here; a compressed block needs Apple's framework.
Bytes inflate_lzfse(const Bytes& payload, std::size_t expected, const std::string& what) {
    Bytes out;
    std::size_t offset = 0;
    const auto magic_is = [&](const char* magic) {
        return offset + 4 <= payload.size() && std::memcmp(payload.data() + offset, magic, 4) == 0;
    };
    while (true) {
        if (magic_is("bvx$")) {
            offset += 4;
            break;
        }
        if (magic_is("bvx1") || magic_is("bvx2") || magic_is("bvxn")) {
#if defined(ROOM_MEMORY_APPLE_LZFSE)
            return inflate_lzfse_apple(payload, expected, what);
#else
            throw Unsupported(what + ": compressed LZFSE blocks need Apple's Compression framework");
#endif
        }
        if (!magic_is("bvx-") || offset + 8 > payload.size()) not_one_stream(what, payload.size(), "LZFSE", expected);
        const std::uint8_t* p = payload.data() + offset + 4;
        const std::size_t count =
            std::uint32_t{p[0]} | std::uint32_t{p[1]} << 8 | std::uint32_t{p[2]} << 16 | std::uint32_t{p[3]} << 24;
        if (out.size() + count > expected || offset + 8 + count > payload.size()) {
            not_one_stream(what, payload.size(), "LZFSE", expected);
        }
        out.insert(out.end(), payload.begin() + static_cast<std::ptrdiff_t>(offset + 8),
                   payload.begin() + static_cast<std::ptrdiff_t>(offset + 8 + count));
        offset += 8 + count;
    }
    if (offset != payload.size() || out.size() != expected) not_one_stream(what, payload.size(), "LZFSE", expected);
    return out;
}

// One complete raw DEFLATE stream of exactly `expected` bytes, nothing after it.
Bytes inflate_raw(const Bytes& payload, std::size_t expected, const std::string& what) {
    z_stream stream{};
    if (inflateInit2(&stream, -15) != Z_OK) throw std::bad_alloc();
    Bytes out(expected + 1);
    stream.next_in = const_cast<Bytef*>(payload.data());
    stream.avail_in = static_cast<uInt>(payload.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    const int status = inflate(&stream, Z_FINISH);
    const std::size_t produced = stream.total_out;
    const std::size_t left = stream.avail_in;
    inflateEnd(&stream);
    if (status == Z_DATA_ERROR || status == Z_MEM_ERROR || status == Z_NEED_DICT) {
        throw Invalid(what + ": " + str(std::uint64_t{payload.size()}) + " ZLIB bytes do not inflate");
    }
    if (status != Z_STREAM_END || produced != expected || left != 0) not_one_stream(what, payload.size(), "ZLIB", expected);
    out.resize(expected);
    return out;
}

Bytes decompress(const Bytes& payload, Compression compression, std::size_t expected, const std::string& what) {
    if (compression == Compression::none) {
        if (payload.size() != expected) {
            throw Invalid(what + " holds " + str(std::uint64_t{payload.size()}) + " bytes, expected " +
                          str(std::uint64_t{expected}));
        }
        return payload;
    }
    if (payload.empty()) throw Invalid(what + ": an empty " + name(compression) + " payload is not a stream");
    if (compression == Compression::lzfse) return inflate_lzfse(payload, expected, what);
    return inflate_raw(payload, expected, what);
}

void write_map_bricks(Writer& writer, const MapBricks& m) {
    writer.header(m.header);
    writer.u8(static_cast<std::uint8_t>(m.compression));
    writer.u8(static_cast<std::uint8_t>(m.kind));
    writer.u64(m.base_epoch);
    writer.f32(m.voxel_size);
    writer.u8(kBrickDim);
    writer.u32(m.bricks.size());
    writer.u32(m.removed.size());
    const Bytes payload = compress(pack_payload(m.bricks, m.removed), m.compression);
    writer.u32(payload.size());
    writer.bytes(payload);
    if (m.publisher_generation) writer.u64(*m.publisher_generation);
}

// `tail` is the size of the v2 fields that end the buffer from here: 8 at the top level, 16 inside
// ChangeEvent.mask (the mask's generation, then the parent's). The layout is decided after the payload.
MapBricks read_map_bricks(Reader& reader, const std::string& path, std::size_t tail) {
    MapBricks m;
    m.header = reader.header(path);
    m.compression = read_enum<Compression>(reader.u8(path + "compression"), 3, "MapBricks: " + path + "compression");
    m.kind = read_enum<MapKind>(reader.u8(path + "kind"), 2, "MapBricks: " + path + "kind");
    m.base_epoch = reader.u64(path + "base_epoch");
    m.voxel_size = reader.f32(path + "voxel_size");
    const std::uint8_t brick_dim = reader.u8(path + "brick_dim");
    if (brick_dim != kBrickDim) {
        throw Unsupported("MapBricks: " + path + "brick_dim " + str(std::int64_t{brick_dim}) + "; v1 handles only 8");
    }
    const std::uint32_t brick_count = reader.u32(path + "n_bricks");
    const std::uint32_t removed_count = reader.u32(path + "n_removed");
    const std::uint64_t size = payload_size(brick_count, removed_count);
    if (size > kMaxPayloadBytes) {
        throw Invalid("MapBricks: " + path + "n_bricks " + str(std::uint64_t{brick_count}) + " and n_removed " +
                      str(std::uint64_t{removed_count}) + " need a " + str(size) + "-byte payload; the limit is " +
                      str(std::uint64_t{kMaxPayloadBytes}));
    }
    const Bytes payload = reader.byte_sequence(path + "payload");
    const Bytes raw = decompress(payload, m.compression, static_cast<std::size_t>(size), "MapBricks: " + path + "payload");
    unpack_payload(raw, brick_count, removed_count, m.bricks, m.removed);
    if (reader.is_v2(tail, path + "payload")) m.publisher_generation = reader.u64(path + "publisher_generation");
    return m;
}

template <typename T, typename Body>
T decode(std::span<const std::uint8_t> data, const char* message, Body&& body) {
    Reader reader(data, message);
    T value = body(reader);
    reader.finish();
    return value;
}

bool time_after(const Time& a, const Time& b) {
    return a.sec != b.sec ? a.sec > b.sec : a.nanosec > b.nanosec;
}

std::optional<float> maybe(float value) { return std::isnan(value) ? std::nullopt : std::optional<float>(value); }

}  // namespace

const char* name(MemoryState state) {
    switch (state) {
        case MemoryState::building: return "BUILDING";
        case MemoryState::tracking: return "TRACKING";
        case MemoryState::rig_moved: return "RIG_MOVED";
        case MemoryState::degraded: return "DEGRADED";
        case MemoryState::paused: return "PAUSED";
    }
    return "UNKNOWN";
}

const char* name(Compression compression) {
    switch (compression) {
        case Compression::none: return "NONE";
        case Compression::zlib: return "ZLIB";
        case Compression::lzfse: return "LZFSE";
    }
    return "UNKNOWN";
}

bool lzfse_blocks_supported() {
#if defined(ROOM_MEMORY_APPLE_LZFSE)
    return true;
#else
    return false;
#endif
}

std::uint64_t payload_size(std::uint64_t bricks, std::uint64_t removed) { return bricks * kBytesPerBrick + removed * 4; }

void Time::validate(const std::string& what) const {
    require_int(sec, -(std::int64_t{1} << 31), (std::int64_t{1} << 31) - 1, what + ".sec");
    require_int(nanosec, 0, (std::int64_t{1} << 32) - 1, what + ".nanosec");
    if (nanosec >= 1'000'000'000) throw Invalid(what + ".nanosec " + str(nanosec) + " is not below 1e9");
}

void Header::validate(const std::string& context) const {
    if (room_id.empty()) throw Invalid(context + ": room_id is empty");
    if (room_id.find('\0') != std::string::npos) throw Invalid(context + ": room_id contains a NUL");
    if (!valid_utf8(room_id)) throw Invalid(context + ": room_id is not UTF-8");
    stamp.validate(context + ": stamp");
}

void MapBricks::validate(const std::string& context) const {
    header.validate(context);
    require_generation(publisher_generation, context + ": publisher_generation");
    require_enum(compression, 3, context + ": compression");
    require_enum(kind, 2, context + ": kind");
    if (!(std::isfinite(voxel_size) && voxel_size > 0)) throw Invalid(context + ": voxel_size must be finite and > 0 m");
    if (kind == MapKind::delta && header.map_epoch <= base_epoch) {
        throw Invalid(context + ": delta map_epoch " + str(header.map_epoch) + " is not after base_epoch " + str(base_epoch));
    }
    if (kind == MapKind::snapshot && !removed.empty()) {
        throw Invalid(context + ": a snapshot carries " + str(std::uint64_t{removed.size()}) + " removed_bricks_idx");
    }
    for (std::size_t n = 0; n < removed.size(); ++n) {
        require_int(removed[n], 0, (std::int64_t{1} << 32) - 1, context + ": removed_bricks_idx[" + str(std::uint64_t{n}) + "]");
        if (n && removed[n - 1] >= removed[n]) {
            throw Invalid(context + ": removed_bricks_idx not strictly increasing at " + str(std::uint64_t{n}));
        }
    }
    const std::uint64_t size = payload_size(bricks.size(), removed.size());
    if (size > kMaxPayloadBytes) {
        throw Invalid(context + ": " + str(std::uint64_t{bricks.size()}) + " bricks need a " + str(size) +
                      "-byte payload; the limit is " + str(std::uint64_t{kMaxPayloadBytes}));
    }
    for (std::size_t n = 0; n < bricks.size(); ++n) {
        const Brick& brick = bricks[n];
        const char* axes = "ijk";
        for (int a = 0; a < 3; ++a) {
            require_int(brick.coordinate[a], -(1 << 15), (1 << 15) - 1,
                        context + ": brick " + str(std::uint64_t{n}) + " " + axes[a]);
        }
        if (n && !(bricks[n - 1].coordinate < brick.coordinate)) {
            throw Invalid(context + ": brick " + str(std::uint64_t{n}) + " is not after brick " + str(std::uint64_t{n - 1}) +
                          " in (i, j, k) order");
        }
        bool any_known = false;
        for (std::size_t b = 0; b < kMaskBytes; ++b) {
            if (brick.occupied[b] & ~brick.known[b] & 0xFF) {
                throw Invalid(context + ": brick " + str(std::uint64_t{n}) + " marks a voxel occupied that is not known");
            }
            any_known = any_known || brick.known[b];
        }
        if (!any_known) {
            throw Invalid(context + ": brick " + str(std::uint64_t{n}) +
                          " has no known voxel; an all-unknown brick is omitted");
        }
    }
}

Bytes MapBricks::to_cdr() const {
    validate();
    Writer writer;
    write_map_bricks(writer, *this);
    return writer.take();
}

MapBricks MapBricks::from_cdr(std::span<const std::uint8_t> data) {
    return decode<MapBricks>(data, "MapBricks", [](Reader& reader) {
        MapBricks m = read_map_bricks(reader, "", 8);
        m.validate();
        return m;
    });
}

void ChangeEvent::validate() const {
    const std::string context = "ChangeEvent";
    header.validate(context);
    require_generation(publisher_generation, context + ": publisher_generation");
    require_enum(kind, 3, context + ": kind");
    require_enum(state, 5, context + ": state");
    require_finite(centroid, context + ": centroid");
    require_finite(aabb_min, context + ": aabb min");
    require_finite(aabb_max, context + ": aabb max");
    for (int a = 0; a < 3; ++a) {
        if (aabb_min[a] > aabb_max[a]) throw Invalid(context + ": aabb min exceeds max");
    }
    if (kind == EventKind::moved && !from_centroid) throw Invalid(context + ": MOVED needs from_centroid");
    if (kind != EventKind::moved && from_centroid) throw Invalid(context + ": from_centroid is only for MOVED");
    if (from_centroid) require_finite(*from_centroid, context + ": from_centroid");
    require_int(voxel_count, 0, (std::int64_t{1} << 32) - 1, context + ": voxel_count");
    if (!(std::isfinite(volume_m3) && volume_m3 >= 0)) throw Invalid(context + ": volume_m3 must be finite and >= 0");
    require_fraction(evidence, context + ": evidence");
    require_fraction(alpha, context + ": alpha");
    require_int(sensors, 0, 0xFF, context + ": sensors");
    if (sensors & ~kSensorsAll) throw Invalid(context + ": sensors sets an undefined bit");
    first_seen.validate(context + ": first_seen");
    last_seen.validate(context + ": last_seen");
    if (time_after(first_seen, last_seen)) throw Invalid(context + ": last_seen is before first_seen");
    if (mask) {
        if (mask->kind != MapKind::snapshot) throw Invalid(context + ": mask must be KIND_SNAPSHOT");
        mask->validate(context + " mask[0]");
        if (mask->header.room_id != header.room_id) throw Invalid(context + ": mask room_id is not the event's");
        if (mask->header.map_epoch != header.map_epoch) throw Invalid(context + ": mask map_epoch is not the event's");
        if (!(mask->header.stamp == header.stamp)) throw Invalid(context + ": mask stamp is not the event's stamp");
        if (mask->header.seq != 0) throw Invalid(context + ": mask seq must be 0");
        if (mask->base_epoch != 0) throw Invalid(context + ": mask base_epoch must be 0");
        if (mask->publisher_generation != publisher_generation) {
            const auto text = [](const std::optional<std::uint64_t>& g) { return g ? str(*g) : std::string("None"); };
            throw Invalid(context + ": mask publisher_generation " + text(mask->publisher_generation) +
                          " is not the event's " + text(publisher_generation) +
                          " (a legacy event has a legacy mask, a v2 event a v2 mask)");
        }
    }
}

Bytes ChangeEvent::to_cdr() const {
    validate();
    Writer writer;
    writer.header(header);
    writer.u64(event_id);
    writer.u8(static_cast<std::uint8_t>(kind));
    writer.u8(static_cast<std::uint8_t>(state));
    writer.floats(centroid);
    writer.floats(aabb_min);
    writer.floats(aabb_max);
    if (from_centroid) {
        writer.floats(*from_centroid);
    } else {
        for (int n = 0; n < 3; ++n) writer.f32(std::nullopt);
    }
    writer.u32(static_cast<std::uint64_t>(voxel_count));
    writer.f32(volume_m3);
    writer.f32(evidence);
    writer.f32(alpha);
    writer.u8(static_cast<std::uint8_t>(sensors));
    writer.time(first_seen);
    writer.time(last_seen);
    writer.u32(mask ? 1 : 0);
    if (mask) write_map_bricks(writer, *mask);
    if (publisher_generation) writer.u64(*publisher_generation);
    return writer.take();
}

ChangeEvent ChangeEvent::from_cdr(std::span<const std::uint8_t> data) {
    return decode<ChangeEvent>(data, "ChangeEvent", [](Reader& reader) {
        ChangeEvent e;
        e.header = reader.header("");
        e.event_id = reader.u64("event_id");
        e.kind = read_enum<EventKind>(reader.u8("kind"), 3, "ChangeEvent: kind");
        e.state = read_enum<EventState>(reader.u8("state"), 5, "ChangeEvent: state");
        e.centroid = reader.floats("centroid");
        e.aabb_min = reader.floats("aabb");
        e.aabb_max = reader.floats("aabb");
        const std::array<float, 3> from = reader.floats("from_centroid");
        const int nans = std::isnan(from[0]) + std::isnan(from[1]) + std::isnan(from[2]);
        if (nans != 0 && nans != 3) throw Invalid("ChangeEvent: from_centroid is partly NaN");
        if (nans == 0) e.from_centroid = from;
        e.voxel_count = reader.u32("voxel_count");
        e.volume_m3 = reader.f32("volume_m3");
        e.evidence = maybe(reader.f32("evidence"));
        e.alpha = maybe(reader.f32("alpha"));
        e.sensors = reader.u8("sensors");
        e.first_seen = reader.time("first_seen");
        e.last_seen = reader.time("last_seen");
        const std::uint32_t mask_count = reader.u32("mask length");
        if (mask_count > 1) throw Invalid("ChangeEvent: mask holds " + str(std::uint64_t{mask_count}) + " maps; the bound is 1");
        // With a mask, the nested reader decides the layout after its payload, for both messages.
        if (mask_count) e.mask = read_map_bricks(reader, "mask[0].", 16);
        const bool v2 = e.mask ? e.mask->publisher_generation.has_value() : reader.is_v2(8, "mask length");
        if (v2) e.publisher_generation = reader.u64("publisher_generation");
        e.validate();
        return e;
    });
}

void MemoryStatus::validate() const {
    const std::string context = "MemoryStatus";
    header.validate(context);
    require_generation(publisher_generation, context + ": publisher_generation");
    require_enum(state, 5, context + ": state");
    require_int(n_bricks, 0, (std::int64_t{1} << 32) - 1, context + ": n_bricks");
    require_int(n_open_events, 0, (std::int64_t{1} << 32) - 1, context + ": n_open_events");
    require_fraction(coverage_known_frac, context + ": coverage_known_frac");
    if (pose_residual_m && !(std::isfinite(*pose_residual_m) && *pose_residual_m >= 0)) {
        throw Invalid(context + ": pose_residual_m must be finite and >= 0");
    }
    if (window_s && !(std::isfinite(*window_s) && *window_s > 0)) throw Invalid(context + ": window_s must be finite and > 0");
    if (mid360_hz) require_int(*mid360_hz, 0, kHzUnavailable - 1, context + ": mid360_hz (65535 is the unavailable sentinel)");
    if (jt128_hz) require_int(*jt128_hz, 0, kHzUnavailable - 1, context + ": jt128_hz (65535 is the unavailable sentinel)");
    require_int(dropped_windows, 0, (std::int64_t{1} << 32) - 1, context + ": dropped_windows");
    if (events_seq == kEventsSeqNone) throw Invalid(context + ": events_seq (2^64 - 1 is EVENTS_SEQ_NONE; use nullopt)");
}

Bytes MemoryStatus::to_cdr() const {
    validate();
    Writer writer;
    writer.header(header);
    writer.u8(static_cast<std::uint8_t>(state));
    writer.u32(static_cast<std::uint64_t>(n_bricks));
    writer.u32(static_cast<std::uint64_t>(n_open_events));
    writer.f32(coverage_known_frac);
    writer.f32(pose_residual_m);
    writer.f32(window_s);
    writer.u16(static_cast<std::uint64_t>(mid360_hz.value_or(kHzUnavailable)));
    writer.u16(static_cast<std::uint64_t>(jt128_hz.value_or(kHzUnavailable)));
    writer.u32(static_cast<std::uint64_t>(dropped_windows));
    writer.u64(events_seq.value_or(kEventsSeqNone));
    if (publisher_generation) writer.u64(*publisher_generation);
    return writer.take();
}

MemoryStatus MemoryStatus::from_cdr(std::span<const std::uint8_t> data) {
    return decode<MemoryStatus>(data, "MemoryStatus", [](Reader& reader) {
        MemoryStatus s;
        s.header = reader.header("");
        s.state = read_enum<MemoryState>(reader.u8("state"), 5, "MemoryStatus: state");
        s.n_bricks = reader.u32("n_bricks");
        s.n_open_events = reader.u32("n_open_events");
        s.coverage_known_frac = maybe(reader.f32("coverage_known_frac"));
        s.pose_residual_m = maybe(reader.f32("pose_residual_m"));
        s.window_s = maybe(reader.f32("window_s"));
        const std::uint16_t mid = reader.u16("per_sensor_hz[0]");
        const std::uint16_t jt = reader.u16("per_sensor_hz[1]");
        if (mid != kHzUnavailable) s.mid360_hz = mid;
        if (jt != kHzUnavailable) s.jt128_hz = jt;
        s.dropped_windows = reader.u32("dropped_windows");
        const std::uint64_t events_seq = reader.u64("events_seq");
        if (events_seq != kEventsSeqNone) s.events_seq = events_seq;
        if (reader.is_v2(8, "events_seq")) s.publisher_generation = reader.u64("publisher_generation");
        s.validate();
        return s;
    });
}

void Epoch::validate() const {
    header.validate("Epoch");
    require_int(n_appeared, 0, (std::int64_t{1} << 32) - 1, "Epoch: n_appeared");
    require_int(n_disappeared, 0, (std::int64_t{1} << 32) - 1, "Epoch: n_disappeared");
    require_int(n_moved, 0, (std::int64_t{1} << 32) - 1, "Epoch: n_moved");
    require_fraction(coverage_known_frac, "Epoch: coverage_known_frac");
    require_enum(state, 5, "Epoch: state");
}

Bytes Epoch::to_cdr() const {
    validate();
    Writer writer;
    writer.header(header);
    writer.u32(static_cast<std::uint64_t>(n_appeared));
    writer.u32(static_cast<std::uint64_t>(n_disappeared));
    writer.u32(static_cast<std::uint64_t>(n_moved));
    writer.f32(coverage_known_frac);
    writer.u8(static_cast<std::uint8_t>(state));
    return writer.take();
}

Epoch Epoch::from_cdr(std::span<const std::uint8_t> data) {
    return decode<Epoch>(data, "Epoch", [](Reader& reader) {
        Epoch e;
        e.header = reader.header("");
        e.n_appeared = reader.u32("n_appeared");
        e.n_disappeared = reader.u32("n_disappeared");
        e.n_moved = reader.u32("n_moved");
        e.coverage_known_frac = maybe(reader.f32("coverage_known_frac"));
        e.state = read_enum<MemoryState>(reader.u8("state"), 5, "Epoch: state");
        e.validate();
        return e;
    });
}

void Activity::validate() const {
    const std::string context = "Activity";
    header.validate(context);
    require_int(sensors, 0, 0xFF, context + ": sensors");
    if (sensors & ~kSensorsAll) throw Invalid(context + ": sensors sets an undefined bit");
    require_int(largest_sensor, 0, 0xFF, context + ": largest_sensor");
    require_int(n_regions, 0, (std::int64_t{1} << 32) - 1, context + ": n_regions");
    require_int(voxel_count, 0, (std::int64_t{1} << 32) - 1, context + ": voxel_count");
    if (active) {
        if (n_regions < 1 || voxel_count < 1) throw Invalid(context + ": active needs at least one region and voxel");
        if ((largest_sensor != kSensorMid360 && largest_sensor != kSensorJT128) || !(sensors & largest_sensor)) {
            throw Invalid(context + ": largest_sensor is not one bit of sensors");
        }
        if (!centroid || !aabb_min || !aabb_max) throw Invalid(context + ": active needs the largest region's centroid and aabb");
        require_finite(*centroid, context + ": centroid");
        require_finite(*aabb_min, context + ": aabb min");
        require_finite(*aabb_max, context + ": aabb max");
        for (int a = 0; a < 3; ++a) {
            if ((*aabb_min)[a] > (*aabb_max)[a]) throw Invalid(context + ": aabb min exceeds max");
        }
        if (!since_s) throw Invalid(context + ": active needs since_s");
    } else {
        if (n_regions || voxel_count || sensors || largest_sensor) {
            throw Invalid(context + ": inactive must have zero counts and sensor bits");
        }
        if (centroid || aabb_min || aabb_max) throw Invalid(context + ": inactive must have NaN geometry");
    }
    if (since_s && !(std::isfinite(*since_s) && *since_s >= 0)) {
        throw Invalid(context + ": since_s must be finite and >= 0, or NaN");
    }
}

Bytes Activity::to_cdr() const {
    validate();
    Writer writer;
    writer.header(header);
    writer.u8(active ? 1 : 0);
    writer.u8(static_cast<std::uint8_t>(sensors));
    writer.u8(static_cast<std::uint8_t>(largest_sensor));
    writer.u32(static_cast<std::uint64_t>(n_regions));
    writer.u32(static_cast<std::uint64_t>(voxel_count));
    for (const auto* values : {&centroid, &aabb_min, &aabb_max}) {
        if (*values) {
            writer.floats(**values);
        } else {
            for (int n = 0; n < 3; ++n) writer.f32(std::nullopt);
        }
    }
    writer.f32(since_s);
    return writer.take();
}

Activity Activity::from_cdr(std::span<const std::uint8_t> data) {
    return decode<Activity>(data, "Activity", [](Reader& reader) {
        Activity a;
        a.header = reader.header("");
        const std::uint8_t active = reader.u8("active");
        if (active > 1) throw Invalid("Activity: active " + str(std::uint64_t{active}) + " is not a bool");
        a.active = active == 1;
        a.sensors = reader.u8("sensors");
        a.largest_sensor = reader.u8("largest_sensor");
        a.n_regions = reader.u32("n_regions");
        a.voxel_count = reader.u32("voxel_count");
        const std::pair<std::optional<std::array<float, 3>>*, const char*> geometry[] = {
            {&a.centroid, "centroid"}, {&a.aabb_min, "aabb"}, {&a.aabb_max, "aabb"}};
        for (const auto& [field, what] : geometry) {
            const std::array<float, 3> values = reader.floats(what);
            const int nans = std::isnan(values[0]) + std::isnan(values[1]) + std::isnan(values[2]);
            if (nans != 0 && nans != 3) throw Invalid(std::string("Activity: ") + what + " is partly NaN");
            if (nans == 0) *field = values;
        }
        a.since_s = maybe(reader.f32("since_s"));
        a.validate();
        return a;
    });
}

void RequestSync::validate() const {
    const std::string context = "RequestSync";
    header.validate(context);
    require_enum(topic, 2, context + ": topic");
    if (expected_generation.has_value() != token.has_value()) {
        throw Invalid(context + ": expected_generation and request_token are both set (v2) or both absent (legacy)");
    }
    if (token && *token == kZeroToken) throw Invalid(context + ": an all-zero request_token is invalid in v2");
    if (since && (since->epoch == kSinceNone || since->seq == kSinceNone)) {
        throw Invalid(context + ": since uses SINCE_NONE; use an empty since");
    }
    const std::uint64_t since_epoch = since ? since->epoch : kSinceNone;
    if (header.map_epoch != since_epoch) {
        throw Invalid(context + ": map_epoch " + str(header.map_epoch) + " must repeat since_epoch " + str(since_epoch));
    }
}

Bytes RequestSync::to_cdr() const {
    validate();
    Writer writer;
    writer.header(header);
    writer.u64(since ? since->epoch : kSinceNone);
    writer.u64(since ? since->seq : kSinceNone);
    writer.u8(static_cast<std::uint8_t>(topic));
    if (token) {
        writer.u64(*expected_generation);
        writer.token(*token);
    }
    return writer.take();
}

RequestSync RequestSync::from_cdr(std::span<const std::uint8_t> data) {
    return decode<RequestSync>(data, "RequestSync", [](Reader& reader) {
        RequestSync r;
        r.header = reader.header("");
        const std::uint64_t since_epoch = reader.u64("since_epoch");
        const std::uint64_t since_seq = reader.u64("since_seq");
        r.topic = read_enum<SyncTopic>(reader.u8("topic"), 2, "RequestSync: topic");
        if ((since_epoch == kSinceNone) != (since_seq == kSinceNone)) {
            throw Invalid("RequestSync: since_epoch and since_seq must both be SINCE_NONE or neither");
        }
        if (since_epoch != kSinceNone) r.since = Since{since_epoch, since_seq};
        if (reader.is_v2(8 + kTokenBytes, "topic")) {
            r.expected_generation = reader.u64("expected_generation");
            r.token = reader.token("request_token");
        }
        r.validate();
        return r;
    });
}

namespace {

std::string range_text(const std::string& what, std::size_t index) { return what + "[" + str(std::uint64_t{index}) + "]"; }

// Canonical: first <= last, strictly increasing and non-adjacent, every range within the bound.
void require_ranges(const std::vector<SeqRange>& values, const std::optional<std::uint64_t>& bound, const std::string& what) {
    for (std::size_t n = 0; n < values.size(); ++n) {
        const SeqRange& range = values[n];
        if (range.last < range.first) {
            throw Invalid(range_text(what, n) + ".last (first <= last) " + str(range.last) + " is outside " +
                          str(range.first) + "..." + str(kUint64Max));
        }
        // The previous range passed these checks, so its last is below this first's and + 1 fits.
        if (n && range.first <= values[n - 1].last + 1) {
            throw Invalid(range_text(what, n) + " is not after " + range_text(what, n - 1) + " with a gap (canonical ranges)");
        }
        if (!bound || range.last > *bound) {
            throw Invalid(range_text(what, n) + " ends at " + str(range.last) + ", past the bound " +
                          (bound ? str(*bound) : std::string("SEQ_NONE")));
        }
    }
}

bool overlaps(const std::vector<SeqRange>& a, const std::vector<SeqRange>& b) {
    for (const SeqRange& x : a) {
        for (const SeqRange& y : b) {
            if (x.first <= y.last && y.first <= x.last) return true;
        }
    }
    return false;
}

}  // namespace

void SyncEnd::validate() const {
    const std::string context = "SyncEnd";
    header.validate(context);
    require_generation(publisher_generation, context + ": publisher_generation");
    require_enum(topic, 2, context + ": topic");
    if (bound == kSeqNone) throw Invalid(context + ": bound (2^64 - 1 is SEQ_NONE; use nullopt)");
    require_ranges(delivered, bound, context + ": delivered");
    require_ranges(unavailable, bound, context + ": unavailable");
    if (overlaps(delivered, unavailable)) throw Invalid(context + ": delivered and unavailable overlap");
    if (topic == SyncTopic::map_bricks && !unavailable.empty()) {
        throw Invalid(context + ": a map answer has no unavailable ranges");
    }
}

Bytes SyncEnd::to_cdr() const {
    validate();
    Writer writer;
    writer.header(header);
    writer.u64(publisher_generation);
    writer.token(token);
    writer.u8(static_cast<std::uint8_t>(topic));
    writer.u64(bound.value_or(kSeqNone));
    writer.ranges(delivered);
    writer.ranges(unavailable);
    return writer.take();
}

SyncEnd SyncEnd::from_cdr(std::span<const std::uint8_t> data) {
    return decode<SyncEnd>(data, "SyncEnd", [](Reader& reader) {
        SyncEnd e;
        e.header = reader.header("");
        e.publisher_generation = reader.u64("publisher_generation");
        e.token = reader.token("request_token");
        e.topic = read_enum<SyncTopic>(reader.u8("topic"), 2, "SyncEnd: topic");
        const std::uint64_t bound = reader.u64("bound");
        if (bound != kSeqNone) e.bound = bound;
        e.delivered = reader.ranges("delivered");
        e.unavailable = reader.ranges("unavailable");
        e.validate();
        return e;
    });
}

}  // namespace room_memory::msg
