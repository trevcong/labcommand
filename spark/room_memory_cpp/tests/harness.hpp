// Minimal test registry: TEST(name) { CHECK(...); }, one binary per file, nonzero exit on any
// failure. Pass a substring to run only matching tests.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace harness {

struct Failure : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct Case {
    const char* name;
    std::function<void()> body;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> body) { registry().push_back({name, std::move(body)}); }
};

inline std::string location(const char* file, int line) {
    return std::filesystem::path(file).filename().string() + ":" + std::to_string(line);
}

// Skips the rest of a test (missing optional fixtures); reported, not failed.
struct Skip : std::runtime_error {
    using std::runtime_error::runtime_error;
};

inline std::string read_text(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw Failure("cannot read " + path.string());
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

inline std::vector<std::uint8_t> read_bytes(const std::filesystem::path& path) {
    const std::string text = read_text(path);
    return std::vector<std::uint8_t>(text.begin(), text.end());
}

// Fixture roots from the build (compile definitions), overridable like the Python suite.
inline std::filesystem::path env_or(const char* variable, const char* fallback) {
    const char* value = std::getenv(variable);
    return value && *value ? value : fallback;
}

inline int run_all(int argc, char** argv) {
    const std::string filter = argc > 1 ? argv[1] : "";
    int failed = 0, passed = 0, skipped = 0;
    for (const Case& c : registry()) {
        if (!filter.empty() && std::string(c.name).find(filter) == std::string::npos) continue;
        try {
            c.body();
            ++passed;
        } catch (const Skip& skip) {
            ++skipped;
            std::printf("SKIP %s: %s\n", c.name, skip.what());
        } catch (const std::exception& error) {
            ++failed;
            std::printf("FAIL %s: %s\n", c.name, error.what());
        }
    }
    std::printf("%d passed, %d failed, %d skipped\n", passed, failed, skipped);
    return failed ? 1 : 0;
}

}  // namespace harness

#define HARNESS_CAT2(a, b) a##b
#define HARNESS_CAT(a, b) HARNESS_CAT2(a, b)
#define TEST(name)                                                                        \
    static void name();                                                                   \
    static const harness::Registrar HARNESS_CAT(registrar_, name)(#name, name);           \
    static void name()

#define CHECK(condition)                                                                                 \
    do {                                                                                                 \
        if (!(condition)) throw harness::Failure(harness::location(__FILE__, __LINE__) + ": " #condition); \
    } while (0)

#define CHECK_EQ(actual, expected)                                                                           \
    do {                                                                                                     \
        const auto actual_value = (actual);                                                                  \
        const auto expected_value = (expected);                                                              \
        if (!(actual_value == expected_value)) {                                                             \
            std::ostringstream text;                                                                         \
            text << harness::location(__FILE__, __LINE__) << ": " #actual " == " #expected " (" << actual_value \
                 << " vs " << expected_value << ")";                                                         \
            throw harness::Failure(text.str());                                                              \
        }                                                                                                    \
    } while (0)

#define CHECK_THROWS(Type, expression)                                                                         \
    do {                                                                                                       \
        bool thrown = false;                                                                                   \
        try {                                                                                                  \
            (void)(expression);                                                                                \
        } catch (const Type&) {                                                                                \
            thrown = true;                                                                                     \
        }                                                                                                      \
        if (!thrown) throw harness::Failure(harness::location(__FILE__, __LINE__) + ": no " #Type " from " #expression); \
    } while (0)

#define TEST_MAIN \
    int main(int argc, char** argv) { return harness::run_all(argc, argv); }
