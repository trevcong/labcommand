#include "room_memory/rejectlog.hpp"

#include <chrono>
#include <utility>

namespace room_memory {

namespace {

double steady_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

RejectionLog::RejectionLog(double interval_s, std::function<double()> clock)
    : interval_s_(interval_s), clock_(clock ? std::move(clock) : std::function<double()>(steady_seconds)) {}

std::optional<std::string> RejectionLog::observe(const std::string& sensor, const std::string& type_name,
                                                 const std::string& message) {
    const double now = clock_();
    const auto last = last_emit_.find(sensor);
    if (last != last_emit_.end() && now - last->second < interval_s_) {
        ++suppressed_[sensor];
        return std::nullopt;
    }
    const std::int64_t count = std::exchange(suppressed_[sensor], 0);
    last_emit_[sensor] = now;
    std::string line = sensor + " rejected frame: " + type_name + ": " + message;
    if (count > 0) line += " (" + std::to_string(count) + " suppressed)";
    return line;
}

}  // namespace room_memory
