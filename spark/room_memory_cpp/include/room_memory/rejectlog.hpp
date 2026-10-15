// Throttled warning text for rejected lidar frames: the C++ twin of
// room_memory/rejectlog.py. No ROS dependency.
//
// One line per sensor per kRejectionIntervalS seconds. The caller logs it. The helper sees
// the error's type name and message, never the frame.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>

namespace room_memory {

inline constexpr double kRejectionIntervalS = 10.0;

// One warning line per sensor per interval, with a suppressed count. `clock` returns seconds;
// the default is std::chrono::steady_clock, so a wall-clock step does not open or close a
// window.
class RejectionLog {
public:
    explicit RejectionLog(double interval_s = kRejectionIntervalS, std::function<double()> clock = {});

    // The warning line, or empty when this rejection is suppressed.
    std::optional<std::string> observe(const std::string& sensor, const std::string& type_name,
                                       const std::string& message);

private:
    double interval_s_;
    std::function<double()> clock_;
    std::map<std::string, double> last_emit_;
    std::map<std::string, std::int64_t> suppressed_;
};

}  // namespace room_memory
