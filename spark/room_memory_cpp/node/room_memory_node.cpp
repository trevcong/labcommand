// Thin rclcpp node around RoomMemory: raw CDR in, codec bytes out. The C++ twin of
// spark/room_memory/room_memory/node.py, with the same topics, QoS and parameters: status at
// 1 Hz, events per transition, map_bricks after each window when RoomMemory::map_update has
// one, and RequestSync answers on the two sync topics (protocol sections 2 and 5).
//
// Subscriptions and publishers are generic (serialized), so frames go through the tested
// decode_pointcloud2 and outputs are the codec's CDR bytes; sensor_msgs and labcommand_msgs
// only provide the type support at run time.
#include <time.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include "room_memory/memory.hpp"

namespace {

using room_memory::MemoryConfig;
using room_memory::RoomMemory;
namespace msg = room_memory::msg;

constexpr const char* kStatusTopic = "/room_memory/status";
constexpr const char* kEventsTopic = "/room_memory/events";
constexpr const char* kMapTopic = "/room_memory/map_bricks";
constexpr const char* kRequestTopic = "/room_memory/request_sync";
constexpr const char* kSyncMapTopic = "/room_memory/sync/map_bricks";
constexpr const char* kSyncEventsTopic = "/room_memory/sync/events";
constexpr const char* kMapType = "labcommand_msgs/msg/MapBricks";
constexpr const char* kEventType = "labcommand_msgs/msg/ChangeEvent";

double cpu_ms() {
    timespec t{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
    return static_cast<double>(t.tv_sec) * 1e3 + static_cast<double>(t.tv_nsec) / 1e6;
}

rclcpp::SerializedMessage serialized(const msg::Bytes& bytes) {
    rclcpp::SerializedMessage out(bytes.size());
    rcl_serialized_message_t& raw = out.get_rcl_serialized_message();
    std::memcpy(raw.buffer, bytes.data(), bytes.size());
    raw.buffer_length = bytes.size();
    return out;
}

std::string names(const std::vector<std::string>& sensors) {
    std::string text = "[";
    for (std::size_t n = 0; n < sensors.size(); ++n) text += (n ? ", '" : "'") + sensors[n] + "'";
    return text + "]";
}

class RoomMemoryNode : public rclcpp::Node {
public:
    RoomMemoryNode() : rclcpp::Node("room_memory") {
        const MemoryConfig defaults;
        MemoryConfig config;
        config.room_id = declare_parameter<std::string>("room_id", defaults.room_id);
        config.sensors = declare_parameter<std::vector<std::string>>("sensors", defaults.sensors);
        config.voxel_size = declare_parameter<double>("voxel_size", defaults.voxel_size);
        const double max_range = declare_parameter<double>("max_range", *defaults.max_range);
        config.max_range = max_range > 0 ? std::optional<double>(max_range) : std::nullopt;
        config.tolerance = declare_parameter<std::int64_t>("tolerance", defaults.tolerance);
        config.min_voxels = declare_parameter<std::int64_t>("min_voxels", defaults.min_voxels);
        config.min_extent = declare_parameter<std::int64_t>("min_extent", defaults.min_extent);
        config.window_s = declare_parameter<double>("window_s", defaults.window_s);
        config.reference_windows = declare_parameter<std::int64_t>("reference_windows", defaults.reference_windows);
        config.recent_windows = declare_parameter<std::int64_t>("recent_windows", defaults.recent_windows);
        const std::string state_dir = declare_parameter<std::string>("state_dir", *defaults.state_dir);
        config.state_dir = state_dir.empty() ? std::nullopt : std::optional<std::string>(state_dir);
        config.occupied_min_hits = declare_parameter<std::int64_t>("occupied_min_hits", defaults.occupied_min_hits);
        config.occupied_min_ratio = declare_parameter<double>("occupied_min_ratio", defaults.occupied_min_ratio);
        config.free_min_seen = declare_parameter<std::int64_t>("free_min_seen", defaults.free_min_seen);
        config.free_max_ratio = declare_parameter<double>("free_max_ratio", defaults.free_max_ratio);
        config.candidate_windows = declare_parameter<std::int64_t>("candidate_windows", defaults.candidate_windows);
        config.expire_windows = declare_parameter<std::int64_t>("expire_windows", defaults.expire_windows);
        config.learn_max_offset_m = declare_parameter<double>("learn_max_offset_m", defaults.learn_max_offset_m);
        config.map_sensor = declare_parameter<std::string>("map_sensor", defaults.map_sensor);
        config.delta_period_s = declare_parameter<double>("delta_period_s", defaults.delta_period_s);
        config.keyframe_period_s = declare_parameter<double>("keyframe_period_s", defaults.keyframe_period_s);
        config.keyframe_delta_ratio = declare_parameter<double>("keyframe_delta_ratio", defaults.keyframe_delta_ratio);
        memory_ = std::make_unique<RoomMemory>(config, stamp());

        events_ = create_generic_publisher(kEventsTopic, kEventType, rclcpp::QoS(100).reliable());
        status_ = create_generic_publisher(kStatusTopic, "labcommand_msgs/msg/MemoryStatus", rclcpp::QoS(1).reliable());
        map_ = create_generic_publisher(kMapTopic, kMapType, rclcpp::QoS(10).reliable());
        sync_map_ = create_generic_publisher(kSyncMapTopic, kMapType, rclcpp::QoS(10).reliable());
        sync_events_ = create_generic_publisher(kSyncEventsTopic, kEventType, rclcpp::QoS(200).reliable());
        requests_ = create_generic_subscription(
            kRequestTopic, "labcommand_msgs/msg/RequestSync", rclcpp::QoS(10).reliable(),
            [this](std::shared_ptr<const rclcpp::SerializedMessage> message) { answer(*message); });
        for (const std::string& name : memory_->config().sensors) {
            const room_memory::Sensor& sensor = memory_->channel(name).sensor;
            subscriptions_.push_back(create_generic_subscription(
                sensor.topic, "sensor_msgs/msg/PointCloud2", rclcpp::SensorDataQoS(),
                [this, name](std::shared_ptr<const rclcpp::SerializedMessage> message) {
                    const rcl_serialized_message_t& raw = message->get_rcl_serialized_message();
                    memory_->add_cdr(name, std::span<const std::uint8_t>(raw.buffer, raw.buffer_length));
                }));
        }
        last_status_ = std::chrono::steady_clock::now();
        window_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(config.window_s),
                                             [this] { close_window(); });
        status_timer_ =
            rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0), [this] { publish_status(); });
        RCLCPP_INFO(get_logger(), "room '%s' sensors %s state %s map_epoch %llu", config.room_id.c_str(),
                    names(memory_->config().sensors).c_str(), msg::name(memory_->state()),
                    static_cast<unsigned long long>(memory_->map_epoch()));
    }

private:
    msg::Time stamp() {
        const std::int64_t ns = now().nanoseconds();
        const std::int64_t sec = ns >= 0 ? ns / 1'000'000'000 : -((-ns + 999'999'999) / 1'000'000'000);
        return msg::Time{sec, ns - sec * 1'000'000'000};
    }

    void close_window() {
        const auto start = std::chrono::steady_clock::now();
        const double cpu = cpu_ms();
        const msg::Time now_stamp = stamp();
        const std::vector<msg::ChangeEvent> events = memory_->close_window(now_stamp);
        for (const msg::ChangeEvent& event : events) events_->publish(serialized(event.to_cdr()));
        const std::optional<msg::Bytes> map = memory_->map_update(now_stamp);
        if (map) map_->publish(serialized(*map));
        const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(get_logger(), "window %.1f ms wall, %.1f ms cpu, %zu event messages, map %zu B, state %s", elapsed,
                    cpu_ms() - cpu, events.size(), map ? map->size() : std::size_t{0}, msg::name(memory_->state()));
        if (elapsed > memory_->config().window_s * 1000) {
            RCLCPP_WARN(get_logger(), "window took %.0f ms, longer than window_s", elapsed);
        }
    }

    void publish_status() {
        const auto now_steady = std::chrono::steady_clock::now();
        const double period = std::chrono::duration<double>(now_steady - last_status_).count();
        last_status_ = now_steady;
        status_->publish(serialized(memory_->status(stamp(), period).to_cdr()));
    }

    void answer(const rclcpp::SerializedMessage& message) {
        const rcl_serialized_message_t& raw = message.get_rcl_serialized_message();
        msg::RequestSync request;
        try {
            request = msg::RequestSync::from_cdr(std::span<const std::uint8_t>(raw.buffer, raw.buffer_length));
        } catch (const msg::CDRError& error) {
            RCLCPP_WARN(get_logger(), "RequestSync dropped: %s", error.what());
            return;
        }
        const std::vector<msg::Bytes> reply = memory_->sync(request, stamp());
        const auto& publisher = request.topic == msg::SyncTopic::map_bricks ? sync_map_ : sync_events_;
        for (const msg::Bytes& bytes : reply) publisher->publish(serialized(bytes));
        RCLCPP_INFO(get_logger(), "RequestSync %s from room '%s': %zu messages",
                    request.topic == msg::SyncTopic::map_bricks ? "map_bricks" : "events", request.header.room_id.c_str(),
                    reply.size());
    }

    std::unique_ptr<RoomMemory> memory_;
    std::shared_ptr<rclcpp::GenericPublisher> events_;
    std::shared_ptr<rclcpp::GenericPublisher> status_;
    std::shared_ptr<rclcpp::GenericPublisher> map_;
    std::shared_ptr<rclcpp::GenericPublisher> sync_map_;
    std::shared_ptr<rclcpp::GenericPublisher> sync_events_;
    std::shared_ptr<rclcpp::GenericSubscription> requests_;
    std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
    rclcpp::TimerBase::SharedPtr window_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::chrono::steady_clock::time_point last_status_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoomMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
