// Thin rclcpp node around RoomMemory: raw CDR in, codec bytes out. The C++ twin of
// spark/room_memory/room_memory/node.py, with the same topics, QoS and parameters: status and
// activity at 1 Hz, events per transition, map_bricks after each window when RoomMemory::map_update has
// one, and RequestSync answers on the two sync topics, each answer followed by its SyncEnd on
// /room_memory/sync/end (protocol sections 2 and 5). A RequestSync arrives on its topic or as the
// /room_memory/sync/request service, which the :8767 bridge exposes to the Mac (D39). The publisher generation is allocated
// (durably, under publisher.lock) before any publisher exists; without it the node refuses to
// start. A rejected frame logs one warning per sensor per 10 s (RejectionLog), with the
// decoder's error.
//
// Subscriptions and publishers are generic (serialized), so frames go through the tested
// decode_pointcloud2 and outputs are the codec's CDR bytes; sensor_msgs and labcommand_msgs
// only provide the type support at run time. The service is typed (Jazzy has no generic
// service): its request is re-serialized to the RequestSync bytes and answered the same way.
#include <time.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <labcommand_msgs/msg/request_sync.hpp>
#include <labcommand_msgs/srv/request_sync.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "room_memory/generation.hpp"
#include "room_memory/memory.hpp"
#include "room_memory/rejectlog.hpp"

namespace {

using room_memory::MemoryConfig;
using room_memory::RoomMemory;
namespace msg = room_memory::msg;

constexpr const char* kStatusTopic = "/room_memory/status";
constexpr const char* kActivityTopic = "/room_memory/activity";
constexpr const char* kEventsTopic = "/room_memory/events";
constexpr const char* kMapTopic = "/room_memory/map_bricks";
constexpr const char* kRequestTopic = "/room_memory/request_sync";
constexpr const char* kRequestService = "/room_memory/sync/request";
constexpr const char* kSyncMapTopic = "/room_memory/sync/map_bricks";
constexpr const char* kSyncEventsTopic = "/room_memory/sync/events";
constexpr const char* kSyncEndTopic = "/room_memory/sync/end";
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

std::string range_list(const std::vector<msg::SeqRange>& values) {
    std::string text = "[";
    for (std::size_t n = 0; n < values.size(); ++n) {
        text += (n ? ", (" : "(") + std::to_string(values[n].first) + ", " + std::to_string(values[n].last) + ")";
    }
    return text + "]";
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
        config.fresh_min_voxels = declare_parameter<std::int64_t>("fresh_min_voxels", defaults.fresh_min_voxels);
        config.fresh_min_fraction = declare_parameter<double>("fresh_min_fraction", defaults.fresh_min_fraction);
        config.stable_min_fraction = declare_parameter<double>("stable_min_fraction", defaults.stable_min_fraction);
        config.activity_radius_m = declare_parameter<double>("activity_radius_m", defaults.activity_radius_m);
        config.learn_max_offset_m = declare_parameter<double>("learn_max_offset_m", defaults.learn_max_offset_m);
        config.learn_windows = declare_parameter<std::int64_t>("learn_windows", defaults.learn_windows);
        config.map_sensor = declare_parameter<std::string>("map_sensor", defaults.map_sensor);
        config.delta_period_s = declare_parameter<double>("delta_period_s", defaults.delta_period_s);
        config.keyframe_period_s = declare_parameter<double>("keyframe_period_s", defaults.keyframe_period_s);
        config.keyframe_delta_ratio = declare_parameter<double>("keyframe_delta_ratio", defaults.keyframe_delta_ratio);
        config.rhythm_enable = declare_parameter<bool>("rhythm_enable", defaults.rhythm_enable);
        config.rhythm_cell_m = declare_parameter<double>("rhythm_cell_m", defaults.rhythm_cell_m);
        config.rhythm_dims = declare_parameter<std::int64_t>("rhythm_dims", defaults.rhythm_dims);
        config.rhythm_period_s = declare_parameter<std::int64_t>("rhythm_period_s", defaults.rhythm_period_s);
        config.rhythm_publish = declare_parameter<bool>("rhythm_publish", defaults.rhythm_publish);
        config.rhythm_dir = declare_parameter<std::string>("rhythm_dir", defaults.rhythm_dir);
        config.rhythm_keep_days = declare_parameter<std::int64_t>("rhythm_keep_days", defaults.rhythm_keep_days);
        config.rhythm_max_bytes = declare_parameter<std::int64_t>("rhythm_max_bytes", defaults.rhythm_max_bytes);
        config.rhythm_max_file_bytes = declare_parameter<std::int64_t>("rhythm_max_file_bytes", defaults.rhythm_max_file_bytes);
        config.rhythm_fsync_s = declare_parameter<std::int64_t>("rhythm_fsync_s", defaults.rhythm_fsync_s);
        // Before any publisher: every message this process sends carries the generation. The
        // allocation holds publisher.lock until the process exits. The wall clock, not now(),
        // which may be sim time.
        const std::int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
        try {
            generation_ = std::make_unique<room_memory::Allocation>(
                room_memory::allocate_generation(config.state_dir, config.room_id, now_ms));
        } catch (const room_memory::GenerationError& error) {
            RCLCPP_FATAL(get_logger(), "refusing to start: %s", error.what());
            throw;
        }
        memory_ = std::make_unique<RoomMemory>(config, stamp(), generation_->generation());

        // Durability is declared on every topic (protocol §2, D24). Status and activity are full state at
        // 1 Hz, so a late subscriber (the bridge, when the first Mac subscribes) gets the last one at once;
        // a copy delivered again repeats its seq in its generation. Everything else is volatile: the Mac
        // gets the map and recent events by RequestSync, and a restarted node never receives an old
        // request. Rhythm, when published, is the same latched full-state QoS as status. Same QoS as node.py.
        const auto volatile_qos = [](std::size_t depth) { return rclcpp::QoS(depth).reliable().durability_volatile(); };
        const auto latched_qos = [] { return rclcpp::QoS(1).reliable().transient_local(); };
        events_ = create_generic_publisher(kEventsTopic, kEventType, volatile_qos(100));
        status_ = create_generic_publisher(kStatusTopic, "labcommand_msgs/msg/MemoryStatus", latched_qos());
        activity_ = create_generic_publisher(kActivityTopic, "labcommand_msgs/msg/Activity", latched_qos());
        map_ = create_generic_publisher(kMapTopic, kMapType, volatile_qos(10));
        sync_map_ = create_generic_publisher(kSyncMapTopic, kMapType, volatile_qos(10));
        sync_events_ = create_generic_publisher(kSyncEventsTopic, kEventType, volatile_qos(200));
        sync_end_ = create_generic_publisher(kSyncEndTopic, "labcommand_msgs/msg/SyncEnd", volatile_qos(10));
        if (config.rhythm_publish) {
            rhythm_pub_ = create_generic_publisher("/room_memory/rhythm", "labcommand_msgs/msg/RhythmMinute", latched_qos());
            auto publisher = rhythm_pub_;
            memory_->set_rhythm_publish([publisher](std::span<const std::uint8_t> bytes) {
                publisher->publish(serialized(msg::Bytes(bytes.begin(), bytes.end())));
            });
        }
        requests_ = create_generic_subscription(
            kRequestTopic, "labcommand_msgs/msg/RequestSync", volatile_qos(10),
            [this](std::shared_ptr<const rclcpp::SerializedMessage> message) { answer(*message); });
        request_service_ = create_service<labcommand_msgs::srv::RequestSync>(
            kRequestService, [this](const std::shared_ptr<labcommand_msgs::srv::RequestSync::Request> request,
                                    std::shared_ptr<labcommand_msgs::srv::RequestSync::Response> response) {
                rclcpp::SerializedMessage message;
                request_serialization_.serialize_message(&request->request, &message);
                response->accepted = answer(message);
            });
        for (const std::string& name : memory_->config().sensors) {
            const room_memory::Sensor& sensor = memory_->channel(name).sensor;
            subscriptions_.push_back(create_generic_subscription(
                sensor.topic, "sensor_msgs/msg/PointCloud2", rclcpp::SensorDataQoS(),
                [this, name](std::shared_ptr<const rclcpp::SerializedMessage> message) { on_cloud(name, *message); }));
        }
        last_status_ = std::chrono::steady_clock::now();
        window_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(config.window_s),
                                             [this] { close_window(); });
        status_timer_ =
            rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0), [this] { publish_status(); });
        RCLCPP_INFO(get_logger(), "room '%s' sensors %s state %s map_epoch %llu publisher_generation %llu",
                    config.room_id.c_str(), names(memory_->config().sensors).c_str(), msg::name(memory_->state()),
                    static_cast<unsigned long long>(memory_->map_epoch()),
                    static_cast<unsigned long long>(memory_->publisher_generation()));
    }

    // Join the writer while the rhythm publisher is still alive.
    ~RoomMemoryNode() override {
        if (memory_) memory_->close();
    }

private:
    msg::Time stamp() {
        const std::int64_t ns = now().nanoseconds();
        const std::int64_t sec = ns >= 0 ? ns / 1'000'000'000 : -((-ns + 999'999'999) / 1'000'000'000);
        return msg::Time{sec, ns - sec * 1'000'000'000};
    }

    void on_cloud(const std::string& name, const rclcpp::SerializedMessage& message) {
        const rcl_serialized_message_t& raw = message.get_rcl_serialized_message();
        if (memory_->add_cdr(name, std::span<const std::uint8_t>(raw.buffer, raw.buffer_length))) return;
        const room_memory::RejectedFrame& error = *memory_->channel(name).last_error;
        if (const auto line = rejections_.observe(name, error.type_name, error.message)) {
            RCLCPP_WARN(get_logger(), "%s", line->c_str());
        }
    }

    void close_window() {
        const auto start = std::chrono::steady_clock::now();
        const double cpu = cpu_ms();
        const msg::Time now_stamp = stamp();
        const std::vector<msg::ChangeEvent> events = memory_->close_window(now_stamp);
        for (const msg::ChangeEvent& event : events) events_->publish(serialized(event.to_cdr()));
        const std::optional<msg::Bytes> map = memory_->map_update(now_stamp);
        if (map) map_->publish(serialized(*map));
        memory_->enqueue_rhythm();
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
        const msg::Time now_stamp = stamp();
        status_->publish(serialized(memory_->status(now_stamp, period).to_cdr()));
        activity_->publish(serialized(memory_->activity(now_stamp).to_cdr()));
    }

    /// Publishes the answer and its SyncEnd; false when the request was dropped.
    bool answer(const rclcpp::SerializedMessage& message) {
        const rcl_serialized_message_t& raw = message.get_rcl_serialized_message();
        msg::RequestSync request;
        try {
            request = msg::RequestSync::from_cdr(std::span<const std::uint8_t>(raw.buffer, raw.buffer_length));
        } catch (const msg::CDRError& error) {
            RCLCPP_WARN(get_logger(), "RequestSync dropped: %s", error.what());
            return false;
        }
        const RoomMemory::SyncAnswer reply = memory_->sync(request, stamp());
        if (!reply.end) {
            RCLCPP_WARN(get_logger(), "RequestSync for room '%s' dropped: not this room", request.header.room_id.c_str());
            return false;
        }
        // The answer first, then its manifest (which may still arrive first: it has its own topic).
        const auto& publisher = request.topic == msg::SyncTopic::map_bricks ? sync_map_ : sync_events_;
        for (const msg::Bytes& bytes : reply.messages) publisher->publish(serialized(bytes));
        sync_end_->publish(serialized(reply.end->to_cdr()));
        const std::string expected =
            request.expected_generation ? std::to_string(*request.expected_generation) : std::string("None");
        const std::string bound = reply.end->bound ? std::to_string(*reply.end->bound) : std::string("None");
        RCLCPP_INFO(get_logger(), "RequestSync %s expected_generation %s: %zu messages, bound %s, delivered %s, unavailable %s",
                    request.topic == msg::SyncTopic::map_bricks ? "MAP_BRICKS" : "EVENTS", expected.c_str(),
                    reply.messages.size(), bound.c_str(), range_list(reply.end->delivered).c_str(),
                    range_list(reply.end->unavailable).c_str());
        return true;
    }

    // First, so it is destroyed last: the lock outlives every publisher.
    std::unique_ptr<room_memory::Allocation> generation_;
    std::unique_ptr<RoomMemory> memory_;
    room_memory::RejectionLog rejections_;
    std::shared_ptr<rclcpp::GenericPublisher> events_;
    std::shared_ptr<rclcpp::GenericPublisher> status_;
    std::shared_ptr<rclcpp::GenericPublisher> activity_;
    std::shared_ptr<rclcpp::GenericPublisher> map_;
    std::shared_ptr<rclcpp::GenericPublisher> sync_map_;
    std::shared_ptr<rclcpp::GenericPublisher> sync_events_;
    std::shared_ptr<rclcpp::GenericPublisher> sync_end_;
    std::shared_ptr<rclcpp::GenericPublisher> rhythm_pub_;
    std::shared_ptr<rclcpp::GenericSubscription> requests_;
    rclcpp::Service<labcommand_msgs::srv::RequestSync>::SharedPtr request_service_;
    rclcpp::Serialization<labcommand_msgs::msg::RequestSync> request_serialization_;
    std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
    rclcpp::TimerBase::SharedPtr window_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::chrono::steady_clock::time_point last_status_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<RoomMemoryNode> node;
    try {
        node = std::make_shared<RoomMemoryNode>();
    } catch (const room_memory::GenerationError&) {
        // Logged fatal by the constructor: no generation, no publishing.
        rclcpp::shutdown();
        return 1;
    }
    // The startup test sets this and waits for the file. Absent in normal launches.
    // Activity is created even with rhythm_publish false, before this returns.
    if (const char* mark = std::getenv("ROOM_MEMORY_STARTUP_MARK")) {
        std::ofstream out(mark, std::ios::out | std::ios::trunc);
        if (!out) {
            RCLCPP_ERROR(node->get_logger(), "could not write startup mark %s", mark);
            rclcpp::shutdown();
            return 2;
        }
        out << "ready\n";
        out.close();
    }
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
