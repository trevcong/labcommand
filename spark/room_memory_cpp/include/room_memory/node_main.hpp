// The room-memory node's main, without rclcpp, so a ctest drives the exit paths the node
// binary takes (#94, #108). No ROS dependency: the node passes rclcpp's calls in.
#pragma once

#include <exception>
#include <string>

#include "room_memory/generation.hpp"

namespace room_memory {

inline constexpr const char* kNodeStoppedPrefix = "room-memory stopped: ";

// make() builds the node (a smart pointer), spin(node) runs it, shutdown() ends rclcpp and
// fatal(line) logs one FATAL line. Returns main's exit code: 0 after spin returns, 1 when
// construction or spin throws. A GenerationError is logged fatal by the constructor; any other
// std::exception is logged here, and the node (with its generation lock) goes before shutdown.
template <class Make, class Spin, class Shutdown, class Fatal>
int run_node(Make make, Spin spin, Shutdown shutdown, Fatal fatal) {
    decltype(make()) node;
    try {
        node = make();
        spin(node);
    } catch (const GenerationError&) {
        // Logged fatal by the constructor: no generation, no publishing.
        shutdown();
        return 1;
    } catch (const std::exception& error) {
        fatal(std::string(kNodeStoppedPrefix) + error.what());
        node.reset();
        shutdown();
        return 1;
    }
    shutdown();
    return 0;
}

}  // namespace room_memory
