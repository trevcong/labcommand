#include <iostream>
#include <string>
#include <vector>

#include "offline.hpp"

int main(int argc, char** argv) {
    std::ios::sync_with_stdio(false);
    const std::vector<std::string> args(argv + 1, argv + argc);
    return room_memory::offline::run(args, std::cout, std::cerr);
}
