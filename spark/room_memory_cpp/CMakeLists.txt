# room_memory_cpp: production C++ core, offline CLI, tests and benchmark (plain CMake), plus
# the rclcpp node when ament_cmake and rclcpp are present (colcon in the ros:jazzy image).
cmake_minimum_required(VERSION 3.20)
project(room_memory_cpp VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
if(ament_cmake_FOUND AND rclcpp_FOUND)
  set(ROOM_MEMORY_NODE_DEFAULT ON)
else()
  set(ROOM_MEMORY_NODE_DEFAULT OFF)
endif()
option(ROOM_MEMORY_BUILD_NODE "Build the rclcpp node (needs ament_cmake and rclcpp)" ${ROOM_MEMORY_NODE_DEFAULT})
option(ROOM_MEMORY_OPENMP "Parallelize with OpenMP" ON)
option(ROOM_MEMORY_NATIVE "Tune for the build machine's CPU (-mcpu=native)" ON)
option(ROOM_MEMORY_SIMD "Use the NEON kernels on aarch64 (OFF: the portable lane code)" ON)
option(ROOM_MEMORY_APPLE_LZFSE "Decode compressed LZFSE blocks with Apple's Compression framework" ${APPLE})
# The differential tests run the Python oracle; the repository's venv is the default when present.
set(ROOM_MEMORY_VENV_PYTHON "${CMAKE_CURRENT_SOURCE_DIR}/../.venv-local/bin/python")
if(EXISTS "${ROOM_MEMORY_VENV_PYTHON}")
  set(ROOM_MEMORY_PYTHON_DEFAULT "${ROOM_MEMORY_VENV_PYTHON}")
else()
  set(ROOM_MEMORY_PYTHON_DEFAULT "")
endif()
set(ROOM_MEMORY_PYTHON "${ROOM_MEMORY_PYTHON_DEFAULT}" CACHE FILEPATH
    "Python with numpy and pytest for the differential tests (empty: skip them)")

find_package(ZLIB REQUIRED)

if(ROOM_MEMORY_OPENMP)
  if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    # Apple clang has no bundled OpenMP runtime; use Homebrew's libomp.
    execute_process(COMMAND brew --prefix libomp OUTPUT_VARIABLE LIBOMP_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
    if(LIBOMP_PREFIX)
      set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I${LIBOMP_PREFIX}/include")
      set(OpenMP_CXX_LIB_NAMES omp)
      set(OpenMP_omp_LIBRARY "${LIBOMP_PREFIX}/lib/libomp.dylib")
    endif()
  endif()
  find_package(OpenMP REQUIRED COMPONENTS CXX)
endif()

# Bit-exact binary32 arithmetic: never fuse a multiply and an add, never reassociate. PUBLIC,
# so every translation unit that includes core headers keeps the spec's rounding too.
set(ROOM_MEMORY_EXACT_FLAGS -ffp-contract=off -fno-fast-math)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  list(APPEND ROOM_MEMORY_EXACT_FLAGS -fno-unsafe-math-optimizations)
endif()

add_library(room_memory_core
  src/keys.cpp
  src/key_set.cpp
  src/occupancy.cpp
  src/rays.cpp
  src/diff.cpp
  src/json.cpp
  src/pointcloud2.cpp
  src/messages.cpp
  src/memory.cpp
  src/generation.cpp
  src/rejectlog.cpp
)
set_target_properties(room_memory_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(room_memory_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(room_memory_core PUBLIC ${ROOM_MEMORY_EXACT_FLAGS})
target_compile_options(room_memory_core PRIVATE -Wall -Wextra -Wpedantic)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(room_memory_core PRIVATE -O3)
endif()
if(ROOM_MEMORY_NATIVE)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    target_compile_options(room_memory_core PRIVATE -mcpu=native)
  else()
    target_compile_options(room_memory_core PRIVATE -march=native)
  endif()
endif()
target_link_libraries(room_memory_core PUBLIC ZLIB::ZLIB)
if(ROOM_MEMORY_OPENMP)
  target_link_libraries(room_memory_core PUBLIC OpenMP::OpenMP_CXX)
endif()
if(NOT ROOM_MEMORY_SIMD)
  target_compile_definitions(room_memory_core PRIVATE ROOM_MEMORY_NO_SIMD)
endif()
if(ROOM_MEMORY_APPLE_LZFSE)
  target_compile_definitions(room_memory_core PRIVATE ROOM_MEMORY_APPLE_LZFSE)
  target_link_libraries(room_memory_core PUBLIC compression)
endif()

add_library(room_memory_offline_cli STATIC tools/offline.cpp)
target_link_libraries(room_memory_offline_cli PUBLIC room_memory_core)
target_include_directories(room_memory_offline_cli PUBLIC tools)

add_executable(room_memory_offline tools/offline_main.cpp)
target_link_libraries(room_memory_offline PRIVATE room_memory_offline_cli)

add_executable(room_memory_replay tools/replay.cpp)
target_link_libraries(room_memory_replay PRIVATE room_memory_core)

add_executable(room_memory_bag tools/bag.cpp)
target_link_libraries(room_memory_bag PRIVATE room_memory_core)

add_executable(bench_memory bench/bench_memory.cpp)
target_link_libraries(bench_memory PRIVATE room_memory_core)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(ROOM_MEMORY_BUILD_NODE)
  add_executable(room_memory_node node/room_memory_node.cpp)
  target_link_libraries(room_memory_node PRIVATE room_memory_core rclcpp::rclcpp)
  install(TARGETS room_memory_node room_memory_offline room_memory_replay room_memory_bag DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
