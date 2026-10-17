# Fixture roots: the shared goldens live next to the Python oracle and in the repository's
# Tests/Fixtures; ROOM_MEMORY_CD_FIXTURES / ROOM_MEMORY_SNAPSHOTS / ROOM_MEMORY_PHASE0_FIXTURES
# override them at run time.
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ROOM_MEMORY_PYTHON_PACKAGE "${REPO_ROOT}/spark/room_memory" CACHE PATH "The Python oracle package (room_memory)")
set(ROOM_MEMORY_FIXTURES "${ROOM_MEMORY_PYTHON_PACKAGE}/tests/fixtures" CACHE PATH "Swift and hand goldens")
set(ROOM_MEMORY_CD_FIXTURES "${REPO_ROOT}/Tests/Fixtures/change-detection" CACHE PATH "change-detection goldens")
set(ROOM_MEMORY_SNAPSHOTS "${REPO_ROOT}/Tests/Fixtures/room/snapshots" CACHE PATH "Snapshot pairs (a/, b/) of the change-detection goldens")
if(DEFINED ENV{ROOM_MEMORY_PHASE0_FIXTURES})
  set(PHASE0_DEFAULT "$ENV{ROOM_MEMORY_PHASE0_FIXTURES}")
else()
  set(PHASE0_DEFAULT "${REPO_ROOT}/Tests/Fixtures/room/synthetic")
endif()
set(ROOM_MEMORY_PHASE0_FIXTURES "${PHASE0_DEFAULT}" CACHE PATH "Phase 0 synthetic fixtures")
# Present at configure time means required: a missing directory then fails instead of skipping.
if(IS_DIRECTORY "${ROOM_MEMORY_PHASE0_FIXTURES}")
  set(REQUIRE_PHASE0 1)
else()
  set(REQUIRE_PHASE0 0)
endif()

add_library(room_memory_test_support STATIC scenes.cpp)
target_link_libraries(room_memory_test_support PUBLIC room_memory_offline_cli)
target_include_directories(room_memory_test_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(room_memory_test_support PUBLIC
  ROOM_MEMORY_FIXTURES="${ROOM_MEMORY_FIXTURES}"
  ROOM_MEMORY_SWIFT_HAND="${REPO_ROOT}/Tests/Fixtures/room/golden/hand"
  ROOM_MEMORY_CD_FIXTURES="${ROOM_MEMORY_CD_FIXTURES}"
  ROOM_MEMORY_SNAPSHOTS="${ROOM_MEMORY_SNAPSHOTS}"
  ROOM_MEMORY_PHASE0_FIXTURES="${ROOM_MEMORY_PHASE0_FIXTURES}"
  ROOM_MEMORY_REQUIRE_PHASE0=${REQUIRE_PHASE0})

cmake_host_system_information(RESULT HOST_CORES QUERY NUMBER_OF_LOGICAL_CORES)
set(THREAD_COUNTS 1 3 ${HOST_CORES})
list(REMOVE_DUPLICATES THREAD_COUNTS)

foreach(suite goldens core messages memory generation cli)
  add_executable(test_${suite} test_${suite}.cpp)
  target_link_libraries(test_${suite} PRIVATE room_memory_test_support)
endforeach()

# Parity and determinism suites run at every thread count; the rest once.
foreach(threads ${THREAD_COUNTS})
  foreach(suite goldens core memory)
    add_test(NAME ${suite}_omp${threads} COMMAND test_${suite})
    set_tests_properties(${suite}_omp${threads} PROPERTIES ENVIRONMENT "OMP_NUM_THREADS=${threads}")
  endforeach()
endforeach()
add_test(NAME messages COMMAND test_messages)
add_executable(test_rhythm test_rhythm.cpp)
target_link_libraries(test_rhythm PRIVATE room_memory_test_support)
add_test(NAME rhythm COMMAND test_rhythm)
# No ROS: the node names labcommand_msgs types, and every .msg in that package is generated.
add_executable(test_interface test_interface.cpp)
target_link_libraries(test_interface PRIVATE room_memory_test_support)
target_compile_definitions(test_interface PRIVATE
  ROOM_MEMORY_MSGS_DIR="${REPO_ROOT}/ros/labcommand_msgs"
  ROOM_MEMORY_NODE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../node/room_memory_node.cpp")
add_test(NAME interface COMMAND test_interface)
# The messages suite encodes real compressed LZFSE payloads with the same framework.
if(ROOM_MEMORY_APPLE_LZFSE)
  target_compile_definitions(test_messages PRIVATE ROOM_MEMORY_APPLE_LZFSE)
endif()
add_test(NAME cli COMMAND test_cli)
add_test(NAME generation COMMAND test_generation)

# The installed binary itself, byte for byte, on both change-detection fixtures.
foreach(sensor jt128 mid360)
  add_test(NAME offline_binary_${sensor}
           COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:room_memory_offline>
                   -DBEFORE=${ROOM_MEMORY_SNAPSHOTS}/a/${sensor}.bin -DAFTER=${ROOM_MEMORY_SNAPSHOTS}/b/${sensor}.bin
                   -DGOLDEN=${ROOM_MEMORY_CD_FIXTURES}/${sensor}.json
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/compare_cli.cmake)
endforeach()

# 30+ minutes of each looped static recording. The mcaps are not in git; the binary skips (exit 77)
# when one is absent. libzstd is only for this reader, not the core.
find_path(ROOM_MEMORY_ZSTD_INCLUDE zstd.h)
find_library(ROOM_MEMORY_ZSTD_LIBRARY zstd)
if(NOT ROOM_MEMORY_ZSTD_INCLUDE OR NOT ROOM_MEMORY_ZSTD_LIBRARY)
  execute_process(COMMAND brew --prefix zstd OUTPUT_VARIABLE ROOM_MEMORY_ZSTD_PREFIX
                  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(ROOM_MEMORY_ZSTD_PREFIX
     AND EXISTS "${ROOM_MEMORY_ZSTD_PREFIX}/include/zstd.h"
     AND EXISTS "${ROOM_MEMORY_ZSTD_PREFIX}/lib/libzstd.dylib")
    set(ROOM_MEMORY_ZSTD_INCLUDE "${ROOM_MEMORY_ZSTD_PREFIX}/include")
    set(ROOM_MEMORY_ZSTD_LIBRARY "${ROOM_MEMORY_ZSTD_PREFIX}/lib/libzstd.dylib")
  endif()
endif()
add_executable(test_static_longrun test_static_longrun.cpp)
target_link_libraries(test_static_longrun PRIVATE room_memory_core)
if(ROOM_MEMORY_OPENMP)
  target_link_libraries(test_static_longrun PRIVATE OpenMP::OpenMP_CXX)
endif()
target_compile_options(test_static_longrun PRIVATE -Wall -Wextra -Wpedantic)
if(ROOM_MEMORY_ZSTD_INCLUDE AND ROOM_MEMORY_ZSTD_LIBRARY)
  target_include_directories(test_static_longrun PRIVATE ${ROOM_MEMORY_ZSTD_INCLUDE})
  target_link_libraries(test_static_longrun PRIVATE ${ROOM_MEMORY_ZSTD_LIBRARY})
  target_compile_definitions(test_static_longrun PRIVATE ROOM_MEMORY_HAS_ZSTD=1)
endif()
# Both recordings, one at a time: a whole recording takes about 5 GB.
add_test(NAME static_longrun COMMAND test_static_longrun)
add_test(NAME static_longrun_b COMMAND test_static_longrun b)
set_tests_properties(static_longrun static_longrun_b PROPERTIES
  SKIP_RETURN_CODE 77
  TIMEOUT 300
  RESOURCE_LOCK static_recording_memory
  ENVIRONMENT "OMP_NUM_THREADS=${HOST_CORES}")

# Differential tests against the Python oracle (pytest), and the real static recording, when a
# Python with numpy and pytest is given. They are registered either way: without one, each reports
# itself skipped with the reason, so a run lists every test and never silently fewer.
set(DIFFERENTIAL_THREADS 1 ${HOST_CORES})
list(REMOVE_DUPLICATES DIFFERENTIAL_THREADS)
set(PYTHON_TESTS)
foreach(threads IN LISTS DIFFERENTIAL_THREADS)
  list(APPEND PYTHON_TESTS differential_omp${threads})
endforeach()
list(APPEND PYTHON_TESTS recording_tools static_recording)
# Full process startup, including the Activity publisher, when this build has the rclcpp node.
# rhythm_publish stays false: that is the path that still creates Activity. Registered before the
# Python block's return(), so a configure without ROOM_MEMORY_PYTHON still runs it.
if(ROOM_MEMORY_BUILD_NODE)
  add_executable(test_node_startup test_node_startup.cpp)
  target_compile_options(test_node_startup PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME node_startup COMMAND test_node_startup $<TARGET_FILE:room_memory_node>)
  set_tests_properties(node_startup PROPERTIES TIMEOUT 60)
endif()
if(NOT ROOM_MEMORY_PYTHON)
  foreach(name IN LISTS PYTHON_TESTS)
    add_test(NAME ${name} COMMAND ${CMAKE_COMMAND} -E echo
             "SKIPPED: ROOM_MEMORY_PYTHON is empty (configure with -DROOM_MEMORY_PYTHON=<a Python with numpy and pytest>)")
    set_tests_properties(${name} PROPERTIES SKIP_REGULAR_EXPRESSION "SKIPPED: ROOM_MEMORY_PYTHON is empty")
  endforeach()
  return()
endif()
foreach(threads IN LISTS DIFFERENTIAL_THREADS)
  add_test(NAME differential_omp${threads}
           COMMAND ${ROOM_MEMORY_PYTHON} -m pytest -q -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/differential)
  set_tests_properties(differential_omp${threads} PROPERTIES
    ENVIRONMENT "OMP_NUM_THREADS=${threads};ROOM_MEMORY_PYTHON_PACKAGE=${ROOM_MEMORY_PYTHON_PACKAGE};ROOM_MEMORY_OFFLINE=$<TARGET_FILE:room_memory_offline>;ROOM_MEMORY_REPLAY=$<TARGET_FILE:room_memory_replay>;ROOM_MEMORY_SNAPSHOTS=${ROOM_MEMORY_SNAPSHOTS};ROOM_MEMORY_PHASE0_FIXTURES=${ROOM_MEMORY_PHASE0_FIXTURES}"
    TIMEOUT 1800)
endforeach()
# The recording tools on synthetic input: no recording needed, so they pass or fail on every machine.
# The recording tests import the oracle's decoder, from wherever the oracle is (/oracle/room_memory
# in the image).
set(STATIC_RECORDING_TEST ${CMAKE_CURRENT_SOURCE_DIR}/recording/test_static_recording.py)
set(RECORDING_ENVIRONMENT "ROOM_MEMORY_BAG=$<TARGET_FILE:room_memory_bag>;ROOM_MEMORY_PYTHON_PACKAGE=${ROOM_MEMORY_PYTHON_PACKAGE};PYTHONPATH=${ROOM_MEMORY_PYTHON_PACKAGE}")
add_test(NAME recording_tools
         COMMAND ${ROOM_MEMORY_PYTHON} -m pytest -q -rs -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/recording/test_tools.py)
# They also check that static_recording, run as below without a recording, reports skipped.
set_tests_properties(recording_tools PROPERTIES
  ENVIRONMENT "${RECORDING_ENVIRONMENT};ROOM_MEMORY_STATIC_RECORDING_TEST=${STATIC_RECORDING_TEST}"
  TIMEOUT 600)
# The real static recording (not in git; skipped with the reason printed when absent). Alone in its
# test, so a machine without the recording reports it skipped: recording/conftest.py exits 77 when
# every test skipped.
add_test(NAME static_recording
         COMMAND ${ROOM_MEMORY_PYTHON} -m pytest -q -rs -p no:cacheprovider ${STATIC_RECORDING_TEST})
set_tests_properties(static_recording PROPERTIES
  ENVIRONMENT "${RECORDING_ENVIRONMENT}"
  SKIP_RETURN_CODE 77
  TIMEOUT 3600)
