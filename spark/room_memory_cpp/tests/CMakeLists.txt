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

foreach(suite goldens core messages memory cli)
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
add_test(NAME cli COMMAND test_cli)

# The installed binary itself, byte for byte, on both change-detection fixtures.
foreach(sensor jt128 mid360)
  add_test(NAME offline_binary_${sensor}
           COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:room_memory_offline>
                   -DBEFORE=${ROOM_MEMORY_SNAPSHOTS}/a/${sensor}.bin -DAFTER=${ROOM_MEMORY_SNAPSHOTS}/b/${sensor}.bin
                   -DGOLDEN=${ROOM_MEMORY_CD_FIXTURES}/${sensor}.json
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/compare_cli.cmake)
endforeach()

# Differential tests against the Python oracle (pytest), when a Python with numpy is given.
if(ROOM_MEMORY_PYTHON)
  foreach(threads 1 ${HOST_CORES})
    add_test(NAME differential_omp${threads}
             COMMAND ${ROOM_MEMORY_PYTHON} -m pytest -q -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/differential)
    set_tests_properties(differential_omp${threads} PROPERTIES
      ENVIRONMENT "OMP_NUM_THREADS=${threads};ROOM_MEMORY_PYTHON_PACKAGE=${ROOM_MEMORY_PYTHON_PACKAGE};ROOM_MEMORY_OFFLINE=$<TARGET_FILE:room_memory_offline>;ROOM_MEMORY_REPLAY=$<TARGET_FILE:room_memory_replay>;ROOM_MEMORY_SNAPSHOTS=${ROOM_MEMORY_SNAPSHOTS};ROOM_MEMORY_PHASE0_FIXTURES=${ROOM_MEMORY_PHASE0_FIXTURES}"
      TIMEOUT 1800)
  endforeach()
  # The real static recording (not in git; skipped with the reason printed when absent). Its tools
  # import the oracle's decoder, from wherever the oracle is (/oracle/room_memory in the image).
  add_test(NAME static_recording
           COMMAND ${ROOM_MEMORY_PYTHON} -m pytest -q -rs -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/recording)
  set_tests_properties(static_recording PROPERTIES
    ENVIRONMENT "ROOM_MEMORY_BAG=$<TARGET_FILE:room_memory_bag>;ROOM_MEMORY_PYTHON_PACKAGE=${ROOM_MEMORY_PYTHON_PACKAGE};PYTHONPATH=${ROOM_MEMORY_PYTHON_PACKAGE}"
    TIMEOUT 3600)
endif()
