# cmake -DCLI=... -DBEFORE=... -DAFTER=... -DGOLDEN=... -P compare_cli.cmake
# Runs the offline CLI with no flags (its defaults are --evidence --json) and requires its
# stdout to equal the golden file byte for byte.
execute_process(COMMAND "${CLI}" "${BEFORE}" "${AFTER}" OUTPUT_VARIABLE out RESULT_VARIABLE code)
if(NOT code EQUAL 0)
  message(FATAL_ERROR "${CLI} exited with ${code}")
endif()
file(READ "${GOLDEN}" golden)
if(NOT out STREQUAL golden)
  message(FATAL_ERROR "output differs from ${GOLDEN}")
endif()
