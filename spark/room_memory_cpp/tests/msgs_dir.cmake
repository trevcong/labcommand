# Where the labcommand_msgs sources are. Two layouts hold the same tree:
#   checkout: <repo>/ros/labcommand_msgs
#   image:    the Dockerfile copies that package to /ws/src/labcommand_msgs, and REPO_ROOT
#             there is /ws (three levels above tests/), so the checkout path does not exist.
# An explicit path (-DROOM_MEMORY_MSGS_DIR, including a cache entry) is used unchanged.
# Empty means "choose". The checkout wins when it is present, so a stray sibling cannot
# hide the tree this repository builds. Otherwise the colcon sibling of this package is used.
function(room_memory_resolve_msgs_dir repo_root tests_dir explicit out_var)
  if(NOT "${explicit}" STREQUAL "")
    set(chosen "${explicit}")
  elseif(EXISTS "${repo_root}/ros/labcommand_msgs/CMakeLists.txt")
    set(chosen "${repo_root}/ros/labcommand_msgs")
  elseif(EXISTS "${tests_dir}/../../labcommand_msgs/CMakeLists.txt")
    get_filename_component(chosen "${tests_dir}/../../labcommand_msgs" ABSOLUTE)
  else()
    set(chosen "${repo_root}/ros/labcommand_msgs")
  endif()
  set(${out_var} "${chosen}" PARENT_SCOPE)
endfunction()

# The interface test reads these two. A configure that points elsewhere fails here, not
# later as "cannot read ..." inside colcon test-result.
function(room_memory_msgs_dir_usable dir out_var)
  if(EXISTS "${dir}/CMakeLists.txt" AND EXISTS "${dir}/msg/Activity.msg")
    set(${out_var} YES PARENT_SCOPE)
  else()
    set(${out_var} NO PARENT_SCOPE)
  endif()
endfunction()

function(room_memory_require_msgs_dir dir)
  room_memory_msgs_dir_usable("${dir}" ok)
  if(NOT ok)
    message(FATAL_ERROR
      "ROOM_MEMORY_MSGS_DIR '${dir}' is not a labcommand_msgs source tree "
      "(need CMakeLists.txt and msg/Activity.msg). "
      "The checkout layout is <repo>/ros/labcommand_msgs. "
      "The image copies the package beside room_memory_cpp and must pass that path "
      "as -DROOM_MEMORY_MSGS_DIR.")
  endif()
endfunction()
