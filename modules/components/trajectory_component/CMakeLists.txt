cmake_minimum_required (VERSION 3.8)
set(MODULE_NAME "trajectory_component")
message(STATUS "Module ${MODULE_NAME}," ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)
if(ROSC_ROBOT_BRAIN_BUILD_TESTING)
   # add_subdirectory(test)
endif()