cmake_minimum_required (VERSION 3.8)

set(MODULE_NAME "robot_interpolation")
message(STATUS "Module ${MODULE_NAME}," ${CMAKE_CURRENT_SOURCE_DIR})

#set(CMAKE_C_FLAGS "-g")

add_subdirectory(src)

if(ROSC_ROBOT_BRAIN_BUILD_TESTING)
    add_subdirectory(test)
endif()
