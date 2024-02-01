cmake_minimum_required (VERSION 3.22.1)
set(MODULE_NAME "test_com")
message(STATUS "Module ${MODULE_NAME}," ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)
if(ROSC_ROBOT_BRAIN_BUILD_TESTING)
    # add_subdirectory(test)
endif()
