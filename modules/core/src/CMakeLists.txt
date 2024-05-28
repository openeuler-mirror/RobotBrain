include_directories("../include")
aux_source_directory(./ DIR_SRCS)

find_package (glog 0.6.0 REQUIRED)
find_package(Threads REQUIRED)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# 主要用于解决typegen时候找不到头文件位置的问题
# set(ENV{CPLUS_INCLUDE_PATH} ${PROJECT_SOURCE_DIR}/modules/config_tools/include/)
# include_directories(${PROJECT_SOURCE_DIR}/modules/config_tools/include/)
# orocos_typegen_headers(${PROJECT_SOURCE_DIR}/modules/core/include/robot_brain/core/ethercat_frame_types.hpp
#                       # ${PROJECT_SOURCE_DIR}/modules/core/include/robot_brain/core/command_frame_types.hpp
#                       # ...you may add multiple header files


add_library(${MODULE_NAME} SHARED ${DIR_SRCS})
# ADD_LIBRARY(${MODULE_NAME} SHARED ${DIR_SRCS})
target_link_options(${MODULE_NAME} PRIVATE LINKER:-Bsymbolic)

set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost 1.50 REQUIRED COMPONENTS filesystem)
message(STATUS "Boost version: ${Boost_VERSION}")

# This is needed if your Boost version is newer than your CMake version
# or if you have an old version of CMake (<3.5)
if(NOT TARGET Boost::filesystem)
    add_library(Boost::filesystem IMPORTED INTERFACE)
    set_property(TARGET Boost::filesystem PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
    set_property(TARGET Boost::filesystem PROPERTY
        INTERFACE_LINK_LIBRARIES ${Boost_LIBRARIES})
endif()

target_include_directories(${MODULE_NAME} PUBLIC

  # ${PROJECT_SOURCE_DIR}/modules/robot_planning/include
  ${PROJECT_SOURCE_DIR}/modules/robot_exception/include
)
target_link_libraries(${MODULE_NAME} PUBLIC crypto)
target_link_libraries(${MODULE_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${MODULE_NAME} PUBLIC glog::glog)
target_link_libraries(${MODULE_NAME} PUBLIC Threads::Threads)
target_link_libraries(${MODULE_NAME} PUBLIC orocos-kdl)
target_link_libraries(${MODULE_NAME} PUBLIC yaml-cpp)
target_link_libraries(${MODULE_NAME} PUBLIC ${LUA_LIBRARIES})
target_link_libraries(${MODULE_NAME} PUBLIC Boost::filesystem)
target_link_libraries(${MODULE_NAME} PUBLIC robot_exception)

install(TARGETS ${MODULE_NAME}
       LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ../include/robot_brain/
  DESTINATION include/robot_brain
)
