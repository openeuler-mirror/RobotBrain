# Generated from orogen/lib/orogen/templates/typekit/CMakeLists.txt

##
# Header for typegen-generated typekits
#
project("robot_brain-typekit")
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")

# In Orogen components, the build target is specified at generation time
# But we allow to override in case of typegen/USE_OROCOS_RTT:
if (NOT OROCOS_TARGET)
  set(OROCOS_TARGET "gnulinux")
endif()
# If UseOrocos-RTT.cmake is used, install libs in /target/package subdir in order to allow
# multi-target/package installs.
if ( USE_OROCOS_RTT )
  if ( NOT OROCOS_SUFFIX )
    set (OROCOS_SUFFIX "/${OROCOS_TARGET}")
  endif()
  set(OROCOS_PKG_DIR "${OROCOS_SUFFIX}/robot_brain")
endif()

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif (CXX_SUPPORTS_WALL)

# Define the necessary RPath information for all binaries. That can be stripped
# later
option(WITH_RPATH "enables or disables embedding RPath information in binaries" ON)
if(WITH_RPATH)
    include(RPATHHandling)
    CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib/orocos;${CMAKE_INSTALL_PREFIX}/lib/orocos${OROCOS_PKG_DIR}/types")
endif(WITH_RPATH)

# Set the build type to debug by default
IF ( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug )
ENDIF ( NOT CMAKE_BUILD_TYPE )

include(OrogenPkgCheckModules)
orogen_pkg_check_modules(OrocosRTT REQUIRED "orocos-rtt-${OROCOS_TARGET}>=1.99.4")

ADD_CUSTOM_TARGET(regen
    
   typegen --transports=typelib --output=./typekit robot_brain include/robot_brain/ethercat_frame_types.hpp include/robot_brain/command_types.hpp
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# END if typekit.standalone?
##

# Manually define '-DIS_ROS_PACKAGE' or include the UseOrocos.cmake file to automatically detect&set this flag.
if ( IS_ROS_PACKAGE )
    # When we import a typekit in a ROS package, we expect it to be in the package/lib directory
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX}/types)
endif( IS_ROS_PACKAGE ) 

execute_process(COMMAND cmake -E create_symlink
    ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/robot_brain)

include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/types)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OrocosRTT_INCLUDE_DIRS})
add_definitions(${OrocosRTT_CFLAGS_OTHER})
link_directories(${OrocosRTT_LIBRARY_DIRS})


# Now set up the dependencies
set(DEPS_CFLAGS_OTHER "")

list(REMOVE_DUPLICATES DEPS_CFLAGS_OTHER)
add_definitions(${DEPS_CFLAGS_OTHER})

set(PKG_CFLAGS ${OrocosRTT_CFLAGS})

list(REMOVE_DUPLICATES PKG_CFLAGS)
string(REPLACE ";" "\" \"" PKG_CFLAGS "\"${PKG_CFLAGS}\"")

# Generate the base typekit shared library
set(libname robot_brain-typekit-${OROCOS_TARGET})
add_library(${libname} SHARED
    Plugin.cpp
    type_info/__rosc__Arm_pos___rosc__AxisNum.cpp
    type_info/__rosc__CommandExecResult___rosc__CommandType.cpp
    type_info/__rosc__DigitIOFrame___rosc__EncoderFrame.cpp
    type_info/__rosc__EthercatCtlFrame___rosc__EthercatFrameSendStatus.cpp
    type_info/__rosc__EthercatStatusFrame___rosc__ExecStatus.cpp
    type_info/__rosc__MotionMode___rosc__ServoState.cpp
    type_info/__rosc__TransferSpeedType___std__vector____std__string__.cpp
    type_info/boost__int16_t[]_boost__int32_t[].cpp
    type_info/boost__int8_t[]_boost__uint16_t[].cpp
    type_info/boost__uint8_t[]_double[].cpp
    ${TYPEKIT_ADDITIONAL_SOURCES}
    ${TOOLKIT_ADDITIONAL_SOURCES})


target_link_libraries(${libname} ${OrocosRTT_LIBRARIES} ${TYPEKIT_ADDITIONAL_LIBRARIES} ${TOOLKIT_ADDITIONAL_LIBRARIES})
set_target_properties(${libname} PROPERTIES LINK_INTERFACE_LIBRARIES ${OrocosRTT_LIBRARIES})
set_target_properties(${libname} PROPERTIES INTERFACE_LINK_LIBRARIES ${OrocosRTT_LIBRARIES})
if(WITH_RPATH AND APPLE)
  set_target_properties( ${libname} PROPERTIES
    INSTALL_NAME_DIR "@rpath")
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
endif()
set(PKG_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/robot_brain-typekit-${OROCOS_TARGET}.pc)
configure_file(robot_brain-typekit.pc.in ${PKG_CONFIG_FILE} @ONLY)

install(TARGETS ${libname} LIBRARY DESTINATION lib/orocos${OROCOS_PKG_DIR}/types)
install(FILES Types.hpp Opaques.hpp
    DESTINATION include/orocos/robot_brain)
install(FILES
    
    
    DESTINATION include/orocos/robot_brain/typekit)


install(FILES types/robot_brain/typekit/Types.hpp
    DESTINATION include/orocos/robot_brain/typekit)

install(FILES types/robot_brain/typekit/BoostSerialization.hpp
    DESTINATION include/orocos/robot_brain/typekit)

install(FILES types/robot_brain/typekit/Plugin.hpp
    DESTINATION include/orocos/robot_brain/typekit)

install(FILES ../include/robot_brain/ethercat_frame_types.hpp
    DESTINATION include/orocos/robot_brain)

install(FILES ../include/robot_brain/command_types.hpp
    DESTINATION include/orocos/robot_brain)
install(FILES ${PKG_CONFIG_FILE}
    DESTINATION lib/pkgconfig)
install(FILES robot_brain.tlb
    robot_brain.typelist
    DESTINATION share/orogen)


    add_subdirectory(transports/typelib)
    

# Force the user to regenerate its typekit if the inputs changed
set(TK_STAMP "${CMAKE_CURRENT_SOURCE_DIR}/stamp")
get_directory_property(include_path INCLUDE_DIRECTORIES)
foreach(dep robot_brain/command_types.hpp robot_brain/ethercat_frame_types.hpp)
    find_file(dep_full_path ${dep} PATHS ${include_path})
    if (NOT dep_full_path)
        message(FATAL_ERROR "cannot find ${dep} in ${include_path}. This should never happen, report this as an oroGen bug")
    endif()
    list(APPEND TYPEKIT_SOURCE_HEADERS ${dep_full_path})
    unset(dep_full_path CACHE)
endforeach()
add_custom_command(
    OUTPUT "${TK_STAMP}"
    DEPENDS ${TYPEKIT_SOURCE_HEADERS}
    COMMENT "Typekit input changed. Run make regen in your build directory first"
    COMMAND /bin/false)
add_custom_target(check-typekit-uptodate ALL DEPENDS "${TK_STAMP}")
add_dependencies(${libname} check-typekit-uptodate)

