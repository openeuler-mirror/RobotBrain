# Generated from orogen/lib/orogen/templates/typekit/typelib/CMakeLists.txt

# This policy indicates that the code below is cmake 2.6 compliant. This is necessary
# To properly escape the add_definitions arguments.
cmake_policy(VERSION 2.6)

set(DEPS_CFLAGS_OTHER "")

list(REMOVE_DUPLICATES DEPS_CFLAGS_OTHER)
add_definitions(${DEPS_CFLAGS_OTHER})

orogen_pkg_check_modules(TYPELIB REQUIRED typelib)
include_directories(${TYPELIB_INCLUDE_DIRS})
link_directories(${TYPELIB_LIBRARY_DIRS})

orogen_pkg_check_modules(RTT_Typelib REQUIRED rtt_typelib-${OROCOS_TARGET})
include_directories(${RTT_Typelib_INCLUDE_DIRS})
link_directories(${RTT_Typelib_LIBRARY_DIRS})

set(TYPEKIT_REGISTRY ${CMAKE_INSTALL_PREFIX}/share/orogen/robot_brain.tlb)
add_definitions(-DTYPEKIT_REGISTRY="${TYPEKIT_REGISTRY}")
set(libname_typelib robot_brain-transport-typelib-${OROCOS_TARGET})
add_library(${libname_typelib} SHARED
    __rosc__Arm_pos___rosc__AxisNum.cpp
    __rosc__CommandExecResult___rosc__CommandType.cpp
    __rosc__DigitIOFrame___rosc__EncoderFrame.cpp
    __rosc__EthercatCtlFrame___rosc__EthercatFrameSendStatus.cpp
    __rosc__EthercatStatusFrame___rosc__ExecStatus.cpp
    __rosc__MotionMode___rosc__ServoState.cpp
    __rosc__TransferSpeedType___std__vector____std__string__.cpp
    TransportPlugin.cpp)

target_link_libraries(${libname_typelib}
    ${TYPELIB_LIBRARIES}
    ${OrocosRTT_LIBRARIES}
    ${RTT_Typelib_LIBRARIES}
    )



# rospack support: if we're a ROS package, use rospack to return the location of our .tlb file.
if ( IS_ROS_PACKAGE )
      set(ROS_ROOT $ENV{ROS_ROOT})
      # Catkin style of fetching ROS deps
      FIND_PACKAGE( ROS QUIET COMPONENTS roslib )

      IF(NOT ROS_FOUND) # pre-Fuerte or Groovy
        # TODO: This block to get roslib is deprecated as of ROS Fuerte, but is left in for pre-Fuerte compatibility
        set(roslib_PACKAGE_PATH ${ROS_ROOT}/core/roslib) ### XXX hardcoded
        message("Falling back to roslib in: ${roslib_PACKAGE_PATH}")
        find_library(ROS_LIBRARIES roslib ${roslib_PACKAGE_PATH}/lib )
        if ( NOT ROS_LIBRARIES )
          find_package( ROS COMPONENTS roslib ) # Yells at user (non-QUIET !)
        else()
          set(ROS_FOUND TRUE)
          set(ROS_INCLUDE_DIRS ${roslib_PACKAGE_PATH}/include)
        endif()
      ENDIF(NOT ROS_FOUND)
      
      IF(ROS_FOUND)
        include_directories( ${ROS_INCLUDE_DIRS} )
        add_definitions( -DHAS_ROSLIB )
	target_link_libraries( ${libname_typelib} ${ROS_LIBRARIES} )
      ENDIF(ROS_FOUND)

endif( IS_ROS_PACKAGE )

SET(PKG_CONFIG_FILE_TYPELIB ${CMAKE_CURRENT_BINARY_DIR}/robot_brain-transport-typelib-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(robot_brain-transport-typelib.pc.in ${PKG_CONFIG_FILE_TYPELIB} @ONLY)

install(FILES ${PKG_CONFIG_FILE_TYPELIB} DESTINATION lib/pkgconfig)
install(FILES
    TransportPlugin.hpp
    DESTINATION include/orocos/robot_brain/transports/typelib)
install(TARGETS ${libname_typelib} LIBRARY DESTINATION lib/orocos${OROCOS_PKG_DIR}/types)

