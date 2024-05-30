cmake_minimum_required (VERSION 3.8)

include_directories("../include")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "FIND Eigen3 " ${EIGEN3_INCLUDE_DIR})

aux_source_directory(./ DIR_SRCS)

add_library(${MODULE_NAME} SHARED ${DIR_SRCS})

target_include_directories(${MODULE_NAME} PUBLIC
                           ${PROJECT_SOURCE_DIR}/3rdparty/include
                           ${PROJECT_SOURCE_DIR}/modules/core/include
                           ${PROJECT_SOURCE_DIR}/modules/robot_planning/include
)

target_link_libraries(${MODULE_NAME} PRIVATE orocos-kdl)
target_link_libraries(${MODULE_NAME} PUBLIC core)
# target_link_libraries(${MODULE_NAME} PUBLIC glog::glog)
target_link_libraries(${MODULE_NAME} PRIVATE yaml-cpp)
target_link_libraries(${MODULE_NAME} PRIVATE robot_planning)
target_link_libraries(${MODULE_NAME} PRIVATE Eigen3::Eigen)

install(TARGETS ${MODULE_NAME}
       LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ../include/robot_brain/
  DESTINATION include/robot_brain
)