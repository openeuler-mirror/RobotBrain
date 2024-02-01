cmake_minimum_required (VERSION 3.22.1)

include_directories("../include")

aux_source_directory(./ DIR_SRCS)

add_library(${MODULE_NAME} SHARED ${DIR_SRCS})
# target_link_options(${MODULE_NAME} PRIVATE LINKER:-Bsymbolic)

install(TARGETS ${MODULE_NAME}
       LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ../include/robot_brain/
  DESTINATION include/robot_brain
)