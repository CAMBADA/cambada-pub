INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Server.cc 
             Client.cc 
             Iface.cc 
             IfaceFactory.cc 
             SimIface.cc 
)

SET (headers gazebo.h IfaceFactory.hh)

INCLUDE_DIRECTORIES(
  ${boost_include_dirs}
)

IF (${ENABLE_BINDINGS})
  ADD_SUBDIRECTORY(bindings)
ENDIF (${ENABLE_BINDINGS})

LINK_DIRECTORIES(${boost_library_dirs})

ADD_LIBRARY(gazeboshm EXCLUDE_FROM_ALL SHARED ${sources})
SET_TARGET_PROPERTIES(gazeboshm PROPERTIES OUTPUT_NAME "gazebo" VERSION ${GAZEBO_VERSION})

TARGET_LINK_LIBRARIES( gazeboshm ${boost_libraries})

#INSTALL (TARGETS gazeboshm DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
#INSTALL (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo)

APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                      ${gazeboserver_link_dirs_desc} 
                      ${CMAKE_CURRENT_BINARY_DIR})

APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                      ${gazeboserver_link_libs_desc} 
                      gazeboshm)

