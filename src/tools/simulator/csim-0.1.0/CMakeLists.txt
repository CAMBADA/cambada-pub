#CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

#IF(COMMAND CMAKE_POLICY)
#  CMAKE_POLICY(SET CMP0003 NEW)
#  CMAKE_POLICY(SET CMP0004 NEW)
#ENDIF(COMMAND CMAKE_POLICY)

#project ("CAMBADA Simulator")
#string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (CSIM_MAJOR_VERSION 0)
set (CSIM_MINOR_VERSION 1)
set (CSIM_PATCH_VERSION 0)

set (CSIM_VERSION ${CSIM_MAJOR_VERSION}.${CSIM_MINOR_VERSION}.${CSIM_PATCH_VERSION})

#### Also keep base code version ####
set (GAZEBO_MAJOR_VERSION 0)
set (GAZEBO_MINOR_VERSION 10)
set (GAZEBO_PATCH_VERSION 0)

set (GAZEBO_VERSION ${GAZEBO_MAJOR_VERSION}.${GAZEBO_MINOR_VERSION}.${GAZEBO_PATCH_VERSION})
#####################################

set (BUILD_CSIM ON CACHE INTERNAL "Build CAMBADA Simulator" FORCE)

set (gazeboserver_sources_desc "List of server sources" 
     CACHE INTERNAL "CSim server sources list description" FORCE)
set (gazeboserver_headers_desc "List of server headers" 
     CACHE INTERNAL "CSim server headers list description" FORCE)
set (gazeboserver_include_dirs_desc "List of server include directories" 
     CACHE INTERNAL "Include dirs description")
set (gazeboserver_link_dirs_desc "List of server link directories" 
     CACHE INTERNAL "Link dirs description" FORCE)
set (gazeboserver_link_libs_desc "List of server link libraries" 
     CACHE INTERNAL "Link libs description" FORCE)
set (gazeboserver_cflags_desc "List of server cflags" 
     CACHE INTERNAL "Server cflags description" FORCE)
set (gazebosensor_sources_desc "List of sensor sources" 
     CACHE INTERNAL "Gazebo sensor sources list description" FORCE)
set (gazebocontroller_sources_desc "List of controller sources" 
     CACHE INTERNAL "Gazebo controller sources list description" FORCE)

set (ODE_VERSION 0.10.1 CACHE INTERNAL "ODE version requirement" FORCE)
set (MIN_BOOST_VERSION 1.35.0 CACHE INTERNAL "Boost min version requirement" FORCE)

set (ENABLE_BINDINGS OFF CACHE BOOL "Enable libgazebo bindings")

set (gazebo_cmake_dir ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

INCLUDE (${gazebo_cmake_dir}/DissectVersion.cmake)

# Change default prefix
SET(CSIM_INSTALL_PREFIX "${BASE_DIR}" CACHE PATH     "Prefix prepended to install directories")
SET(CMAKE_INSTALL_PREFIX "${CSIM_INSTALL_PREFIX}"    CACHE INTERNAL "Prefix prepended to install directories" FORCE)


STRING(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

SET (build_errors "" CACHE INTERNAL "build errors" FORCE)

INCLUDE (${gazebo_cmake_dir}/SearchForStuff.cmake)

if ( build_errors )
  message(STATUS "!!! BUILD ERRORS !!!")
  message(STATUS "These errors must be resolved before compiling")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach (msg ${build_errors})
  message(STATUS "!!! END BUILD ERRORS !!!\n")
else (build_errors)
  ########################################
  # Write the config.h file
  CONFIGURE_FILE (${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
  
  include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
  
  if (BUILD_CSIM)
    add_subdirectory(3rd_party)
    add_subdirectory(libgazebo)
    add_subdirectory(server)
    add_subdirectory(worlds)
    
#    message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
#    message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
  endif (BUILD_CSIM)

  #message(STATUS "Configuration successful. Type make to compile csim")

endif(build_errors)

GAZEBOSERVER_RESET_LISTS()
