INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE (${gazebo_cmake_dir}/FindOS.cmake)

INCLUDE (FindPkgConfig)
INCLUDE (${gazebo_cmake_dir}/FindOde.cmake)
INCLUDE (${gazebo_cmake_dir}/Cambada.cmake )

set (boost_include_dirs "" CACHE STRING "Boost include paths. Use this to override automatic detection.")
set (boost_library_dirs "" CACHE STRING "Boost library paths. Use this to override automatic detection.")
set (boost_libraries "" CACHE STRING "Boost libraries. Use this to override automatic detection.")

set (threadpool_include_dirs "" CACHE STRING "Threadpool include paths. Use this to override automatic detection.")

########################################
# Find packages
if (PKG_CONFIG_FOUND)

  pkg_check_modules(XML libxml-2.0)
  IF (NOT XML_FOUND)
    BUILD_ERROR("libxml2 and development files not found. See the following website: http://www.xmlsoft.org")
  ELSE (NOT XML_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${XML_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${XML_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${XML_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${XML_LIBRARIES})
  ENDIF (NOT XML_FOUND)
  
  pkg_check_modules(GL gl)
  IF (NOT GL_FOUND)
    BUILD_ERROR("OpenGL development files not found.")
  ELSE (NOT GL_FOUND)
    APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                          ${gazeboserver_include_dirs_desc} 
                          ${GL_INCLUDE_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                          ${gazeboserver_link_dirs_desc} 
                          ${GL_LIBRARY_DIRS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${GL_LINK_LIBS})
    APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                          ${gazeboserver_link_libs_desc} 
                          ${GL_LIBRARIES})
  ENDIF (NOT GL_FOUND)

ELSE (PKG_CONFIG_FOUND)
  SET (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
  MESSAGE (FATAL_ERROR "\nError: pkg-config not found")
ENDIF (PKG_CONFIG_FOUND)


########################################
# Find Boost, if not specified manually
IF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )

  # Clear some variables to ensure that the checks for boost are 
  # always run
  SET (Boost_THREAD_FOUND OFF CACHE INTERNAL "" FORCE)
  SET (Boost_SIGNALS_FOUND OFF CACHE INTERNAL "" FORCE)
  SET (Boost_SYSTEM_FOUND OFF CACHE INTERNAL "" FORCE)

  SET(Boost_ADDITIONAL_VERSIONS "1.35" "1.35.0" "1.36" "1.36.1" "1.37.0" "1.39.0" CACHE INTERNAL "Boost Additional versions" FORCE)
  INCLUDE (FindBoost)

  FIND_PACKAGE( Boost ${MIN_BOOST_VERSION} REQUIRED thread signals system)

  IF (NOT Boost_FOUND)
    SET (BUILD_GAZEBO OFF CACHE INTERNAL "Build Gazebo" FORCE)
    MESSAGE (FATAL_ERROR "Boost thread and signals not found. Please install Boost threads and signals version ${MIN_BOOST_VERSION} or higher.")
  ENDIF (NOT Boost_FOUND)

  SET (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  SET (boost_library_dirs ${Boost_LIBRARY_DIRS} CACHE STRING
    "Boost link dirs. Use this to override automatic detection." FORCE)

  LIST_TO_STRING(tmp "${Boost_LIBRARIES}")
  SET (boost_libraries ${tmp} CACHE STRING 
    "Boost libraries. Use this to override automatic detection." FORCE )

ENDIF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries ) 

STRING(REGEX REPLACE "(^| )-L" " " boost_library_dirs "${boost_library_dirs}")
STRING(REGEX REPLACE "(^| )-l" " " boost_libraries "${boost_libraries}")
#STRING(STRIP ${boost_library_dirs} boost_library_dirs)
#STRING(STRIP ${boost_libraries} boost_libraries)
STRING(REGEX REPLACE " " ";" boost_libraries "${boost_libraries}")

MESSAGE (STATUS "Boost Include Path: ${boost_include_dirs}")
MESSAGE (STATUS "Boost Library Path: ${boost_library_dirs}")
MESSAGE (STATUS "Boost Libraries: ${boost_libraries}")

########################################
# For Threadpool
message (STATUS "Threadpool Include Path: ${threadpool_include_dirs}")

########################################
# Find libtool
FIND_PATH(libtool_include_dir ltdl.h /usr/include /usr/local/include)
IF (NOT libtool_include_dir)
  MESSAGE (STATUS "Looking for ltdl.h - not found")
  MESSAGE (STATUS "Warning: Unable to find libtool, plugins will not be supported.")
  SET (libtool_include_dir /usr/include)
ELSE (NOT libtool_include_dir)
  MESSAGE (STATUS "Looking for ltdl.h - found")
ENDIF (NOT libtool_include_dir)

FIND_LIBRARY(libtool_library ltdl /usr/lib /usr/local/lib)
IF (NOT libtool_library)
  MESSAGE (STATUS "Looking for libltdl - not found")
  MESSAGE (STATUS "Warning: Unable to find libtool, plugins will not be supported.")
ELSE (NOT libtool_library)
  MESSAGE (STATUS "Looking for libltdl - found")
ENDIF (NOT libtool_library)

IF (libtool_library AND libtool_include_dir)
  SET (HAVE_LTDL TRUE)
ENDIF (libtool_library AND libtool_include_dir)

########################################
# Find libdl
FIND_PATH(libdl_include_dir dlfcn.h /usr/include /usr/local/include)
IF (NOT libdl_include_dir)
  MESSAGE (STATUS "Looking for dlfcn.h - not found")
  MESSAGE (STATUS "Warning: Unable to find libdl, plugins will not be supported.")
  SET (libdl_include_dir /usr/include)
ELSE (NOT libdl_include_dir)
  MESSAGE (STATUS "Looking for dlfcn.h - found")
ENDIF (NOT libdl_include_dir)

FIND_LIBRARY(libdl_library dl /usr/lib /usr/local/lib)
IF (NOT libdl_library)
  MESSAGE (STATUS "Looking for libdl - not found")
  MESSAGE (STATUS "Warning: Unable to find libdl, plugins will not be supported.")
ELSE (NOT libdl_library)
  MESSAGE (STATUS "Looking for libdl - found")
ENDIF (NOT libdl_library)

IF (libdl_library AND libdl_include_dir)
  SET (HAVE_DL TRUE)
ENDIF (libdl_library AND libdl_include_dir)


### Check Cambada dirs ###
##########################
IF (NOT cambada_include_dir)
    BUILD_ERROR ("CAMBADA include directories not set")
ENDIF (NOT cambada_include_dir)

IF (NOT cambada_lib_dir)
    BUILD_ERROR ("CAMBADA libraries directories not set")
ENDIF (NOT cambada_lib_dir)
