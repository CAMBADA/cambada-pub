INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

## Find eigen
pkg_check_modules(EIGEN3 REQUIRED eigen3)

########################################
# Include all the search paths for headers
STRING (REPLACE " " ";" threadpool_include_dirs_split "${threadpool_include_dirs}")

INCLUDE_DIRECTORIES(
  .
  ${CMAKE_CURRENT_SOURCE_DIR}/../libgazebo
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/jsoncpp
  ${EIGEN3_INCLUDE_DIRS}
  visual
  physics 
  physics/ode
  controllers 
  rendering 
  sensors 
  sensors/contact
#  sensors/imu
#  sensors/ir
#  sensors/ray
  sensors/barrier
  referee
  ${libtool_include_path}
  ${gazeboserver_include_dirs}
  ${boost_include_dirs}
  ${threadpool_include_dirs_split}
  ${cambada_include_dir}
)

LINK_DIRECTORIES(  
# ${CMAKE_BINARY_DIR}/libgazebo  
 ${boost_library_dirs}
 ${gazeboserver_link_dirs}
 ${cambada_lib_dir}
)


########################################
# All the library search paths
ADD_SUBDIRECTORY(physics)
ADD_SUBDIRECTORY(sensors)
ADD_SUBDIRECTORY(controllers)
ADD_SUBDIRECTORY(referee)
ADD_SUBDIRECTORY(visual)


########################################
# Process all the subdirectories
SET (sources Common.cc
             Vector3.cc 
             Vector4.cc 
             Quatern.cc
             Pose3d.cc
             World.cc
             XMLConfig.cc
             GazeboConfig.cc
             Time.cc
             Timer.cc
             Entity.cc
             GazeboError.cc
             GazeboMessage.cc
             Model.cc
             Angle.cc
             Param.cc
             Simulator.cc
             Rand.cc
             Factory.cc
						 Color.cc
						 Material.cc
						 Field.cc
)

SET (headers Common.hh
             Entity.hh
             GazeboError.hh
             GazeboMessage.hh
             Global.hh
             Model.hh
             Pose3d.hh
             Quatern.hh
             SingletonT.hh
             StaticPluginRegister.hh
             Time.hh
             Timer.hh
             Vector2.hh
             Vector3.hh
             Vector4.hh
             World.hh
             XMLConfig.hh
             GazeboConfig.hh
             Angle.hh
             Param.hh
             Simulator.hh
             Rand.hh
             Factory.hh
						 Color.hh
						 Material.hh
						 Field.hh
)

APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})

ADD_LIBRARY(gazebo_server EXCLUDE_FROM_ALL SHARED ${gazeboserver_sources})
SET_TARGET_PROPERTIES(gazebo_server PROPERTIES VERSION ${GAZEBO_VERSION})

IF (APPLE)
set_target_properties(gazebo_server PROPERTIES LINK_FLAGS "-flat_namespace -undefined suppress")
ENDIF (APPLE)

ADD_EXECUTABLE(csim-exec EXCLUDE_FROM_ALL main.cc)
SET_TARGET_PROPERTIES(csim-exec PROPERTIES OUTPUT_NAME "csim")

# Removes colon separated rpath. This breaks Ubuntu-intrepid 
# (possibly gcc-4.3.2).
SET_TARGET_PROPERTIES(csim-exec PROPERTIES SKIP_BUILD_RPATH TRUE)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

IF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
ENDIF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries( csim-exec
  ${gazeboserver_link_libs} 
  ${libtool_library}
  ${boost_libraries} 
  gazebo_server
  gazebo_physics
  gazebo
  gazebo_visual
  rtdb
	pman
	geom
  worldstate
  jsoncpp
 	GLU
)

target_link_libraries( gazebo_server ${libtool_library} gazeboshm gazebo_physics xml2)

if (INCLUDE_ODE)
  target_link_libraries(csim-exec gazebo_physics_ode ${ODE_LIBRARIES})
  target_link_libraries(gazebo_server gazebo_physics_ode ${ODE_LIBRARIES})
endif (INCLUDE_ODE)

#install (TARGETS gazebo_server DESTINATION ${BASE_DIR}/lib)
#install (TARGETS csim-exec DESTINATION ${BASE_DIR}/bin)


## the server headers is only needed for controller and sensor plugins.. 
#install (FILES ${gazeboserver_headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)
