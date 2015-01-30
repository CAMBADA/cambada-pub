include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Draw.cc
						 drawstuff.cc
						 x11.cc
)

SET (headers Draw.hh
						 drawstuff.hh
)

add_library(gazebo_draw SHARED ${sources})

IF (APPLE)
set_target_properties(gazebo_draw PROPERTIES LINK_FLAGS "-flat_namespace -undefined suppress -framework GLUT")
ELSE (APPLE)
target_link_libraries( gazebo_draw glut)
ENDIF (APPLE)

target_link_libraries( gazebo_draw ${X11_LIBRARIES} ${GL_LIBRARIES})
install ( TARGETS gazebo_draw DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
