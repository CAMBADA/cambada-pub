include (${gazebo_cmake_dir}/GazeboUtils.cmake)

if (INCLUDE_ODE)
  add_subdirectory(ode)
endif (INCLUDE_ODE)

set (sources Body.cc
             PhysicsFactory.cc
             PhysicsEngine.cc
             SurfaceParams.cc
             Mass.cc
             Joint.cc
             Geom.cc
             Shape.cc
             BoxShape.cc
             CylinderShape.cc
             PlaneShape.cc
             SphereShape.cc
             RayShape.cc
             MultiRayShape.cc
             Contact.cc
) 

set (headers BallJoint.hh
             Body.hh
             PhysicsFactory.hh
             SurfaceParams.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             PhysicsEngine.hh
             SliderJoint.hh
             UniversalJoint.hh
             Mass.hh
             Geom.hh
             Shape.hh
             CylinderShape.hh
             MapShape.hh
             PlaneShape.hh
             RayShape.hh
             MultiRayShape.hh
             SphereShape.hh
             Contact.hh
             JointFeedback.hh
)

add_library(gazebo_physics EXCLUDE_FROM_ALL SHARED ${sources})

IF (APPLE)
set_target_properties(gazebo_physics PROPERTIES LINK_FLAGS "-flat_namespace -undefined suppress")
ENDIF (APPLE)

#install ( TARGETS gazebo_physics DESTINATION ${BASE_DIR}/lib )
#APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
