include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources ODEPhysics.cc
             ODEGeom.cc
             ODEBody.cc
             ODEJoint.cc
             ODESliderJoint.cc
             ODEHingeJoint.cc
             ODEHinge2Joint.cc
             ODEUniversalJoint.cc
             ODEBallJoint.cc
             ODERayShape.cc
             ODEMultiRayShape.cc
)

SET (headers ODEPhysics.hh
             ODEGeom.hh
             ODEBody.hh
             ODEJoint.hh
             ODESliderJoint.hh
             ODEHingeJoint.hh
             ODEHinge2Joint.hh
             ODEUniversalJoint.hh
             ODEBallJoint.hh
             ODERayShape.hh
             ODEMultiRayShape.hh
)

add_library(gazebo_physics_ode EXCLUDE_FROM_ALL SHARED ${sources})
set_target_properties(gazebo_physics_ode PROPERTIES COMPILE_FLAGS "-fPIC")

IF (APPLE)
set_target_properties(gazebo_physics_ode PROPERTIES LINK_FLAGS "-flat_namespace -undefined suppress")
ENDIF (APPLE)

target_link_libraries( gazebo_physics_ode ${ODE_LIBRARIES})
#install ( TARGETS gazebo_physics_ode DESTINATION ${BASE_DIR}/lib )
