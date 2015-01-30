include (${gazebo_cmake_dir}/GazeboUtils.cmake)

## disable unneccessary controllers ##
#ADD_SUBDIRECTORY(actarray)
#ADD_SUBDIRECTORY(bumper)
#ADD_SUBDIRECTORY(gripper)
#ADD_SUBDIRECTORY(imu)
#ADD_SUBDIRECTORY(irarray)
#ADD_SUBDIRECTORY(laser)
#ADD_SUBDIRECTORY(opaque)
ADD_SUBDIRECTORY(position2d)
#ADD_SUBDIRECTORY(ptz)

ADD_SUBDIRECTORY(comm)
ADD_SUBDIRECTORY(kicker)
ADD_SUBDIRECTORY(grabber)

SET (sources Controller.cc 
             ControllerFactory.cc
)

SET (headers Controller.hh
             ControllerFactory.hh
)


APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})
