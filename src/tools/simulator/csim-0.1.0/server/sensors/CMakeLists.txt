include (${gazebo_cmake_dir}/GazeboUtils.cmake)

## disable unneccessary sensors ##
#ADD_SUBDIRECTORY(imu)
#ADD_SUBDIRECTORY(ray)
#ADD_SUBDIRECTORY(ir)

ADD_SUBDIRECTORY(contact)
ADD_SUBDIRECTORY(vision)
ADD_SUBDIRECTORY(compass)
ADD_SUBDIRECTORY(barrier)

SET (sources Sensor.cc
             SensorFactory.cc
             SensorManager.cc
) 

SET (headers Sensor.hh
             SensorFactory.hh
             SensorManager.hh
)

#ADD_LIBRARY(gazebo_sensors STATIC ${gazebosensor_sources} ${sources})
APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
