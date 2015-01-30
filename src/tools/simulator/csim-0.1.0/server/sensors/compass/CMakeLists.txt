include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources CompassSensor.cc)
set (headers CompassSensor.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
