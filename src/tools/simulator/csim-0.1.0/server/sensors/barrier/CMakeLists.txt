include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources BarrierSensor.cc)
set (headers BarrierSensor.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
