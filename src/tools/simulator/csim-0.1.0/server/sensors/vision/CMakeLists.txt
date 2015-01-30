include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources SensorVision.cc FrontVision.cc)
set (headers SensorVision.hh FrontVision.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})