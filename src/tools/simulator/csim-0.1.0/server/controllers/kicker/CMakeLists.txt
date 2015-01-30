include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Kicker.cc)
SET (headers Kicker.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})