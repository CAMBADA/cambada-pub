include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Grabber.cc)
SET (headers Grabber.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
