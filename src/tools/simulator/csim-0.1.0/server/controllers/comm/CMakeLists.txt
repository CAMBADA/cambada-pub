include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources comm.cc)
SET (headers comm.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
