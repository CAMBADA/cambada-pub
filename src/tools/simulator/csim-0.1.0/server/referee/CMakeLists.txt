include (${gazebo_cmake_dir}/GazeboUtils.cmake)


SET (sources Referee.cc
             Socket.cc
             OldRefBoxProtocol.cc

) 

SET (headers Referee.hh
             Socket.hh
             IRefBoxProtocol.hh
             OldRefBoxProtocol.hh
)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
