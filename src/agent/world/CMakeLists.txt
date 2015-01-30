
SET( worldstate_SRC # this is a variable
	Field.cpp
	Obstacle.cpp
	Robot.cpp
	Sonar.cpp
	Ball.cpp
	VisionInfo.cpp
	LowLevelInfo.cpp
	WorldState.cpp
	
	Compass.cpp
	Zones.cpp
	CoachInfo.cpp
	SystemInfo.cpp
)

ADD_LIBRARY( worldstate ${worldstate_SRC} )
TARGET_LINK_LIBRARIES( worldstate util geom )
set_target_properties( worldstate PROPERTIES COMPILE_FLAGS "-fPIC" )
