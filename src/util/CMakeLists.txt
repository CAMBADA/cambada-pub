
# Create cambada.conf.cxx/hxx
add_custom_command(
	OUTPUT cambada.conf.cxx
	OUTPUT cambada.conf.hxx
	COMMAND ${XSD_EXECUTABLE}
	ARGS cxx-tree --generate-serialization ${CAMBADA_CONFIG_DIR}/cambada.conf.xsd
	VERBATIM
)

# Create SetPieces.cxx/hxx
add_custom_command(
	OUTPUT SetPieces.cxx
	OUTPUT SetPieces.hxx
	COMMAND ${XSD_EXECUTABLE}
	ARGS cxx-tree --generate-serialization ${CAMBADA_CONFIG_DIR}/SetPieces.xsd
	VERBATIM
)

# sources for utils library
SET( util_SRC
	cambada.conf.cxx
	SetPieces.cxx
	PID.cpp
	Clock.cpp
	ConfigXML.cpp
	LinRegression.cpp
	Param.cpp
	SharedTimer.cpp
	SlidingWindow.cpp
	Timer.cpp
	KickerConf.cpp
	HeightMap
	ClippedRamp
	
	# Utilities for WorldState
	EgoMotionEstimator
	BallPositionKalman
	BallPositionParticle
	VelocityRegression
	LinearRegression
	StuckRobotSensor
)

ADD_LIBRARY( util ${util_SRC} )
set_target_properties( util PROPERTIES COMPILE_FLAGS "-fPIC" )

