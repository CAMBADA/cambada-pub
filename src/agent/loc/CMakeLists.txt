
SET( loc_SRC
	RobotPositionKalmanFilter
	VisualPositionOptimiser
	FieldLUT
	PositionKF
	CambadaLoc
)

ADD_LIBRARY( loc ${loc_SRC} )
set_target_properties( loc PROPERTIES COMPILE_FLAGS "-fPIC" )
ADD_DEPENDENCIES(loc util)