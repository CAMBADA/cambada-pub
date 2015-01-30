ADD_SUBDIRECTORY( filters )
ADD_SUBDIRECTORY( localization )

SET( integrator_SRC	
	ObstacleHandler
	ObstaclePositionKalman
	Filter
	Localization
	IntegratePlayer
	IntegrateBall
	Integrator
)

ADD_LIBRARY( integrator ${integrator_SRC} )
set_target_properties( integrator PROPERTIES COMPILE_FLAGS "-fPIC" )
ADD_DEPENDENCIES( integrator util filters localization )
