SET( filters_SRC
	KalmanFilter
	ParticleFilter
)

ADD_LIBRARY( filters ${filters_SRC} )
set_target_properties( filters PROPERTIES COMPILE_FLAGS "-fPIC" )
ADD_DEPENDENCIES( filters util )
