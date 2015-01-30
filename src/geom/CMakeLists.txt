
SET( geom_SRC
	Angle
	Vec
	geometry
	Frame2D
)

ADD_LIBRARY( geom ${geom_SRC} )
set_target_properties( geom PROPERTIES COMPILE_FLAGS "-fPIC" )
