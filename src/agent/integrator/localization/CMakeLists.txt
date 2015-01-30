SET( localization_SRC
	UseCompass
)

ADD_LIBRARY( localization ${localization_SRC} )
set_target_properties( localization PROPERTIES COMPILE_FLAGS "-fPIC" )
ADD_DEPENDENCIES( localization util )
