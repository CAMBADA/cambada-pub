# src/

INCLUDE_DIRECTORIES(
	.
	
	# Workaround for hxx/cxx files
	${CAMBADA_CONFIG_DIR}
	${CMAKE_BINARY_DIR}/src/util
	
	# Libraries
	libs/pman
	libs/rtdb
	
	comm					# Comm
	util					# Utilities
	geom					# Geometry
	
	# Agent
	agent/
	agent/loc				# loc
	agent/world			        # worldstate
	agent/behaviours	   		# behaviours
	agent/roles                 		# roles
	agent/strategy              		# strategy

	# Integrator
	agent/integrator/localization		# subdirectory localization
	agent/integrator/filters		# subdirectory filters
	agent/integrator            		# integrator
)

# ---
# Libraries Sub-directories
ADD_SUBDIRECTORY( libs )		# Libraries
ADD_SUBDIRECTORY( util )		# Utilities
ADD_SUBDIRECTORY( geom )		# Geometry Library

# ---
# Processes
ADD_SUBDIRECTORY( comm )		# comm
ADD_SUBDIRECTORY( agent )		# cambadaAgent

# ---
# Tools
#ADD_SUBDIRECTORY( monitor )
ADD_SUBDIRECTORY( tools )
