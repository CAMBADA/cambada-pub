
SET( pman_SRC # this is a variable
	pman.c
#	pman-master-tester.cpp
#	pman-slave-tester.cpp
	sem_utils.c
)

ADD_LIBRARY( pman ${pman_SRC} )
TARGET_LINK_LIBRARIES( pman util )
set_target_properties( pman PROPERTIES COMPILE_FLAGS "-fPIC" )
