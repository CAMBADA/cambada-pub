
ADD_LIBRARY( comm MulticastSocket.cpp )
SET_TARGET_PROPERTIES( comm PROPERTIES COMPILE_FLAGS "-fPIC" )


SET( comm_SRC
	multicast.cpp
	comm.cpp
)

set ( comm_OBJ cambadaComm )
ADD_EXECUTABLE ( ${comm_OBJ} ${comm_SRC} )
TARGET_LINK_LIBRARIES( ${comm_OBJ} rtdb pthread comm util )
SET_TARGET_PROPERTIES( ${comm_OBJ} PROPERTIES OUTPUT_NAME comm )