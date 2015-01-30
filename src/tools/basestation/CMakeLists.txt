# src/tools/basestation

include(${QT_USE_FILE})
include(${VTK_USE_FILE})

INCLUDE_DIRECTORIES(
	.
	FieldWidget
	FullInfoWindow
	FullWindow
	LogWidget
	MainWindow
	RefBoxWidget
	RobotInfoWidget
	RobotWidget
	UpdateWidget
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

SET( BASESTATION_SRCS 
	FieldWidget/FieldWidget.cpp
	FieldWidget/FieldWidget3D.cpp
	FullInfoWindow/FullInfoWindow.cpp
	FullWindow/FullWindow.cpp
	LogWidget/LogWidget.cpp
	MainWindow/MainWindow.cpp
	RefBoxWidget/RefBoxDialog.cpp
	RefBoxWidget/RefBoxWidget.cpp
#	RefBoxWidget/RefBoxXML.cpp
	RobotInfoWidget/RobotInfoWidget.cpp
	RobotWidget/RobotDialog.cpp
	RobotWidget/RobotWidget.cpp
	UpdateWidget/UpdateWidget.cpp
	main.cpp
)

# another list, this time it includes all header files that should be treated with moc
SET( BASESTATION_MOC_HDRS
	FieldWidget/FieldWidget.h
	FieldWidget/FieldWidget3D.h
	FullInfoWindow/FullInfoWindow.h
	FullWindow/FullWindow.h
	LogWidget/LogWidget.h
	MainWindow/MainWindow.h
	RefBoxWidget/RefBoxDialog.h
	RefBoxWidget/RefBoxWidget.h
#	RefBoxWidget/RefBoxXML.h
	RobotInfoWidget/RobotInfoWidget.h
	RobotWidget/RobotDialog.h
	RobotWidget/RobotWidget.h
	UpdateWidget/UpdateWidget.h
)

# some .ui files
SET( BASESTATION_UIS
	FullInfoWindow/FullInfoWindow_WG.ui
	FullWindow/FullWindow.ui
	LogWidget/LogWidget.ui
	MainWindow/MainWindow.ui
	RefBoxWidget/RefBoxDialog.ui
	RefBoxWidget/RefBoxWG.ui
	RobotInfoWidget/RobotInfoWG.ui
	RobotWidget/RobotDialog.ui
	RobotWidget/robotwidget.ui
)
  
# and finally an resource file
SET( BASESTATION_RCS
	basestation.qrc
)

QT4_WRAP_UI( BASESTATION_UI_HDRS ${BASESTATION_UIS} ) #Generate headers using QT UI files
QT4_WRAP_CPP( BASESTATION_MOC_SRCS ${BASESTATION_MOC_HDRS} ) #Generate CPP files from moh headers
QT4_ADD_RESOURCES( BASESTATION_RC_SRCS ${BASESTATION_RCS} )

ADD_EXECUTABLE( basestation EXCLUDE_FROM_ALL
 ${BASESTATION_SRCS} 
 ${BASESTATION_MOC_SRCS} 
 ${BASESTATION_RC_SRCS} 
 ${BASESTATION_UI_HDRS} 
)

TARGET_LINK_LIBRARIES( basestation 
 ${QT_LIBRARIES} 
# ${LIBXMLPP_LIBRARY} 
 util 
 rtdb 
 worldstate 
 geom 
 xerces-c 
 comm 
 pthread
 glut
 GL
 z
	QVTK
)
