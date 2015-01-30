include (${gazebo_cmake_dir}/GazeboUtils.cmake)

#FIND_PACKAGE(Qt4 REQUIRED)

#set(QT_USE_QTOPENGL TRUE)
INCLUDE( ${QT_USE_FILE} )


#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


# .cxx sources
SET (visual_SRC_CXX 
  
  Visual.cc
  MainWindow.cc
  render/RenderWidget.cc
  render/drawstuff.cc
  
)

# files which need to be moc'd by Qt
SET (visual_MOC_SRCS
  
  #Visual.hh
  MainWindow.h
  render/RenderWidget.hh

)


SET (visual_UI
  MainWindow.ui
)


# build ui_XXX files from the XML-style .ui files
QT4_WRAP_UI(visual_SRC_CXX ${visual_UI})

# this moc's the above variable and appends to the cxx sources
QT4_WRAP_CPP(visual_SRC_CXX ${visual_MOC_SRCS})

add_library(gazebo_visual EXCLUDE_FROM_ALL SHARED ${visual_SRC_CXX})


IF (APPLE)
  set_target_properties(gazebo_visual PROPERTIES LINK_FLAGS "-flat_namespace -undefined suppress")

#ELSE (APPLE)

ENDIF (APPLE)

target_link_libraries( gazebo_visual ode ${QT_LIBRARIES})
#install ( TARGETS gazebo_visual DESTINATION ${BASE_DIR}/lib )
