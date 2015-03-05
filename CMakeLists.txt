CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( Cambada )

SET( V_MAJOR 1 )
SET( V_MINOR 0 )
SET( V_PATCH 0 )

# ---
MESSAGE ("  ____    _    __  __ ____    _    ____    _    ")
MESSAGE (" / ___|  / \\  |  \\/  | __ )  / \\  |  _ \\  / \\   ")
MESSAGE ("| |     / _ \\ | |\\/| |  _ \\ / _ \\ | | | |/ _ \\  ")
MESSAGE ("| |___ / ___ \\| |  | | |_) / ___ \\| |_| / ___ \\ ")
MESSAGE (" \\____/_/   \\_\\_|  |_|____/_/   \\_\\____/_/   \\_\\")
message ("                                   version ${V_MAJOR}.${V_MINOR}.${V_PATCH}")
# ---

# http://www.cmake.org/cmake/help/cmake-2-8-docs.html#section_Policies
IF(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# COMPILER FLAGS
SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W" )
SET( CMAKE_C_FLAGS_RELEASE "-O3" )
SET( CMAKE_C_FLAGS_DEBUG "-g3" )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall" )
SET( CMAKE_CXX_FLAGS_RELEASE "-O3" )
SET( CMAKE_CXX_FLAGS_DEBUG "-g3" )

# Set the top directory of the source code (the first CMakeLists of the project, this one actually)
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CAMBADA_CONFIG_DIR ${BASE_DIR}/config )
SET( CAMBADA_SRC_DIR ${BASE_DIR}/src )

# Set output directories for libraries and executables
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

LINK_DIRECTORIES( ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )

# Define where to find modules
SET( CMAKE_MODULE_PATH ${CMAKE_MODULES_PATH} ${BASE_DIR}/CMakeModules )

MESSAGE ("\n==> Finding 3rd Party Packages")
FIND_PACKAGE( XSD       REQUIRED )
FIND_PACKAGE( XERCESC   REQUIRED )
FIND_PACKAGE( EIGEN3	REQUIRED )
#FIND_PACKAGE( OpenCV	REQUIRED )
#FIND_PACKAGE( SDL	REQUIRED )
#FIND_PACKAGE( LibXmlpp	REQUIRED )
FIND_PACKAGE( VTK	REQUIRED )

# by default only QtCore and QtGui modules are enabled
# other modules must be enabled like this:
#SET( QT_USE_QT3SUPPORT TRUE )   
SET( QT_USE_QTXML TRUE )
FIND_PACKAGE( Qt4 		REQUIRED COMPONENTS QtCore QtNetwork QtOpenGL QtGui) 
INCLUDE( ${QT_USE_FILE} ) # adds some useful macros and variables, that contains a path to CMake script
SET( QT_WRAP_CPP	TRUE )
SET( QT_USE_QTXML	TRUE )

# Make include directories available for all..
INCLUDE_DIRECTORIES( 
	${XSD_INCLUDE_DIRS}
	${XERCESC_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
#	${OpenCV_INCLUDE_DIR}
#	${SDL_INCLUDE_DIR}
#	${LIBXMLPP_INCLUDE_DIR}
	/usr/local/include/libtcod-1.5.1 # libtcod include files
)

# Add sources directory
ADD_SUBDIRECTORY( ${CAMBADA_SRC_DIR} )

# ---
MESSAGE ("\n==> Building Options")
MESSAGE (STATUS "Building as [${CMAKE_BUILD_TYPE}]")
MESSAGE ("\n==> Building Instructions")
MESSAGE (STATUS "Compile CAMBADA project using: make")
MESSAGE (STATUS "Generate documentation using: make doc")
MESSAGE ("\n")

# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)



