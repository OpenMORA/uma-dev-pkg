PROJECT(pFaceRecognizer)

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE( MRPT REQUIRED base obs gui opengl hwdrivers )
FIND_PACKAGE( OpenCV REQUIRED )

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Declare the target (an executable)
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CFaceRecognizerApp.cpp
	CFaceRecognizerApp.h
	)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${OpenCV_LIBS})

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


