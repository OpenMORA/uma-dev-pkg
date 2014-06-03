PROJECT(VirtualCam_Desktop)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/modules/")

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED)


# Find DirectShow
FIND_PACKAGE(DirectShow) 

if (${DIRECTSHOW_BASECLASS_DIR} STREQUAL  "DIRECTSHOW_BASECLASS_DIR-NOTFOUND")
	message ("Download and install the Windows SDK and compile the DirectShow baseclasses")
endif (${DIRECTSHOW_BASECLASS_DIR} STREQUAL  "DIRECTSHOW_BASECLASS_DIR-NOTFOUND")

# Find compiled strambasd.lib and strmbase.lib to make sure the baseclasses were built
if(	NOT EXISTS "${DIRECTSHOW_BASECLASS_DIR}/Debug/strmbasd.lib" )
	message( FATAL_ERROR "Please build the Directshow baseclasses (debug), located in ${DIRECTSHOW_BASECLASS_DIR}")
endif()
if(	NOT EXISTS "${DIRECTSHOW_BASECLASS_DIR}/Release/strmbase.lib")
	message( FATAL_ERROR "Please build the Directshow baseclasses (release), located in ${DIRECTSHOW_BASECLASS_DIR}")
endif()

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

INCLUDE_DIRECTORIES( ${DIRECTSHOW_INCLUDE_DIRS})
LINK_DIRECTORIES( ${DIRECTSHOW_BASECLASS_DIR} )

# Add library target
ADD_LIBRARY( ${PROJECT_NAME} SHARED
	Dll.cpp
	Filters.def
	Filters.h
	Filters.cpp
	)
	
# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )
