PROJECT(CamControl_ImagingSource)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED hwdrivers )

SET( IC_IMAGING_CONTROL_ROOT_DIR "" CACHE PATH "Path to IC Imaging Control")

IF(NOT EXISTS ${IC_IMAGING_CONTROL_ROOT_DIR})
	MESSAGE(FATAL_ERROR "The directory 'IC_IMAGING_CONTROL_ROOT_DIR' does not exist. Provide the correct path. (Install the IC Imaging Control C++ Class library)")
ENDIF(NOT EXISTS ${IC_IMAGING_CONTROL_ROOT_DIR})

IF(NOT EXISTS ${IC_IMAGING_CONTROL_ROOT_DIR}/classlib/include/tisudshl.h)
	MESSAGE(FATAL_ERROR "'IC_IMAGING_CONTROL_ROOT_DIR/classlib/include/tisudshl.h' does not exist. Provide the correct path.")
ENDIF(NOT EXISTS ${IC_IMAGING_CONTROL_ROOT_DIR}/classlib/include/tisudshl.h)

 if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
	set( IC_IMAGING_CONTROL_LIB_DIR "${IC_IMAGING_CONTROL_ROOT_DIR}/classlib/win64/" CACHE PATH "IC Imaging Control C++ Libraries")
 else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( IC_IMAGING_CONTROL_LIB_DIR "${IC_IMAGING_CONTROL_ROOT_DIR}/classlib/win32/" CACHE PATH "IC Imaging Control C++ Libraries")
 endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
 
IF(NOT EXISTS ${IC_IMAGING_CONTROL_LIB_DIR})
	MESSAGE(FATAL_ERROR "The directory 'IC_IMAGING_CONTROL_LIB_DIR' does not exist. Provide the correct path. (Install the IC Imaging Control C++ Class library)")
ENDIF(NOT EXISTS ${IC_IMAGING_CONTROL_LIB_DIR})
 
INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
INCLUDE_DIRECTORIES("${IC_IMAGING_CONTROL_ROOT_DIR}/classlib/include")

LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} ${IC_IMAGING_CONTROL_LIB_DIR})


# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CamControl_ImagingSource.cpp
	CamControl_ImagingSource.h
	SimplePropertyAccess.cpp
	SimplePropertyAccess.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS})