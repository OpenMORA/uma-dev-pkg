PROJECT(Kinect_v1)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui hwdrivers maps graphslam opengl )
FIND_PACKAGE(OpenCV REQUIRED )


INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CKinect_v1_App.cpp
	CKinect_v1_App.h
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${OpenCV_LIBS})

