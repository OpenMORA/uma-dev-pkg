PROJECT(MobileRobot_Segway)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui  hwdrivers ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  "Include")
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS})
#"${OpenMORA_SOURCE_DIR}/modules/MobileRobot_Segway/Lib/"
FILE(GLOB DA_LIBS Lib/*.lib)
# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CSegwayApp.h CSegwayApp.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${DA_LIBS})