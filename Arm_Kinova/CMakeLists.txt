PROJECT(Arm_Kinova)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui hwdrivers opengl  ) # hmtslam hwdrivers ... any other sub-library
FIND_PACKAGE( PCL 1.2 REQUIRED  )

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS}  ${OpenCV_LIB_DIR} )

# Use OpenCV:
# Try to find OpenCV config file (NO_MODULE: Don't find a module, but OpenCVConfig.cmake):
FIND_PACKAGE(OpenCV NO_MODULE)
ADD_DEFINITIONS(/DOPENCV_VERSION=${OpenCV_VERSION})
SET(AUX_PATH "\"${OpenCV_DIR}\"")
ADD_DEFINITIONS(/DMORA_OPENCV_SOURCE_DIR=${AUX_PATH})


MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found through OpenCVConfig.cmake")


find_path(KINOVA_PATH NAMES Kinova.API.Jaco.DLL HINTS $ENV{PROGRAMFILES}/Kinova/Jacosoft/bin/release)
if (${KINOVA_PATH} STREQUAL  "KINOVA_PATH-NOTFOUND")
	message ("Install Kinova software")
endif (${KINOVA_PATH} STREQUAL  "KINOVA_PATH-NOTFOUND")

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CArm.h 
	CArm.cpp
	CKinova.h
	CKinova.cpp
	CCalibration.h
	CCalibration.cpp
	)

set_target_properties(${PROJECT_NAME} PROPERTIES VS_DOTNET_REFERENCES "${KINOVA_PATH}/Kinova.API.Jaco.DLL; ${KINOVA_PATH}/Kinova.DLL.Data.DLL; ${KINOVA_PATH}/Kinova.DLL.SafeGate.DLL; ${KINOVA_PATH}/Kinova.DLL.TestData.DLL; ${KINOVA_PATH}/Kinova.DLL.Tools.DLL; ${KINOVA_PATH}/Kinova.DLL.USBManager.DLL;")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/clr")


if(CMAKE_CXX_FLAGS MATCHES "/Zm1000 ") 
  string(REPLACE "/Zm1000" "/Zm453" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") 
  message(STATUS "CMAKE_CXX_FLAGS matches /Zm1000 before end of string -- replaced...") 
  message(STATUS "") 
endif() 

if(CMAKE_CXX_FLAGS MATCHES "/EHsc ") 
  string(REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") 
  message(STATUS "CMAKE_CXX_FLAGS matches /EHsc before end of string -- replaced...") 
  message(STATUS "") 
endif() 

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${OpenCV_LIBS})
