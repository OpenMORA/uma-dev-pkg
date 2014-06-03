PROJECT(SkypeComm_WindowsAPI)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED)

#We need skype4com http://developer.skype.com/accessories/skype4com
find_path(SKYPE4COMDLL_PATH NAMES skype4com.dll HINTS $ENV{PROGRAMFILES}/Skype/Phone)

#Skype is not needed at compilation time but we will check for it to warn the user:
find_path(SKYPE_PATH NAMES skype.exe HINTS $ENV{PROGRAMFILES}/Skype/Phone)

if (${SKYPE4COMDLL_PATH} STREQUAL  "SKYPE4COMDLL_PATH-NOTFOUND")
	message (STATUS "Download skype4com.dll from http://developer.skype.com/accessories/skype4com")
endif (${SKYPE4COMDLL_PATH} STREQUAL  "SKYPE4COMDLL_PATH-NOTFOUND")

if (${SKYPE_PATH} STREQUAL  "SKYPE_PATH-NOTFOUND")
	message (STATUS "Warning: Skype not found. It is not needed at compilation time but it will be needed for the module to work")
endif (${SKYPE_PATH} STREQUAL  "SKYPE_PATH-NOTFOUND")

# Don't try to build the module if there are missing stuff:
IF (SKYPE4COMDLL_PATH)
	INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
	LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )
	INCLUDE_DIRECTORIES("${SKYPE4COMDLL_PATH}")

	# Add executable
	ADD_EXECUTABLE( ${PROJECT_NAME}
		main.cpp
		SkypeComm_winapi.h SkypeComm_winapi.cpp
		)

	# Link the executable
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS})
ENDIF (SKYPE4COMDLL_PATH)

