// File: TunstallSoprano.h
//------------------------------------------------------------------------------
// TUNSTALL HEALTHCARE (c) Copyright 2008
//
// Athena
//
//------------------------------------------------------------------------------
//
// Notes: Header file provides the interface to the Tunstall Soprano Library
//        for the Social Alarm Gateway (SAG)
//
//------------------------------------------------------------------------------
//
// History :
//
// Date        Who  Modification
// 03 Sep2008  NJH  Initial version
//
//------------------------------------------------------------------------------

#ifndef __TUNSTALL_SOPRANO_H
#define __TUNSTALL_SOPRANO_H

//------------------------------------------------------------------------------
// Library Externally Exported Functions
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#define STDCALL __stdcall
#else
#define DLLEXPORT
#define STDCALL
#endif

/* ------- o0 TUNSTALL SOPRANO API ATHENA FUNCTIONS 0o ------- */
// Make connection to Athena unit
DLLEXPORT int  STDCALL connectAthena(int comPortNum);
// Disconnect from Athena unit
DLLEXPORT int  STDCALL disconnectAthena(void);
// Acquire event from Athena, returned in provided character buffer
DLLEXPORT void STDCALL getEvent(unsigned char *eventString);
// Activate integral Athena relay to unlock door
DLLEXPORT int  STDCALL unlockDoor(int relayActionUnlock);
// Activate integral Athena relay to lock door
DLLEXPORT int  STDCALL lockDoor(int relayActionLock);
// Request Athena unit to dial supplied number
DLLEXPORT int  STDCALL dialNumber(unsigned char *number, unsigned int mode);
// End a call initiated using the dialNumber() function
DLLEXPORT int  STDCALL endCall(void);
// Request Athena alarm call with passed buffer attributes in alarmDetails
DLLEXPORT int  STDCALL raiseAlarm(unsigned char *alarmDetails);

/* ------- o0 TUNSTALL SOPRANO API X10 MY LIFE ADAPTER FUNCTIONS 0o ------- */
// Make connection to MyLife adapter
DLLEXPORT int  STDCALL connectMyLife(int comPortNum);
// Disconnect from MyLife adapter
DLLEXPORT int  STDCALL disconnectMyLife(void);
// Send command to MyLife X10 adapter
DLLEXPORT int  STDCALL X10Command(int X10House, int X10Unit, int X10Action);

#ifdef __cplusplus
}
#endif

#endif
