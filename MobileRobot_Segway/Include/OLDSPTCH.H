#if !defined(_DISPATCH_)
#define _DISPATCH_

class CDtDataCopyThread;

#include <oldacfg.h>


#define OLDRV_GETSSCAPS                (DRV_USER+0)
#define OLDRV_SETDMAUSAGE              (DRV_USER+1)
#define OLDRV_GETDMAUSAGE              (DRV_USER+2)
#define OLDRV_SETTRIGGEREDSCANUSAGE    (DRV_USER+3)
#define OLDRV_GETTRIGGEREDSCANUSAGE    (DRV_USER+4)
#define OLDRV_SETCHANNELLISTSIZE       (DRV_USER+5)
#define OLDRV_GETCHANNELLISTSIZE       (DRV_USER+6)
#define OLDRV_SETCHANNELLISTENTRY      (DRV_USER+7)
#define OLDRV_GETCHANNELLISTENTRY      (DRV_USER+8)
#define OLDRV_SETGAINLISTENTRY         (DRV_USER+9)
#define OLDRV_GETGAINLISTENTRY         (DRV_USER+10)
#define OLDRV_SETCHANNELFILTER         (DRV_USER+11)
#define OLDRV_GETCHANNELFILTER         (DRV_USER+12)
#define OLDRV_SETRANGE                 (DRV_USER+13)
#define OLDRV_GETRANGE                 (DRV_USER+14)
#define OLDRV_SETCHANNELTYPE           (DRV_USER+15)
#define OLDRV_GETCHANNELTYPE           (DRV_USER+16)
#define OLDRV_SETENCODING              (DRV_USER+17)
#define OLDRV_GETENCODING              (DRV_USER+18)
#define OLDRV_SETTRIGGER               (DRV_USER+19)
#define OLDRV_GETTRIGGER               (DRV_USER+20)
#define OLDRV_SETRESOLUTION            (DRV_USER+21)
#define OLDRV_GETRESOLUTION            (DRV_USER+22)
#define OLDRV_SETCLOCKSOURCE           (DRV_USER+23)
#define OLDRV_GETCLOCKSOURCE           (DRV_USER+24)
#define OLDRV_SETCLOCKFREQUENCY        (DRV_USER+25)
#define OLDRV_GETCLOCKFREQUENCY        (DRV_USER+26)
#define OLDRV_SETRETRIGGERFREQUENCY    (DRV_USER+27)
#define OLDRV_GETRETRIGGERFREQUENCY    (DRV_USER+28)
#define OLDRV_SETEXTERNCLOCKDIVIDER    (DRV_USER+29)
#define OLDRV_GETEXTERNCLOCKDIVIDER    (DRV_USER+30)
#define OLDRV_SETGATETYPE              (DRV_USER+31)
#define OLDRV_GETGATETYPE              (DRV_USER+32)
#define OLDRV_SETPULSETYPE             (DRV_USER+33)
#define OLDRV_GETPULSETYPE             (DRV_USER+34)
#define OLDRV_SETPULSEWIDTH            (DRV_USER+35)
#define OLDRV_GETPULSEWIDTH            (DRV_USER+36)
#define OLDRV_SETCOUNTERMODE           (DRV_USER+37)
#define OLDRV_GETCOUNTERMODE           (DRV_USER+38)
#define OLDRV_SETCASCADEMODE           (DRV_USER+39)
#define OLDRV_GETCASCADEMODE           (DRV_USER+40)
#define OLDRV_SETWRAPMODE              (DRV_USER+41)
#define OLDRV_GETWRAPMODE              (DRV_USER+42)
#define OLDRV_SETDATAFLOW              (DRV_USER+43)
#define OLDRV_GETDATAFLOW              (DRV_USER+44)
#define OLDRV_SETWINDOWHANDLE          (DRV_USER+45)
#define OLDRV_GETWINDOWHANDLE          (DRV_USER+46)
#define OLDRV_SETNOTIFYPROC            (DRV_USER+47)
#define OLDRV_GETNOTIFYPROC            (DRV_USER+48)
#define OLDRV_SETCHANNELLISTINHIBIT    (DRV_USER+49)
#define OLDRV_GETCHANNELLISTINHIBIT    (DRV_USER+50)
#define OLDRV_SETDIGITALIOLISTENTRY    (DRV_USER+51) 
#define OLDRV_GETDIGITALIOLISTENTRY    (DRV_USER+52) 
#define OLDRV_SETSYNCHRONOUSDIOUSAGE   (DRV_USER+53) 
#define OLDRV_GETSYNCHRONOUSDIOUSAGE   (DRV_USER+54) 
#define OLDRV_SETTRANSFERSIZE          (DRV_USER+55) 
#define OLDRV_GETTRANSFERSIZE          (DRV_USER+56) 
#define OLDRV_SETTRANSFERCOUNT         (DRV_USER+57) 
#define OLDRV_GETTRANSFERCOUNT         (DRV_USER+58) 
#define OLDRV_SETCHANNELRANGE          (DRV_USER+59)
#define OLDRV_GETCHANNELRANGE          (DRV_USER+60)
#define OLDRV_PUTBUFFER                (DRV_USER+61)
#define OLDRV_GETBUFFER                (DRV_USER+62)
#define OLDRV_GETQUEUESIZES            (DRV_USER+63)
#define OLDRV_FLUSHBUFFERS             (DRV_USER+64)
#define OLDRV_SETRETRIGGERMODE         (DRV_USER+65)
#define OLDRV_GETRETRIGGERMODE         (DRV_USER+66)

//  new for v2.5 
#define OLDRV_SETPRETRIGGERSOURCE       (DRV_USER+67)
#define OLDRV_GETPRETRIGGERSOURCE       (DRV_USER+68)
#define OLDRV_SETMULTISCANCOUNT         (DRV_USER+69)
#define OLDRV_GETMULTISCANCOUNT         (DRV_USER+70)

#define OLDRV_SETRETRIGGERSOURCE        (DRV_USER+71)
#define OLDRV_GETRETRIGGERSOURCE        (DRV_USER+72)

#define OLDRV_SETSPECIALVPICONFIGMODE   (DRV_USER+73)
#define OLDRV_GETSPECIALVPICONFIGMODE   (DRV_USER+74)

#define OLDRV_SETSTARTEDGE				(DRV_USER+75)
#define OLDRV_GETSTARTEDGE				(DRV_USER+76)
#define OLDRV_SETSTOPEDGE				(DRV_USER+77)
#define OLDRV_GETSTOPEDGE				(DRV_USER+78)
#define OLDRV_SETQUADRATURE             (DRV_USER+79)

#define OLDRV_NULL                     (DRV_USER+99)
#define OLDRV_READEVENTS               (DRV_USER+100)
#define OLDRV_SETSINGLEVALUE           (DRV_USER+101)
#define OLDRV_GETSINGLEVALUE           (DRV_USER+102)
#define OLDRV_CONFIG                   (DRV_USER+103)
#define OLDRV_START                    (DRV_USER+104)
#define OLDRV_STOP                     (DRV_USER+105)
#define OLDRV_PAUSE                    (DRV_USER+106)
#define OLDRV_CONTINUE                 (DRV_USER+107)
#define OLDRV_RESET                    (DRV_USER+108)
#define OLDRV_ABORT                    (DRV_USER+109)
#define OLDRV_BURSTDTCONNECT           (DRV_USER+110)

#define OLDRV_PRESTART                 (DRV_USER+111)
#define OLDRV_SIMULSTART               (DRV_USER+112)
#define OLDRV_ALLOCATE                 (DRV_USER+113)
#define OLDRV_DEALLOCATE               (DRV_USER+114)
#define OLDRV_FLUSHINPROCESS           (DRV_USER+115)

#define OLDRV_WRITEREG                 (DRV_USER+116)
#define OLDRV_READREG                  (DRV_USER+117)

#define OLDRV_GETSINGLEVALUES          (DRV_USER+118)


#define OLDRVMSGMIN     (OLDRV_GETSSCAPS)
#define OLDRVMSGMAX     (OLDRV_SETQUADRATURE)
#define OLDRVMSGCOUNT   (OLDRVMSGMAX - OLDRVMSGMIN + 1)

#define OLDRVUSERMIN    (OLDRV_READEVENTS)
#define OLDRVUSERMAX    (OLDRV_GETSINGLEVALUES)
#define OLDRVUSERCOUNT  (OLDRVUSERMAX - OLDRVUSERMIN + 1)



typedef struct 
{
    union 
    {
        struct 
        {                
            UINT  uiCap;
            UINT  uiIndex;
            DBL   dfCap;
        } Caps;                // GETSSCAPS
        
        UINT  uiDmaUse;         // SET/GETDMAUSAGE
        BOOL  fTrigUse;         // SET/GETTRIGGEREDSCANUSAGE
        UINT  uiChanListSize;   // SET/GETCHANNELLISTSIZE
        
        struct 
        {                
            UINT uiEntry;
            UINT uiChan;
        } ChannelListEntry;     // SET/GETCHANNELLISTENTRY
        
        struct 
        {          
            UINT uiEntry;
            DBL  dfGain;
        } GainListEntry;        // SET/GETGAINLISTENTRY
        
        struct 
        {    
            UINT  uiChannel;
            DBL   dfMin;
            DBL   dfMax;
        } Range;                // SET/GETRANGE, SET/GETCHANNELRANGE
        
        struct 
        {    
            UINT  uiChan;
            DBL   dfFreq;
        } ChannelFilter;        // SET/GETCHANNELFILTER
        
        UINT  uiChannelType;    // SET/GETCHANNELTYPE
        UINT  uiEncoding;       // SET/GETENCODING
        UINT  uiTrigger;        // SET/GETTRIGGER

        UINT  uiPreTrigger;     // SET/GETPRETRIGGER        SEB 12/27
        UINT  uiCount;          // SET/GETMULTISCANCOUNT    SEB 12/27
        
        UINT  uiReTrigger;      // SET/GETRETRIGGER         SEB 02/24
        
        UINT  uiResolution;     // SET/GETRESOLUTION
        UINT  uiClockSource;    // SET/GETCLOCKSOURCE
        DBL   dfFrequency;      // SET/GETCLOCKFREQUENCY ,SET/GETRETRIGGERFREQUENCY
        ULNG  ulDivider;        // SET/GETEXTERNCLOCKDIVIDER    
        UINT  uiGate;           // SET/GETGATETYPE
        UINT  uiPulse;          // SET/GETPULSETYPE
        UINT  uiPercent;        // SET/GETPULSEWIDTH
        UINT  uiMode;           // SET/GETCOUNTERMODE, SET/GETCASCADEMODE, GET/SETWRAPMODE
        ULNG  ulEvents;         // READEVENTS
		UINT  uiStartEdge;		// SET/GETSTARTEDGE
		UINT  uiStopEdge;		// SET/GETSTARTEDGE

        UINT  uiRetriggerMode;  // SET/GETRETRIGGERMODE     6/11/96  mak

        UINT uiElement;         //  olDaSimultaneousPrestart
        UINT OlSs;              //  olDaSimultaneousPrestart
        
        struct 
        {                  
            HWND   hWnd;
            WPARAM wParam;
            LPARAM lParam;
        } WindowNotification;   // SET/GETWINDOWHANDLE
        
        struct 
        {        
            UINT uiQueue;
            UINT uiSize;
        } QueueSize;            // GETQUEUESIZES
        
        UINT  uiFlow;           // SET/GETDATAFLOW
        HBUF  hBuf;             // SET/GETBUFFER
        struct 
        {             
            UINT  uiChannel;
            LNG   lValue;
            BOOL  bAutoRange;
            DBL   dfGain;
            DBL   dfValueVolts; 
        } SingleValue;          // SET/GETSINGLEVALUE

        struct 
        {             
            DBL   dfGain;
            LNG   lValues[32];
        } SingleValues;         // SET/GETSINGLEVALUES

        struct 
        {       
            UINT uiEntry;
            BOOL fInhibit;
        } InhibitEntry;         // SET/GETCHANNELLISTINHIBIT
        
        struct 
        {    
            UINT uiEntry;
            UINT uiValue;
        } IOListEntry;          // SET/GETDIGITALIOLISTENTRY
        
        BOOL fSyncDigitalIO;        // SET/GETSYNCHRONOUSDIOUSAGE
        BOOL bSpecialVpiConfigMode; // SET/GETSETSPECIALVPICONFIGMODE
        
        struct 
        {     
            HBUF  hBuf;
            ULNG  ulSamples;
        } FlushBuffer;          // FLUSHINPROCESS
        
        struct 
        {    
            OLNOTIFYPROC pfnFunc;
            WPARAM      wParam;
            LPARAM      lParam;
        } NotifyProc;           // SET/GETNOTIFYPROC

        struct
        {
            ULNG  ulData;           // Read/WriteReg            SEB 02/24
            ULNG  ulOffset;         // Read/WriteReg            SEB 02/24
            UINT  uiSize;           // Read/WriteReg            SEB 02/24
        } ReadWriteReg;         // Read/Write Register

		struct
		{
			 UINT ClockPreScale; 
			 BOOL X4Scaling;
			 UINT IndexMode;
		} SetQuad;
    };
}DISP_PKT;

typedef DISP_PKT *PDISP_PKT;
#define LPDISP_PKT PDISP_PKT

 
typedef struct personality_tag
   {
   // these guys are filled in pu the personality library
   // for the appropriate board and device
   PDBL     pdfGains;
   PDBL*    ppdfMinRanges;
   PDBL*    ppdfMaxRanges;
   PDBL     pdfFilters;
   PDBL     pdfResolutions;
   PUINT    puiCaps;
   PDBL     pdfCaps;
   }
PERSONALITY, *PPERSONALITY;



// located in personality module

#ifdef STRICT
typedef BOOL (WINAPI* BOARDINFOPROCEX)(UINT uiType, CHAR *szUserBoardName, PTSTR* ppszBoardName, PUINT* ppCounts);
#else
typedef FARPROC BOARDINFOPROCEX;
#endif

// name to use for GetProcAddress()
#define BOARDINFOPROCNAMEEX  TEXT("_BoardInfoProcEx@16")

// located in personality module

#ifdef STRICT
 typedef BOOL (WINAPI* PERSONALITYPROCEX)(UINT uiType, CHAR *szBoardName, OLSS ss, UINT el, PPERSONALITY pPersonality);
#else
 typedef FARPROC PERSONALITYPROCEX;
#endif

// name to use for GetProcAddress()
#define PERSONALITYPROCNAMEEX  TEXT("_PersonalityProcEx@20")


 /* 
   Provides conversion from a physical element within SsType to a target virtual Channel.
   The subsystem that owns the channel is owner of CHANNELMAP struct.
*/
typedef struct _channelMap
{
	int SsType;			    // Source subsystem type
	int LogicalChannel;		// Source subsystem channel number
	int logicalChannelPart;	// Source subsystem channel part if logical channel bitness is larger
							// than the physical channel bitness 
	int PhysicalChannel;	// Target physical channel within the subsystem owning this channel map

} CHANNELMAP, *PCHANNELMAP;

/* 
  Provides a list of channel map items
*/
typedef struct _channelMapList
{
	int Count;				// Number of channel map items
	PCHANNELMAP pItems;		// A pointer to an array containing the <Count> items

} CHANNELMAP_LIST, *PCHANNELMAP_LIST;

#ifdef STRICT
 typedef BOOL (WINAPI* GETCHANNELMAPPING)(UINT uiType, CHAR *szBoardName, OLSS ss, UINT el, PCHANNELMAP_LIST pChannelMapList);
#else
 typedef FARPROC GETCHANNELMAPPING;
#endif

// name to use for GetProcAddress()
#define GETCHANNELMAPPINGNAME  TEXT("_GetChannelMapping@20")

 
 // An HDEV as returned from oldaInitialize actually points to one of these

typedef struct boarddata_tag 
{
   PVOID       pSelf;
   HINSTANCE   hPersonalityDll;             // from LoadLibrary call
   PERSONALITYPROCEX pfnPersonalityProcEx;
   UINT        uiUseCount;                  // cannot be freed unless this is zero
   UINT        uiBoardType;                 // from "Type" in registry
   PUINT       pDevCounts;                  // filled in by BoardInfoProc(), storage in Personality DLL
   TCHAR       szDriverName[64];			
   TCHAR       szUserName[64];              // Open Layers Board Name    
   TCHAR       szDeviceInterfacePath[128];  // Base Name of Symbolic Links to Driver 
   PTSTR       pszDeviceName;               // Device Name ( i.e. Model Name ), storage in personality DLL 
   HANDLE      hDrv;                        // returned by CreateFile() - for driver IOCTLs not subsystem-specific
   DWORD       StartSleepTimeMSecs;			// read from registry or 0
   DWORD       StopSleepTimeMSecs;			// read from registry or 0
   DWORD       NonBlockingStop;			    // Read from registry or 0
   DWORD       MessageOnAbort;			    // Read from registry or 0
   DWORD       AbortSleepTimeMSecs;		    // Read from registry or 0
   DWORD       CopyThreadPriority;		    // Read from registry or 0
} 
BOARD_DATA, *PBOARD_DATA;


// A DASS as returned by oldaGetDASS actually points to one of these

typedef struct device_data_tag 
{
   PVOID        pSelf;                 // self reference for reality checks
   HANDLE       hDevice;               // cookie returned by CreateFile()
   PBOARD_DATA  pBoard;                // pointer to board information
   PERSONALITY  Personality;           // embedded structure containing device personality information
   TCHAR        szSubSystemName[64];   // subsystem name 
   OLSS         ss;                    // subsystem type
   UINT         el;                    // subsystem element
   PDEVICE_CFG  pCfgData;              // must be filled with default settings
   HLIST        hReady;                // Ready Queue, must be allocated
   HLIST        hDone;                 // Done Queue, must be allocated
   HANDLE       hThreadId;             // real time thread for copying data
   HANDLE       hThreadEvent;
   HANDLE       hThreadAckEvent;
   UINT         uiMsg;    
   CRITICAL_SECTION  csDevLock;

   CDtDataCopyThread* pCopyThreadObject;
   
   HWND         hWndNotify;
   WPARAM       wWndNotifyData;
   LPARAM       lWndNotifyData;
   OLNOTIFYPROC pfnNotify;
   WPARAM       wProcNotifyData;
   LPARAM       lProcNotifyData;
   HBUF         hFlushBuf;
   ULONG        ulFlushSamples;

} DEVICE_DATA, *PDEVICE_DATA;  
 
#ifdef _DEBUG
#define Oops(string) OutputDebugString(TEXT(string))
#else
#define Oops(string)
#endif


#define AreLargeIntsEqual(x,y) (x.QuadPart == y.QuadPart)

#define LARGEST_32BIT_VALUE  4294967295.0

#define CONVERT_DBL_TO_LARGE_INT(dbl,large_int) { large_int.HighPart =((LONG)(int)dbl); \
                                                  large_int.LowPart = (LONG)((dbl - (int)dbl) * LARGEST_32BIT_VALUE); \
                                                }
#define CONVERT_LARGE_INT_TO_DBL(large_int,dbl) { dbl = (double)large_int.HighPart + \
                                                     ((ULONG)large_int.LowPart / LARGEST_32BIT_VALUE); \
                                                }
#define CONVERT_LARGE_INT_TO_DBL_RANGE(large_int,dbl) { dbl = (double)large_int.HighPart + \
                                                     ((LONG)large_int.LowPart / LARGEST_32BIT_VALUE); \
                                                }
        // JSJ Modified to Support NEG DECIMAL LARGE INTS

#define GetMemory(size) (PVOID)GlobalAlloc(GPTR,size)
#define ReleaseMemory(ptr) GlobalFree((HGLOBAL)ptr)
                                                
#endif
