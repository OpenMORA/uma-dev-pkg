#if !defined(_OLDACFG_)
#define _OLDACFG_

#include "OlDaDefs.h"

typedef unsigned char BYTE, *PBYTE;
typedef unsigned short WORD, *PWORD;
typedef unsigned int UINT, *PUINT;
typedef unsigned short USHRT, *PUSHRT;

typedef enum ss_states_tag
   {
   OL_STATE_DORMANT = 1000,
   OL_STATE_INITIALIZED,
   OL_STATE_SV_READY,
   OL_STATE_BURST_READY,
   OL_STATE_READY,
   OL_STATE_RUNNING,
   OL_STATE_STOPPING,
   OL_STATE_PAUSED,
   OL_STATE_PRESTARTED,
   OL_STATE_IOCOMPLETE
   } 
SS_STATES;

#define OLDA_FM_USE_LI 0
#define OLDA_FM_USE_UL 1

#define MAX_LEGACY_PER_CHANNEL_FILTERS 2 //16

typedef struct channel_info
{
   USHORT         usChannelNumber;         // Entry Channel Number
   LARGE_INTEGER  liGain;                  // Entry Gain Factor
   BOOLEAN        bInhibit;                // Entry Inhibit Enable
   USHORT         usSynchDioValue;         // Entry synched Digital I/O Value
} CHANNEL_ENTRY_INFO, *PCHANNEL_ENTRY_INFO;


typedef struct ___device_cfg_tag 
{
   // The following 4 fields must be first.  This structure is part of a union and 
   // these fields are accessed thru another representation of that union when this 
   // structure is being used in IOCTL to kernel mode drivers.
   //
   OLSS           ssType;                
   UINT           ssElement;
   SS_STATES      State;
   ECODE          ssOlErrorStatus;

   // General Stuff
   //
   BOOLEAN        bUseDma;               // Use DMA or not
   BOOLEAN        bGapFree;              // insist on gap free or not
   BOOLEAN        bTrigScan;             // use of triggerd scan mode
   BOOLEAN        bSynchDio;             // use of synchronous digital I/O
   USHORT         usChannelListSize;     // size of Channel Gain List
   USHORT         usNumChannels;         // set by user mode code according to
   USHORT         usChannelType;         // Channel Type setting 
   USHORT         usEncoding;            // Digital Encoding setting
   USHORT         usResolution;          // resolution setting
   USHORT         usWrap;                // buffer wrap mode setting
   USHORT         usDataFlow;            // data flow mode

   // Clock stuff
   //
   USHORT         usClockSource;         // clock source setting
   USHORT         usClockFreqMode;       // 0-use li, 1-use ul
   ULONG          ulBase;                // base frequency
   ULONG          ulExternClockDivider;  // external clock divider setting
   ULONG          ulClockFrequency;      // really a float stored here
   LARGE_INTEGER  liClockFrequency;      // internal clock frequency setting

   // Trigger stuff
   //
   USHORT         usTrigger;             // Trigger Source setting
   USHORT         usRetriggerMode;       // retrigger mode
   USHORT         usRetriggerFreqMode;   // 0-use li, 1-use ul
   USHORT         usPreTrigger;          // Trigger Source setting     
   USHORT         usCount;               // # of counts through CGL per Trigger 
   USHORT         usRetrigger;           // ReTrigger Source setting    
   ULONG          ulRetriggerFrequency;  // really a float stored here
   LARGE_INTEGER  liRetriggerFrequency;  // internal retrigger frequency setting

   
   // Ranges on per channel basis... used only on a few boards (DACS)
   //
   LARGE_INTEGER  liRangeMin[2];         // minimum range setting.  
   LARGE_INTEGER  liRangeMax[2];         // maximum range ng

   // liFilter is no longer used so we are replacing it with UnusedBytes
   // so that we can use these bytes for new features.  
   LARGE_INTEGER  liFilter[MAX_LEGACY_PER_CHANNEL_FILTERS]; 
   CHAR			  UnusedBytes[106];
   USHORT		  ClockPreScale;		// For Quad Encoder
   BOOLEAN		  ALeadsB;				// For Quad Encoder
   BOOLEAN		  X4Scaling;			// For Quad Encoder
   USHORT		  usIndex;				// For Quad Encoder

// LARGE_INTEGER  liFilter[MAX_LEGACY_PER_CHANNEL_FILTERS];  

   // Counter timer stuff
   //
   USHORT         usGateType;            // counter/timer gate type
   USHORT         usPulseType;           // counter/timer output pulse type
   USHORT         usPulseWidth;          // counter/timer output pulse width
   USHORT         usCounterMode;         // counter/timer operating mode
   USHORT         usEventSource;         // RESERVED
   USHORT         usCascadeMode;         // counter/timer cascading mode

   // Misc stuff
   //
   HWND           hMfWndNotify;          // Window to notify for Measure Frequency
   DBL            dfCount_Duration;      // Length of time to measure for
   UINT           uiResolution;

   // Following flag instructs driver to recalc freqs only and not touch the hardware.
   // This is to accomodate VPI frequency interrogations which can be extremely 
   // time consuming.
   //
   BOOLEAN        bSpecialVpiConfigMode;
   
   // Following field can be used by Device Driver to save a context 
   // between successive Configs if it needs to...user mode guranteed to not touch it.
   ULONG          DdContext;       

   // Specifies the signal edges to be used with the counter Measure mode
   USHORT         usStartEdge;
   USHORT		  usStopEdge;

   // Following Reserved fields for forward compatibility in case we add a new feature 
   // to the API or for a specific driver and additional config info needs to passed.
   // This way we have the option of re-use the existing interface and don't have to resort 
   // to an interface change for small feature changes. 
   //
   // Since Open Layers DLL's and the drivers are now built together and all ship together 
   // on the same CD, the dll-driver interface should be a little more manageable than 
   // in the past, but allowing some flexibility here will help regarding upgrading of 
   // individual releases between OmniCD maintenance releases.  In any event, the latest 
   // stuff we keep on the Web should still always have the DLL's and drivers bound together 
   // so nothing can go wrong in this regard. We aree bound to preserving the 
   // published Open Layers Interface for all eternity but having some flexibility in the
   // private interface between our dll's and drivers is a good thing...and if we always 
   // ship them together then we have the luxury of that flexibility.
   //
   ULONG          Reserved2;
   ULONG          Reserved3;

   // Note: Following single element array must always be last in this structure definition as 
   //       this structure will "grow" as needed to accomodate any given boards CGL caps
   //       and this structure will still be used to dereference.

   CHANNEL_ENTRY_INFO   ChannelListEntry[ 1 ];

} DEVICE_CFG, *PDEVICE_CFG;
            
#define IsSetOk(state) ((state) < OL_STATE_RUNNING)
#define IsFreeOk(state) (IsSetOk(state) && (state) > OL_STATE_DORMANT)
#define IsConfigOk(state) IsFreeOk(state)
#define IsConfigured(state) ((state) == OL_STATE_READY || (state) == OL_STATE_SV_READY)
#define IsRunning(state) ((state) > OL_STATE_READY)

#define IsDataFlowContinuous(flow) ( ( (flow) == OL_DF_CONTINUOUS ) || \
                                     ( (flow) == OL_DF_CONTINUOUS_PRETRIG ) || \
                                     ( (flow) == OL_DF_CONTINUOUS_ABOUTTRIG ) )

#define IsDataFlowSingleValue(flow) ((flow) == OL_DF_SINGLEVALUE)
                                               
#endif
