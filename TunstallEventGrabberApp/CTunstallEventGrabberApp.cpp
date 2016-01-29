/**  @moos_module Module to detect faces in a stream of images grabbed from a webcam
*  This module gets images from a webcam or an IP cam (or video stream) and looks for faces in the images
*/

#include "CTunstallEventGrabberApp.h"

#include "mrpt/utils/CTimeLogger.h"
#include "mrpt/utils/CConfigFileMemory.h"
#include "mrpt/obs/CObservationImage.h"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace mrpt::utils;

#define VERBOSE_LEVEL(_LEVEL) if(m_verbose_level>=_LEVEL) cout

/** Default Constructor */
CTunstallEventGrabberApp::CTunstallEventGrabberApp() :
	m_initialized_ok	(false),
	m_show_view			(false),
	m_save_to_file		(false),
	m_start_grabbing	(false),
	m_verbose_level		(1),
	m_port				(3),
	m_display_window	(NULL),
	m_image				(UNINITIALIZED_IMAGE),
	m_tout				(0),			// 0 : indefinitely
	m_log_filename		(""),
	m_log_file			(NULL)
{} // end-constructor

/** Default Destructor */
CTunstallEventGrabberApp::~CTunstallEventGrabberApp()
{
	// clear windows
	if(m_display_window)
		m_display_window.clear();

	// close log file if opened
	if( m_log_file )
		mrpt::system::os::fclose(m_log_file);

	// disconnect Anthena
	int res = wrapDisconnectAthena();
} // end-destructor

// CUSTOM METHODS
/** Connect to Athena */
int CTunstallEventGrabberApp::wrapConnectAthena(const char* in)
{
	int res, comPortNum;
	sscanf(in, "%d", &comPortNum);

	VERBOSE_LEVEL(1) << "Connecting to Athena in port COM " << comPortNum << "...";
	res = connectAthena(comPortNum);
	if( res == 1 )	
	{
		VERBOSE_LEVEL(1) << " -- DONE." << endl;
	}
	else		
	{
		VERBOSE_LEVEL(1) << " -- ERROR connecting to Athena. Output code(" << res << ")" << endl;
	}
	return res;
}

//int CTunstallEventGrabberApp::wrapConnectAthena(const int & port) const
//{
//	VERBOSE_LEVEL(1) << "Connecting to Athena in port COM" << port << "...";
//	int res = connectAthena(port);
//	if( !res )	VERBOSE_LEVEL(1) << " -- ERROR connecting to Athena" << endl;
//	else		VERBOSE_LEVEL(1) << " -- DONE. Output code(" << res << ")" << endl;
//	return res;
//}

/** Disconnect Athena */
int CTunstallEventGrabberApp::wrapDisconnectAthena() const
{
	VERBOSE_LEVEL(1) << "Disconnecting Athena...";
	int res = disconnectAthena();
	if( !res )	VERBOSE_LEVEL(1) << " -- ERROR disconnecting Athena" << endl;
	else		VERBOSE_LEVEL(1) << " -- DONE. Output code(" << res << ")" << endl;
	return res;
}

/** Get Event */
bool CTunstallEventGrabberApp::getTunstallEvent( TTunstallEvent & out_event ) const
{
#define EVENT_STRING_BUFFER_SIZE 4
	unsigned char eventStringBuf[EVENT_STRING_BUFFER_SIZE + 1];
	eventStringBuf[EVENT_STRING_BUFFER_SIZE] = 0;

	getEvent(eventStringBuf);

	// fill output structure
	out_event.tstamp = mrpt::system::now();
	out_event.plain_event = mrpt::format("%s",eventStringBuf);
	out_event.tunstall_event = string2tet(out_event.plain_event);

	// filter out NON VALID events (CODE: 0000)
	if( out_event.tunstall_event == TET_NONE ) 
		return false;

	return true;
}

/** Convert event to TUNSTALL_EVENT_TYPE */
TUNSTALL_EVENT_TYPE CTunstallEventGrabberApp::string2tet(const string & in_event) const
{
	if( !in_event.compare(0,3,"XXX") )			return TET_HANDSHAKE;
	else if( !in_event.compare(0,4,"0000") )	return TET_NONE;
	else if( !in_event.compare(0,2,"AR") )		return TET_DOOR_CLOSED;
	else if( !in_event.compare(0,2,"AQ") )		return TET_DOOR_OPEN;
	else if( !in_event.compare(0,2,"BA") )		return TET_CHAIR_ON;
	else if( !in_event.compare(0,2,"AZ") )		return TET_CHAIR_OFF;
	else										return TET_UNKNOWN;
}

/** OnStartup */
bool CTunstallEventGrabberApp::OnStartUp()
{
	// load params from mission file
	// show view flag
	m_MissionReader.GetConfigurationParam( "show_view", m_show_view );

	if( m_show_view )
		m_display_window = mrpt::gui::CDisplayWindow::Create("Tunstall Event Info");

	// sets the level of verbosity for this module: [0] Only errors, [1] Important info, [2] Everything!
	m_MissionReader.GetConfigurationParam( "verbose_level", m_verbose_level );

	// sets the detector timeout (in seconds)
	m_MissionReader.GetConfigurationParam( "timeout", m_tout );

	// sets the COM port number
	m_MissionReader.GetConfigurationParam( "com_port_number", m_port );
	
	// log info
	m_MissionReader.GetConfigurationParam( "save_to_file", m_save_to_file );
	m_MissionReader.GetConfigurationParam( "log_filename", m_log_filename );

	if( m_save_to_file )
	{
		m_log_file = m_log_filename.empty() ?
			mrpt::system::os::fopen( "TunstallEvents.log",   "wt") :
			mrpt::system::os::fopen( m_log_filename.c_str(), "wt");
		ASSERT_(m_log_file)
	}
	Sleep(1000);

	// Connect to Athena:
	char str[256];
	sprintf(str,"%d",m_port);
	if( wrapConnectAthena(str) != 1 )
	{
		VERBOSE_LEVEL(0) << "[TunstallEventGrabberApp -- ERROR] Could not connect to Athena!" << endl;
		return false;
	}

	m_initialized_ok = true;
	return true;
} // end-OnStartUp

/** DoRegistrations */
bool CTunstallEventGrabberApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe TUNSTALL_CMD
	// TUNSTALL_CMD {START,STOP}
	AddMOOSVariable( "TUNSTALL_CMD", "TUNSTALL_CMD", "TUNSTALL_CMD", 0 );

	RegisterMOOSVariables();
	return true;
} // end-DoRegistrations

/** OnNewMail */
bool CTunstallEventGrabberApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		if( (i->GetName() == "SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}

		if( i->GetName() == "TUNSTALL_CMD" )
		{
			VERBOSE_LEVEL(2) << "[CTunstallEventGrabberApp -- INFO] Received Command: " << i->GetString().c_str() << endl;

			std::deque<std::string> lista;
			mrpt::system::tokenize( i->GetString(), " ", lista );

			// Parse command
			cad = MOOSToLower( lista[0].c_str() );

			// START DETECTING
			if( MOOSStrCmp(cad,"start") )
			{
				m_ini_time = mrpt::system::now();	// set initial detecting time
				m_start_grabbing = true;
				VERBOSE_LEVEL(1) << "[CTunstallEventGrabberApp -- INFO] Starting to grab ..." << endl;
			}
			// STOP DETECTING
			else if( MOOSStrCmp(cad,"stop") )
			{
				m_start_grabbing = false;
				VERBOSE_LEVEL(1) << "[CTunstallEventGrabberApp -- INFO] Stopped" << endl;
			}
			else VERBOSE_LEVEL(0) << "[CTunstallEventGrabberApp -- ERROR] Command not recognized: " << cad << endl;
		} // end
	} // end-for
	UpdateMOOSVariables(NewMail);
	return true;
} // end-OnNewMail

/** Iterate */
bool CTunstallEventGrabberApp::Iterate()
{
	if( !m_initialized_ok )
	{
		// try to connect here (we assume that parameters have been already loaded correctly)
		char str[256];
		sprintf(str,"%d",m_port);
		if( wrapConnectAthena(str) != 1 )
		{
			VERBOSE_LEVEL(0) << "[TunstallEventGrabberApp -- ERROR] Could not connect to Athena in port COM " << m_port << "!" << endl;
			return true;
		}
		else
			m_initialized_ok = true;
	}

	if( !m_start_grabbing )
		return true;

	// get tunstall event
	VERBOSE_LEVEL(2) << "Getting event...";
	TTunstallEvent tunstall_event;
	bool valid = getTunstallEvent( tunstall_event );
	VERBOSE_LEVEL(2) << " -- DONE" << endl;

	if( true /*valid*/ ) // only send information if there is a valid message
	{
		mrpt::system::TTimeParts tp;
		mrpt::system::timestampToParts(tunstall_event.tstamp,tp,true/*localtime*/);

		string friendly_text = mrpt::format("[%s]:%d-%02d-%02d %02d:%02d:%02d",
			tunstall_event.plain_event.c_str(),
			tp.year, tp.month, tp.day,
			tp.hour, tp.minute, int(std::floor(tp.second)));

		VERBOSE_LEVEL(1) << "Received event: " << friendly_text;

		//!  @moos_publish TUNSTALL_EVENT
		string message = mrpt::format("%02d,%d,%02d,%02d,%02d,%02d,%02d",
			int(tunstall_event.tunstall_event),
			tp.year, tp.month, tp.day,
			tp.hour, tp.minute, int(std::floor(tp.second)));

		m_Comms.Notify( "TUNSTALL_EVENT", message);
		VERBOSE_LEVEL(1) << " -- Sent: " << message << endl;

		if( m_log_file )
			fprintf( m_log_file, "%s\n", friendly_text.c_str() );
	}

	return true;
} // end-Iterate

bool CTunstallEventGrabberApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

bool CTunstallEventGrabberApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}