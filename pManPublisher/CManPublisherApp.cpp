/**  @moos_module Module to detect faces in a stream of images grabbed from a webcam
  *  This module gets images from a webcam or an IP cam (or video stream) and looks for faces in the images
  */

#include "CManPublisherApp.h"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace mrpt::utils;

enum CAM_TYPE {RTSP,WEBCAM};

/** Default Constructor */
CManPublisherApp::CManPublisherApp() : m_initialized_ok(false), m_start_to_detect(false)
{
	
} // end-constructor

/** Default Destructor */
CManPublisherApp::~CManPublisherApp() 
{
} // end-destructor

/** OnStartup */
bool CManPublisherApp::OnStartUp()
{
	// do nothing
	m_initialized_ok = true;
	return true;
} // end-OnStartUp

/** DoRegistrations */
bool CManPublisherApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
} // end-DoRegistrations

/** OnNewMail */
bool CManPublisherApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
	}
    UpdateMOOSVariables(NewMail);
    return true;
} // end-OnNewMail

/** Iterate */
bool CManPublisherApp::Iterate()
{
	// get topic from input
	cout << endl << "PUBLISH: ";
	string message, topic, cmd;
	getline(cin,message);
	
	std::deque<std::string> lista;
	mrpt::system::tokenize( message, " ", lista );
	ASSERT_(lista.size()>0);
	topic = lista[0].c_str();	// set the topic

	if( lista.size()>1 )		// it has commands
	{
		cmd = lista[1].c_str();
		for(int i = 2; i < lista.size(); ++i)
			cmd = cmd + " " + lista[i].c_str();
	}
	// cin >> topic >> message >> param;

	//!  @moos_publish TOPIC MESSAGE
	//m_Comms.Notify( topic, message+" "+param );
	// cout << "Topic: " << lista[0].c_str() << " and command: " << cmd << endl;
	m_Comms.Notify( topic, cmd );

	return true;
} // end-Iterate

bool CManPublisherApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

bool CManPublisherApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}