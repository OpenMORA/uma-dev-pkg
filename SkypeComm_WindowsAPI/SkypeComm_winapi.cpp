/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact:  Manuel López Antequera  <mlopezantequera@gmail.com>          |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/**  @moos_module SkypeComm_WindowsAPI module.
  *  This module manages a Skype client to communicate with a remote user
  *	 it only works under Windows, as it uses the Skype4COM APIs.
*/

#include "SkypeComm_winapi.h"


SkypeComm::SkypeComm()
{
	lastmsg=NULL, newmsg=NULL;

	// Initialize COM
	CoInitialize(NULL);

	//ListWebcams();

	// Initialize pointers
	pSkype.CreateInstance(__uuidof(SKYPE4COMLib::Skype));

	// Check if Skype is running. Start Skype otherwise.
	if (pSkype->Client->IsRunning == VARIANT_FALSE)
	{
		std::cout << "Skype is not running. Attempting to start Skype client..." << std::endl;
		
		if(pSkype->Client->Start(VARIANT_TRUE,VARIANT_TRUE) == VARIANT_FALSE)
		{
			std::cout << "Started client. Waiting 10 seconds to attach" << std::endl;
			Sleep(10000);
		}
		else
		{
			std::cout << "Can't start Skype! Quitting"<<std::endl;
			pSkype = NULL;
			CoUninitialize();
			this->RequestQuit();
		}
	}

	//Attach to Skype client
	try 
	{
		pSkype->Attach(8,VARIANT_TRUE);
	}
	catch (_com_error &ex )
	{
		std::cout << "Could not attach to a Skype client. Error description: " << ex.Description() << std::endl 
			      << "Quitting" << std::endl;

		pSkype = NULL;
		CoUninitialize();
		this->RequestQuit();
	}

	// Print Skype version
	_bstr_t bstrSkypeVersion = pSkype->GetVersion();
	printf("Skype client version %s\n", (char*)bstrSkypeVersion);

	// Print COM wrapper version
	_bstr_t bstrWrapperVersion = pSkype->GetApiWrapperVersion();
	printf("COM wrapper version %s\n", (char*)bstrWrapperVersion);

	//todo: make skype accept calls automatically when this is run.
	
	pSkype->ClearChatHistory();
	double lastTimestamp=0;
}

SkypeComm::~SkypeComm()
{
	pSkype = NULL;
	lastmsg = NULL;
	newmsg = NULL;

	CoUninitialize();
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool SkypeComm::OnStartUp()
{
	std::vector<std::string>      tokens;
	mrpt::system::tokenize(m_ini.read_string("pSkypeComm_WindowsAPI","valid_skype_cameras","",false),",",tokens);
	camlist = new mrpt::utils::CStringList(tokens);

		
	std :: cout << "\nRead the following cameras from the mission file: \n" << camlist->getText();
	return true;
}

//-------------------------------------
// Iterate()
//-------------------------------------
bool SkypeComm::Iterate()
{
	newmsg = GetLastMessage(pSkype);
	if (newmsg)
	{
		if (!lastmsg)
		{
			lastmsg=newmsg;
			if (strcmp(lastmsg->GetSender()->GetHandle(),pSkype->GetCurrentUser()->GetHandle()))
			{
				std::cout << "User: '"<<lastmsg->GetSender()->GetHandle() << "' sent message: '" <<lastmsg->Body << "'" <<std::endl;
				PublishNewMsg();
			}
		}
		else if (newmsg->GetTimestamp() > lastmsg->GetTimestamp())
		{
			lastmsg=newmsg;
			if (strcmp(lastmsg->GetSender()->GetHandle(),pSkype->GetCurrentUser()->GetHandle()))
			{
				std::cout << "User: '"<<lastmsg->GetSender()->GetHandle() << "' sent message: '" <<lastmsg->Body << "'" <<std::endl;
				PublishNewMsg();
			}
		}
	}
	return true;
}


//-------------------------------------
// PublishNewMsg() Publishes a received message from Skype into the SKYPE_RECV variable. The contact who send the message is published into the M_RECV_USER variable.
//-------------------------------------
void SkypeComm::PublishNewMsg()
{
    // Get the message
    std::string str;
	str = newmsg->GetBody();
    // Publish it
    //MOOSTrace(msg);

	//!  @moos_publish <SKYPE_RECV> Text received from the skype chat
    m_Comms.Notify("SKYPE_RECV",str);

	// Publish the user that the message is coming from (set in remoteUser)
	//!  @moos_publish <SKYPE_CONTACT> Username from who the last message was received
	str = newmsg->GetSender()->GetHandle();
	m_Comms.Notify("SKYPE_CONTACT",str);
}


//-------------------------------------
// runCommand(): runs a command
//-------------------------------------
void SkypeComm::runCommand(std::string command)
{
	std::stringstream commandline;
	commandline << "SKYPECOMM " << taskLaunch << " " << command.c_str();
	
	//!  @moos_publish <NEW_TASK> Launch a task from the chat
	m_Comms.Notify("NEW_TASK", commandline.str().c_str());
	taskLaunch++;
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool SkypeComm::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool SkypeComm::DoRegistrations()
{
	//! @moos_subscribe SKYPE_SEND 
	AddMOOSVariable( M_SEND, M_SEND, M_SEND, 0 );

	//! @moos_subscribe SKYPE_CONTACT
	AddMOOSVariable( M_SEND_USER, M_SEND_USER, M_SEND_USER, 0 );

	//! @moos_subscribe SKYPE_CONTACT
	AddMOOSVariable( "SKYPE_CAM", "SKYPE_CAM", "SKYPE_CAM", 0 );

	RegisterMOOSVariables();
	return true;
}

void SkypeComm::ChangeCam(unsigned int cam)
{
	if (cam < camlist->size())
	{
		std::cout<< "\n\nAttempting to change Skype camera to number "<< cam << " with name: " << (*camlist)(cam) <<std::endl;
		try 
		{ 
			pSkype->Settings->PutVideoIn((*camlist)(cam).c_str());
		}
		catch (_com_error ex) 
		{
			std::cout << "Error while changing the camera. Description: "<< ex.Description() <<std::endl;
		}
		
	}
	else
	{
		std::cout<< "Camera number " << cam << " is not on the list."<< std::endl << "Camera list is:" << std::endl << camlist->getText() <<std::endl;
		return;
	}
			
	try { std::cout << "Current camera:" << pSkype->Settings->GetVideoIn() << std::endl;}
	catch (_com_error ex) { std::cout << "Can't get current camera. ex: " << ex.Description() << std::endl; }

	//We restart every call to send the new video stream
	SKYPE4COMLib::ICallCollectionPtr calls = pSkype->GetActiveCalls(); 
	for (int j = 1 ; j <= calls->GetCount() ; j++)
	{
		SKYPE4COMLib::ICallPtr call;
		call = calls->GetItem(j);
		_bstr_t phndle = call->GetPartnerHandle();
			
		//Hang up
		try{call->Finish();}
		catch(_com_error ex){std::cout << "Error0: " << ex.Description() << std::endl;}

		std::cout << "debug: 1" << std::endl;
			
		SKYPE4COMLib::TCallStatus status = call->GetStatus();
		//Wait until call is finished
		while (!(status == SKYPE4COMLib::clsFinished || status == SKYPE4COMLib::clsFailed))
		{
			mrpt::system::sleep(500);
			status = call->GetStatus();
		}
		std::cout << "debug: 2" << std::endl;
			
		//Call back
		try{call = pSkype->PlaceCall(phndle,"","","");}
		catch(_com_error ex){std::cout << "Error2: " << ex.Description() << std::endl;}
		std::cout << "debug: 3" << std::endl;
			
		//Wait until call fails or works.
		unsigned int timeout = 0;
		status = SKYPE4COMLib::clsUnknown;
		while (!(status == SKYPE4COMLib::clsInProgress || status == SKYPE4COMLib::clsFailed || (timeout>2)))
		{
			mrpt::system::sleep(1000);
			status = call->GetStatus();
			timeout++;
			std::cout << "Call start attempt: " << timeout << std::endl;
			std::cout << "Call status: " << status << std::endl;
		}
		std::cout << "debug: 4" << std::endl;

		//if (status == SKYPE4COMLib::clsInProgress) 
		//{
		//	std::cout << "debug: 5a" << std::endl;
		//	call->StartVideoSend();
		//}
		//else
		//{
		//	std::cout << "debug: 5b" << std::endl;
		//}

		timeout = 0;
		SKYPE4COMLib::TCallVideoSendStatus vstatus;
		while (!(vstatus == SKYPE4COMLib::TCallVideoSendStatus::vssRunning || vstatus == SKYPE4COMLib::TCallVideoSendStatus::vssStarting || (timeout>2)))
		{
			mrpt::system::sleep(1000);
			try{call->StartVideoSend();}
			catch(_com_error ex){std::cout << "Error4: " << ex.Description() << std::endl;}
			mrpt::system::sleep(1000);
			vstatus = call->GetVideoSendStatus();
			timeout++;
			std::cout << "Video start attempt: " << timeout << std::endl;
			std::cout << "Video status: " << vstatus << std::endl;
		}
		std::cout << "debug: 5" << std::endl;
				
		call = NULL;
	}
	calls=NULL;
}


//-------------------------------------
// OnNewMail()
//-------------------------------------
bool SkypeComm::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;

	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
	//	MOOSTrace(format("New msg received: %s \n",i->GetKey()));

		MOOSVars[i->GetName()] = i->GetAsString();	// save the values in the map

		//std::cout << i->GetName() << "=" << i->GetAsString() << std::endl;

		if (i->GetName() == "SKYPE_SEND")
		{
			std::cout << "Sending '" << i->GetAsString() <<"' to contact '" << remoteUser << "'" <<std::endl;
			pSkype->sk4com_SendMessage(remoteUser.c_str(),i->GetAsString().c_str());
		}

		if (i->GetName() == "SKYPE_CAM") //We change the active Skype camera
		{
			if (i->IsDouble())
			{
				unsigned int cam = i->GetDouble();
				unsigned int j = 0;
				char szFinal[255];
				
				while( ((*camlist)(cam).compare(szFinal) != 0) && (j<1))
				{
					try 
					{
						ChangeCam(cam);		
						mrpt::system::sleep(500);
						_stprintf(szFinal, _T("%s"), (LPCTSTR)pSkype->Settings->GetVideoIn());
						j++;
					}
					catch (_com_error &ex ) { std::cout << "Can't get current camera. ex: " << ex.Description() << std::endl; }
				}
				if ((*camlist)(cam).compare(szFinal) != 0) 
				{
					std::cout << "Could not change camera. Please try again" << std::endl;
					pSkype->sk4com_SendMessage(remoteUser.c_str(),"Could not change camera. Please try again");
				}
				else {std::cout << "Camera changed to " << (*camlist)(cam) << std::endl;}
			}
		}

		if (i->GetName() == "SKYPE_CONTACT")
		{
			remoteUser = i->GetAsString();
			std::cout << "Contact changed to '" << remoteUser << "'"  <<std::endl;
		}

        if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
	}

	//UpdateMOOSVariables(NewMail);
	return true;
}

SKYPE4COMLib::IChatMessagePtr SkypeComm::GetLastMessage(SKYPE4COMLib::ISkypePtr pSkype)
{
	SKYPE4COMLib::IChatCollectionPtr chats;
	SKYPE4COMLib::IChatMessagePtr lastMessagePtr;
	SKYPE4COMLib::IChatPtr lastChatPtr;
	double lastTimestamp = 0;

	pSkype->ResetCache();


	//todo Find a better, faster way to get the latest chat msg. comparing timestamps doesn't allow us to differentiate messages which are sent in fast succession.
	//A quick option is to clear the chat history every time. Any messages in the history will be 'new' after that.


	chats = pSkype->GetRecentChats();
	for (int i = 1 ; i<= chats->GetCount() ; i++)
	{
		for (int j = 1 ; j<= chats->GetItem(i)->GetMessages()->GetCount() ; j++)
		{
			if (chats->GetItem(i)->GetMessages()->GetItem(j)->GetTimestamp() > lastTimestamp)
			{
				lastTimestamp = chats->GetItem(i)->GetMessages()->GetItem(j)->GetTimestamp();
				lastChatPtr = chats->GetItem(i);
				lastMessagePtr = lastChatPtr->GetMessages()->GetItem(j);
			}
		}
	}
	chats = NULL;
	lastChatPtr = NULL;
	return lastMessagePtr;
}

