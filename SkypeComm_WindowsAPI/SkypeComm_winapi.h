/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Emil Jatib Khatib  <emilkhatib@uma.es>		                   |
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

#ifndef SkypeComm_H
#define SkypeComm_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <CMapirMOOSApp.h>

#include <string>
#include <fstream>
#include <mrpt/system.h>
#include <map>

#include <stdio.h>
#include <tchar.h>
#include <Windows.h>
#include <iostream>

#include <mrpt/utils/CStringList.h>

#import  <Skype4COM.dll> rename("CreateEvent", "sk4com_CreateEvent") rename("SendMessage", "sk4com_SendMessage")

#define M_SEND "SKYPE_SEND"
#define M_SEND_USER "SKYPE_CONTACT"

class SkypeComm : public CMapirMOOSApp
{
public:
    SkypeComm();
    virtual ~SkypeComm();

protected:
	SKYPE4COMLib::ISkypePtr pSkype;
	SKYPE4COMLib::IChatMessagePtr lastmsg, newmsg;
	SKYPE4COMLib::IChatMessagePtr GetLastMessage(SKYPE4COMLib::ISkypePtr pSkype);
	mrpt::utils::CStringList *camlist;

	void PublishNewMsg();

	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	//bool OnCommandMsg( CMOOSMsg Msg );
	/** state our interest in variables from other modules (registration for mail)*/
	bool DoRegistrations();

	//bool ChangeCam(int cam);
	/** Changes the camera to the one specified in camlist(cam).
	To make sure the partners get the new video feed, all the calls are restarted*/
	void SkypeComm::ChangeCam(unsigned int cam);

	/** run a command */
	void runCommand(std::string command);

	/** subscribe to MOOS variables */
	//void AddVars();

	/** Task number */
	int taskLaunch;
	
	/** Remote user name */
	std::string remoteUser;

	/** list of MOOS variables */
	std::map<std::string,std::string> MOOSVars;
	
};
#endif
