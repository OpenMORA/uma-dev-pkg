/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2013  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Mariano Jaimez Tarifa  <marianojt@uma.es>                     |
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

#include <COpenMORAMOOSApp.h>
#include <mrpt/system.h>
#include <mrpt/slam.h>
#include <mrpt/opengl.h>
#include <mrpt/gui.h>
#include <mrpt/otherlibs/do_opencv_includes.h> // <opencv2\highgui.hpp> // To avoid errors with all versions of OpenCV
#include <OpenNI.h>  // Requires OpenNI2 (right?)
#include <sstream>
#include <iostream>
#include <PS1080.h>

#include "CSocketCom.h"

class CextRGBD_App : public CMapirMOOSApp
{
public:
    CextRGBD_App();
    virtual ~CextRGBD_App();

protected:
	
	/** called at startup */
    virtual bool OnStartUp();
    /** called when new mail arrives */
    virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
    /** called when work is to be done */
    virtual bool Iterate();
    /** called when app connects to DB */
    virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
    /** performs the registration for mail */
    bool DoRegistrations();

	/** Server socket */
	CSocketCom* m_comms;	//!< Pointer to a CGiraffCommunication object for data exchange

	/** OpenCV images */
	cv::Mat dimg,RGBimg;

private:
	std::string		m_ip;	//!< IP direction where we are going to open the port 
	unsigned short	m_port; //!< Port to open
	CServerTCPSocket *srvSocket; //!< Server
	CClientTCPSocket *socket;	//!< Client	
};