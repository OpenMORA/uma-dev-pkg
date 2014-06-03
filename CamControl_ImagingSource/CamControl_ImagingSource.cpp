/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2013  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Manuel Lopez Antequera  <mlopezantequera@gmail.com>           |
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

/**  @moos_module Module to reading the information from some waspmote.
	* This module reads the variables published by other modules as:
	* The file saved is an MRPT rawlog format file that can be viewed, edited and used withing the MRPT RawlogViewer tool
	*/
	
#include "CamControl_ImagingSource.h"
#include <mrpt/utils.h>

#define MAPIR_ZOOMCAM_SERIAL_N 77353521959
	
using namespace std;
using namespace mrpt;
using namespace DShowLib;

CamControl_ImagingSourceApp::CamControl_ImagingSourceApp()
{
}

CamControl_ImagingSourceApp::~CamControl_ImagingSourceApp()
{
	ExitLibrary();
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CamControl_ImagingSourceApp::OnStartUp()
{
	EnableCommandMessageFiltering(true);

	try
	{
	    //Read variables from mission file.
	    serial	=	m_ini.read_uint64_t("pCamComtrol_ImagingSource","serial",MAPIR_ZOOMCAM_SERIAL_N);
		return true;
	}
	catch (std::exception &e)
	{
	    cerr << "**ERROR** " << e.what() << endl;
	    return MOOSFail( "Closing due to an exception." );
	}
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CamControl_ImagingSourceApp::OnCommandMsg( CMOOSMsg Msg )
{
  if(Msg.IsSkewed(MOOSTime())) return true;
  if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
  const std::string sCmd = Msg.GetString();
  //MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
  // Process the command "sCmd".

  return true;
}


//-------------------------------------
// Personal Procedures
//-------------------------------------
bool CamControl_ImagingSourceApp::zoom(double zoom_percentage)
{
	if (!m_Grabber.isDevValid())
	{
		std::cout << "Device not open, can't zoom!" << std::endl;
		return false;
	}
	
	DShowLib::tIVCDPropertyItemsPtr p_properties = m_Grabber.getAvailableVCDProperties();
	CSimplePropertyAccess prop_access(p_properties);
	tVCDPropertyItemArray prop_array = p_properties->getItems();
	float maxzoom,minzoom;


		
	if (zoom_percentage > 1 || zoom_percentage < 0)
	{
		std::cout << "Invalid zoom_percentage, 0<zoom_percentage<1" << std::endl;
		return false;
	}

	if (!prop_access.isAvailable(VCDID_Zoom))
	{
		std::cout << "Zoom not available!" << std::endl;
		return false;
	}
		
	minzoom = prop_access.getRangeMin(VCDID_Zoom);
	maxzoom = prop_access.getRangeMax(VCDID_Zoom);
	long zoomval = (maxzoom-minzoom)*zoom_percentage+minzoom;

	prop_access.setValue(VCDID_Zoom,zoomval);
					
	std::cout << "Zoomed to " << zoom_percentage*100 << " percent. [" << zoomval << "]" << std::endl;
	return true;
}

bool CamControl_ImagingSourceApp::autofocus()
{
	if (!m_Grabber.isDevValid())
	{
		std::cout << "Device not open, can't autofocus!" << std::endl;
		return false;
	}

	DShowLib::tIVCDPropertyItemsPtr p_properties = m_Grabber.getAvailableVCDProperties();
	CSimplePropertyAccess prop_access(p_properties);
	tVCDPropertyItemArray prop_array = p_properties->getItems();

	if (!prop_access.isAvailable(VCDID_Focus))
	{
		std::cout << "Focus control not available!" << std::endl;
		return false;
	}
	if (!prop_access.isOnePushAvailable(VCDID_Focus))
	{
		std::cout << "Autofocus not available!" << std::endl;
		return false;
	}
	std::cout << "Autofocus successful" << std::endl;
	prop_access.push(VCDID_Focus);
	return true;
}

bool CamControl_ImagingSourceApp::ConnectCam(long long serial)
{
	bool ok;
	
	InitLibrary();

	Grabber::tVidCapDevListPtr pVidCapDevList = m_Grabber.getAvailableVideoCaptureDevices();
	if( pVidCapDevList == 0 || pVidCapDevList->empty() )
	{
		ExitLibrary();
		return MOOSFail( "No devices available" ); // No device available.
	}

	for(unsigned int i = 0 ; i < pVidCapDevList->size() ; i++) 
	{
		ok = pVidCapDevList->at(i).getSerialNumber(serial);
		if (ok && (serial == serial))
			{
				std::cout <<  "Found camera: " << pVidCapDevList->at(i).getUniqueName() << std::endl;
				break;
			}
	}

	if (!m_Grabber.openDev(serial))
	{
		std::cout << "ERROR: Can't open device!" << std::endl;
		ExitLibrary();
		return MOOSFail( "Couldn't open device." );
	}
    return true;
}

bool CamControl_ImagingSourceApp::ListAllAvailableCamProperties()
{
	if (!m_Grabber.isDevValid())
	{
		std::cout << "Device not open, can't get properties!" << std::endl;
		return false;
	}
	DShowLib::tIVCDPropertyItemsPtr p_properties = m_Grabber.getAvailableVCDProperties();
	CSimplePropertyAccess prop_access(p_properties);
	tVCDPropertyItemArray prop_array = p_properties->getItems();

	if ((m_Grabber.isDevValid() && m_Grabber.isDevOpen()))
	{
		for(unsigned int i = 0 ; i < prop_array.size() ; i++) 
		{
			GUID iid = prop_array.at(i)->getItemID();

			std::cout << prop_array.at(i)->getName() << ": " << prop_access.getValue(iid);

			if (prop_access.isAutoAvailable(iid))
			{
				std::cout <<"Auto Available: ";
				if (prop_access.getAuto(iid))
					std::cout << "ON";
				else
					std::cout << "OFF";
			}

			std::cout << " Range[" <<prop_access.getRangeMin(iid)  << "," << prop_access.getRangeMax(iid) << "]";

			if (prop_access.isSwitchAvailable(iid))
			{
				std::cout <<"Switch Available: ";
				if (prop_access.getSwitch(iid))
					std::cout << "ON";
				else
					std::cout << "OFF";
			}

			if (prop_access.isOnePushAvailable(iid))
				std::cout << " [OnePush available]";
			
			std::cout << std::endl;
		}
		//p_properties->Release();
		return true;
	}
	std::cout << "Device invalid or not open" <<std::endl;
	//p_properties->Release();
	return false;
}

//-------------------------------------
// Iterate()
//-------------------------------------

bool CamControl_ImagingSourceApp::Iterate()
{
  try
  {
	if (!m_Grabber.isDevValid())
	{
		ConnectCam(serial);
		MOOSPause(1000);
		if (m_Grabber.isDevValid())
			ListAllAvailableCamProperties();
	}
    return true;
  }

  catch(exception& e)
  {
    cerr << "**ERROR** " << e.what() << endl;
    return MOOSFail( "Closing due to an exception." );
  }
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CamControl_ImagingSourceApp::OnConnectToServer()
{
  DoRegistrations();
  return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CamControl_ImagingSourceApp::DoRegistrations()
{
  //! @moos_subscribe SHUTDOWN
  AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

  //! @moos_subscribe zoomcam_zoom
  AddMOOSVariable( "zoomcam_zoom", "zoomcam_zoom", "zoomcam_zoom", 0 );

  //! @moos_subscribe zoomcam_focus
  AddMOOSVariable( "zoomcam_focus", "zoomcam_focus", "zoomcam_focus", 0 );

  RegisterMOOSVariables();
  return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CamControl_ImagingSourceApp::OnNewMail(MOOSMSG_LIST &NewMail)
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
      if( (i->GetName()=="zoomcam_zoom") && (MOOSIsNumeric(i->GetString())) )
        {
			std::cout << "zooming to " << i->GetDouble() << std::endl;	
			zoom(i->GetDouble());
        }
	  if( (i->GetName()=="zoomcam_focus") )
        {
			std::cout << "autofocusing" << std::endl;
			autofocus();
        }
    }
  UpdateMOOSVariables(NewMail);
  return true;
}
