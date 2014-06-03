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


/**  @moos_module RGBD sensor data adquisition and processing.*/

#include "CextRGBD_App.h"

using namespace std;
using namespace mrpt;
//using namespace mrpt::slam;
//using namespace mrpt::utils;
using namespace cv;


CextRGBD_App::CextRGBD_App()
{
}

CextRGBD_App::~CextRGBD_App()
{
}

bool CextRGBD_App::OnStartUp()
{
	try
	{
		cout << "Waiting for new frames from External Camera" << endl;
		//Create image
		dimg.create(120,160,CV_16U);
		RGBimg.create(240,320,CV_16U);

		//Create Socket (mrpt)
		m_comms = new CSocketCom("150.214.109.179", 25557);
				
		DoRegistrations();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CextRGBD_App::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("This module only accepts string command messages\n");

    std::string sCmd = Msg.GetString();
//  MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CextRGBD_App::Iterate()
{
	try
	{
		static int nSent = 0;
		static CTicTac tictacShow;		
		/*
							USING MOOS

		// Depth Frame:
		CMOOSVariable * pVarDepth = GetMOOSVar( "DEPTH_FRAME" );
		if( pVarDepth && pVarDepth->IsFresh() )
		{			
			pVarDepth->SetFresh(false);			
			std::string sTemp = pVarDepth->GetStringVal();
			
			//Copy as CV matrix and display
			dimg.data = (uchar*) sTemp.c_str();
			cv::namedWindow( "Depth", cv::WINDOW_AUTOSIZE );// Create a window for display.			
			imshow( "Depth", dimg );                   // Show our image inside it.
			cv::waitKey(1); // waits to display frame

			nSent++;		
		}

		// RGB Frame:
		CMOOSVariable * pVarRGB = GetMOOSVar( "RGB_FRAME" );		
		if( pVarRGB && pVarRGB->IsFresh() )
		{
			pVarRGB->SetFresh(false);
			std::string sTemp = pVarRGB->GetStringVal();
			
			//Copy as CV matrix and display			
			RGBimg.data = (uchar*) sTemp.c_str();
			cv::namedWindow( "RGB", cv::WINDOW_AUTOSIZE );// Create a window for display.						
			cv::flip(RGBimg, RGBimg, 1);
			imshow( "RGB", RGBimg );                   // Show our image inside it.
			cv::waitKey(1); // waits to display frame
		}
		*/

		// Read from Socket Depth		
		uchar imgDepth[40000];		
		if (m_comms->read(imgDepth,38400) )
		{
			//Copy as CV matrix and display			
			dimg.data = imgDepth;
			cv::namedWindow( "Depth", cv::WINDOW_AUTOSIZE );// Create a window for display.			
			imshow( "Depth", dimg );                   // Show our image inside it.
			cv::waitKey(1); // waits to display frame			
		}
		// Read from Socket RGB		
		uchar imgRGB[200000];		
		if (m_comms->read(imgRGB,153600) )
		{
			//Copy as CV matrix and display			
			RGBimg.data = imgRGB;
			cv::namedWindow( "RGB", cv::WINDOW_AUTOSIZE );// Create a window for display.			
			cv::flip(RGBimg, RGBimg, 1);
			imshow( "RGB", RGBimg );                   // Show our image inside it.			
			cv::waitKey(1); // waits to display frame			
		}

		nSent++;
		// Show FrameRate
		if (tictacShow.Tac()>1.0)
		{
			const double rate = double(nSent)/(1e-5+tictacShow.Tac());
			printf("[PrimeSense_RGBD] SENSOR RATE %f\n", rate );
			nSent = 0;
			tictacShow.Tic();
		}

		return true;

	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CextRGBD_App::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CextRGBD_App::DoRegistrations()
{
	//! @moos_subscribe DEPTH_FRAME
	AddMOOSVariable( "DEPTH_FRAME", "DEPTH_FRAME", "DEPTH_FRAME", 0 );
	AddMOOSVariable( "RGB_FRAME", "RGB_FRAME", "RGB_FRAME", 0 );	

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
    RegisterMOOSVariables();

    return true;
}


bool CextRGBD_App::OnNewMail(MOOSMSG_LIST &NewMail)
{
    UpdateMOOSVariables(NewMail);

    for (MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
    {
    	try
    	{
			if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
			{
				if ( srvSocket )
					delete srvSocket;

				if ( socket )
					delete socket;

				// Disconnect comms:
				MOOSTrace("Closing Module \n");
				this->RequestQuit();
			}
    	}
    	catch (std::exception &e)
    	{
    		cerr << "**ERROR** processing mail: " << i->GetName() << endl << e.what() << endl;
    	}
		system::sleep(1);
	}

    return true;
}