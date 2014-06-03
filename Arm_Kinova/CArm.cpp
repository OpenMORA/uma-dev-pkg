/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2012  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |    Robotics (MAPIR) Lab, University of Malaga (Spain).                    |
   |    Contact: Carlos Sánchez  <carlossanchez@uma.es>                        |
   |                                                                           |
   |   This file is part of the MORA project.                                  |
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

/**  @moos_module  */


#include "CArm.h"
#include <mrpt/base.h>

using namespace std;
using namespace System;
using namespace System::Collections::Generic;
using namespace System::Threading;
using namespace Kinova::API::Jaco;
using namespace Kinova::API::Jaco::Control;
using namespace Kinova::API::Jaco::Diagnostic;
using namespace Kinova::DLL::SafeGate;
using namespace Kinova::DLL::Data::Jaco::Diagnostic;
using namespace Kinova::DLL::Data::Jaco;
using namespace Kinova::DLL::Data::Jaco::Control;
using namespace Kinova::DLL::Data::Util;
using namespace Kinova::DLL::Data::Jaco;
using namespace Kinova::DLL::Data::Jaco::Config;

mrpt::synch::CSemaphore m_semaphore(1,1,"semaphore");

// Constructor
CArm::CArm() 
{
	state = 99;
	joystick_Mode = 0;
	
	Jaco = gcnew CKinova(); 
	calibration = new CCalibration();

	Jaco->activeJoystick = 0;

	Jaco->Inicialize ();

	Console::WriteLine(L"Jaco Inicialized");

	Jaco->ChangeSpeed (0.5, 0.04);

	//Jaco->PositionDisplace(0,0,0);

	mrpt::utils::createThreadFromObjectMethod(this,&CArm::Protection,124);

	mrpt::utils::createThreadFromObjectMethod(this,&CArm::JoystickController,123);

}

// Destructor
CArm::~CArm()
{
		
	//Jaco->OffSecuence();

	delete calibration;

}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CArm::OnStartUp()
{
	//! @moos_param debugMode
	Jaco->debugMode = m_ini.read_bool("","debugMode",0,true); 
	//! @moos_param cameraId
	Jaco->cameraId = m_ini.read_int("","cameraId",0,true); 
	//! @moos_param framesIC
	Jaco->framesIC = m_ini.read_int("","framesIC",0,true);
	//! @moos_param framesEC
	Jaco->framesEX = m_ini.read_int("","framesEX",25,true); 
	//! @moos_param squareSize
	Jaco->squareSize = m_ini.read_float("","squareSize",0,true); 
	//! @moos_param boardSizewidth
	Jaco->boardSizewidth = m_ini.read_int("","boardSizewidth",0,true); 
	//! @moos_param boardSizeheight
	Jaco->boardSizeheight = m_ini.read_int("","boardSizeheight",0,true); 

	return DoRegistrations();

}

//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CArm::OnCommandMsg( CMOOSMsg Msg ) 
{

	if(Msg.IsSkewed( MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

//-------------------------------------
// Iterate()
//-------------------------------------
bool CArm::Iterate()
{
		
/*		if (Jaco->JacoIsReady())
		{
			//Console::WriteLine("Jaco is Ready");

			mrpt::math::CPose3D arm_Pose (Jaco->XPosition, Jaco->YPosition, Jaco->ZPosition, Jaco->ThetaX, Jaco->ThetaY, Jaco->ThetaZ);

			string sArm_Pose;

			arm_Pose.asString(sArm_Pose);

			if (state!=99)
				m_Comms.Notify("ARM_POSE", sArm_Pose);	

			switch (state)
			{
				case 0:
					Jaco->MoveTo(-0.032f, -0.695f, 0.33f);
					state = 1;
				
				case 1:
					if (Jaco->TargetReachedCartesian(Jaco->xTarget, Jaco->yTarget, Jaco->zTarget))
					{
						Jaco->MoveTo(-0.032f, -0.695f, 0.33f);
						Thread::Sleep(200);
						state = 2;
					}
					break;


				case 2:
					if (Jaco->TargetReachedCartesian(Jaco->xTarget, Jaco->yTarget, Jaco->zTarget))
					{
						Jaco->MoveTo(-0.032f, -0.695f, 0.18f);
						Thread::Sleep(200);
						state = 3;
					}
					break;

				case 3:
			
					if (Jaco->TargetReachedCartesian(Jaco->xTarget, Jaco->yTarget, Jaco->zTarget))
					{
						Jaco->MoveTo(-0.032f, -0.545f, 0.18f);
						Thread::Sleep(200);
						state = 4;
					}
					break;

				case 4:
					if (Jaco->TargetReachedCartesian(Jaco->xTarget, Jaco->yTarget, Jaco->zTarget))
					{
						Jaco->MoveTo(-0.032f, -0.545f, 0.33f);
						Thread::Sleep(200);
						state = 5;
					}
					break;

				case 5:
					if (Jaco->TargetReachedCartesian(Jaco->xTarget, Jaco->yTarget, Jaco->zTarget))
					{
						state = 99;
						//!  @moos_publish   
						m_Comms.Notify("ARM_INUSE", "false");
					}
					break;
			} 
		}*/
	return true; 
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CArm::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CArm::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe ARM_GOHOME
	AddMOOSVariable( "ARM_INUSE", "ARM_INUSE", "ARM_INUSE", 0 );
	
	//! @moos_subscribe ARM_GOHOME
	AddMOOSVariable( "ARM_GOHOME", "ARM_GOHOME", "ARM_GOHOME", 0 );

	//! @moos_subscribe ARM_OFFSECUENCE
	AddMOOSVariable( "ARM_OFFSECUENCE", "ARM_OFFSECUENCE", "ARM_OFFSECUENCE", 0 );

	//! @moos_subscribe ARM_RETRACT
	AddMOOSVariable( "ARM_RETRACT", "ARM_RETRACT", "ARM_RETRACT", 0 );

	//! @moos_subscribe ARM_READYTOPICK
	AddMOOSVariable( "ARM_READYTOPICK", "ARM_READYTOPICK", "ARM_READYTOPICK", 0 );

	//! @moos_subscribe ARM_RETRACT
	AddMOOSVariable( "ARM_READYTOPUSH", "ARM_READYTOPUSH", "ARM_READYTOPUSH", 0 );

	//! @moos_subscribe ARM_READYTOCLAMP
	AddMOOSVariable( "ARM_READYTOCLAMP", "ARM_READYTOCLAMP", "ARM_READYTOCLAMP", 0 );

	//! @moos_subscribe ARM_READYTOCALIBRATE
	AddMOOSVariable( "ARM_READYTOCALIBRATE", "ARM_READYTOCALIBRATE", "ARM_READYTOCALIBRATE", 0 );

	//! @moos_subscribe ARM_PICKOBJECT
	AddMOOSVariable( "ARM_PICKOBJECT", "ARM_PICKOBJECT", "ARM_PICKOBJECT", 0 );

	//! @moos_subscribe ARM_DROPOBJECT
	AddMOOSVariable( "ARM_DROPOBJECT", "ARM_DROPOBJECT", "ARM_DROPOBJECT", 0 );

	//! @moos_subscribe ARM_ADDCOMPLETETRAJECTORY
	AddMOOSVariable( "ARM_ADDCOMPLETETRAJECTORY", "ARM_ADDCOMPLETETRAJECTORY", "ARM_ADDCOMPLETETRAJECTORY", 0 );

	//! @moos_subscribe ARM_MOVETOANGLE
	AddMOOSVariable( "ARM_MOVETOANGLE", "ARM_MOVETOANGLE", "ARM_MOVETOANGLE", 0 );

	//! @moos_subscribe ARM_MOVETO
	AddMOOSVariable( "ARM_MOVETO", "ARM_MOVETO", "ARM_MOVETO", 0 );

	//! @moos_subscribe ARM_ROTATIONTO
	AddMOOSVariable( "ARM_ROTATIONTO", "ARM_ROTATIONTO", "ARM_ROTATIONTO", 0 );

	//! @moos_subscribe ARM_FINGERTO
	AddMOOSVariable( "ARM_FINGERTO", "ARM_FINGERTO", "ARM_FINGERTO", 0 );

	//! @moos_subscribe ARM_POSITIONDISPLACE
	AddMOOSVariable( "ARM_POSITIONDISPLACE", "ARM_POSITIONDISPLACE", "ARM_POSITIONDISPLACE", 0 );

	//! @moos_subscribe ARM_ROTATIONDISPLACE
	AddMOOSVariable( "ARM_ROTATIONDISPLACE", "ARM_ROTATIONDISPLACE", "ARM_ROTATIONDISPLACE", 0 );

	//! @moos_subscribe ARM_FINGERDISPLACE
	AddMOOSVariable( "ARM_FINGERDISPLACE", "ARM_FINGERDISPLACE", "ARM_FINGERDISPLACE", 0 );

	//! @moos_subscribe ARM_PICK
	AddMOOSVariable( "ARM_PICK", "ARM_PICK", "ARM_PICK", 0 );

	//! @moos_subscribe ARM_DROP
	AddMOOSVariable( "ARM_DROP", "ARM_DROP", "ARM_DROP", 0 );

	//! @moos_subscribe ARM_GIVEMEYOURHAND
	AddMOOSVariable( "ARM_GIVEMEYOURHAND", "ARM_GIVEMEYOURHAND", "ARM_GIVEMEYOURHAND", 0 );

	//! @moos_subscribe ARM_TURNHANDLEFT
	AddMOOSVariable( "ARM_TURNHANDLEFT", "ARM_TURNHANDLEFT", "ARM_TURNHANDLEFT", 0 );

	//! @moos_subscribe ARM_TURNHANDRIGHT
	AddMOOSVariable( "ARM_TURNHANDRIGHT", "ARM_TURNHANDRIGHT", "ARM_TURNHANDRIGHT", 0 );

	//! @moos_subscribe ARM_DEMOSTRATION3
	AddMOOSVariable( "ARM_DEMOSTRATION3", "ARM_DEMOSTRATION3", "ARM_DEMOSTRATION3", 0 );

	//! @moos_subscribe ARM_BLOCK
	AddMOOSVariable( "ARM_BLOCK", "ARM_BLOCK", "ARM_BLOCK", 0 );

	//! @moos_subscribe ARM_DEMOSTRATIONVIDEO
	AddMOOSVariable( "ARM_DEMOSTRATIONVIDEO", "ARM_DEMOSTRATIONVIDEO", "ARM_DEMOSTRATIONVIDEO", 0 );

	//! @moos_subscribe ARM_CALIBRATEARM
	AddMOOSVariable( "ARM_CALIBRATEARM", "ARM_CALIBRATEARM", "ARM_CALIBRATEARM", 0 );

	//! @moos_subscribe ARM_CALIBRATEARMDEPTH
	AddMOOSVariable( "ARM_CALIBRATEARMDEPTH", "ARM_CALIBRATEARMDEPTH", "ARM_CALIBRATEARMDEPTH", 0 );

	//! @moos_subscribe ARM_CALCULATEMATRIX
	AddMOOSVariable( "ARM_CALCULATEMATRIX", "ARM_CALCULATEMATRIX", "ARM_CALCULATEMATRIX", 0 );

	//! @moos_subscribe ARM_CALCULATEMATRIX2
	AddMOOSVariable( "ARM_CALCULATEMATRIX2", "ARM_CALCULATEMATRIX2", "ARM_CALCULATEMATRIX2", 0 );

	//! @moos_subscribe ARM_INTRINSICCALIBRATION
	AddMOOSVariable( "ARM_INTRINSICCALIBRATION", "ARM_INTRINSICCALIBRATION", "ARM_INTRINSICCALIBRATION", 0 );

	//! @moos_subscribe ARM_TRANSFORMATIONPOINT
	AddMOOSVariable( "ARM_TRANSFORMATIONPOINT", "ARM_TRANSFORMATIONPOINT", "ARM_TRANSFORMATIONPOINT", 0 );

	//! @moos_subscribe ARM_LOCATEPATTERN
	AddMOOSVariable( "ARM_LOCATEPATTERN", "ARM_LOCATEPATTERN", "ARM_LOCATEPATTERN", 0 );

	//! @moos_subscribe ARM_LOCATEONEPATTERN
	AddMOOSVariable( "ARM_LOCATEONEPATTERN", "ARM_LOCATEONEPATTERN", "ARM_LOCATEONEPATTERN", 0 );


	//! @moos_subscribe ARM_REPRODUCEMOVEMENT
	AddMOOSVariable( "ARM_REPRODUCEMOVEMENT", "ARM_REPRODUCEMOVEMENT", "ARM_REPRODUCEMOVEMENT", 0 );

	//! @moos_subscribe ARM_REPRODUCEONEMOVEMENT
	AddMOOSVariable( "ARM_REPRODUCEONEMOVEMENT", "ARM_REPRODUCEONEMOVEMENT", "ARM_REPRODUCEONEMOVEMENT", 0 );

	//! @moos_subscribe ARM_TAKEDEPTH
	AddMOOSVariable( "ARM_TAKEDEPTH", "ARM_TAKEDEPTH", "ARM_TAKEDEPTH", 0 );

	//! @moos_subscribe ARM_MARIANOMETHOD
	AddMOOSVariable( "ARM_MARIANOMETHOD", "ARM_MARIANOMETHOD", "ARM_MARIANOMETHOD", 0 );

	//! @moos_subscribe RESET_ODO
	AddMOOSVariable( "RESET_ODO", "RESET_ODO", "RESET_ODO", 0 );

	//! @moos_subscribe WORKING
	AddMOOSVariable( "WORKING", "WORKING", "WORKING", 0 );
	

	//! @moos_subscribe ARM_JOYSTICK
	AddMOOSVariable( "ARM_JOYSTICK", "ARM_JOYSTICK", "ARM_JOYSTICK", 0 );

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CArm::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{				
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:			
			MOOSTrace("Closing Module \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->Retract();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			this->RequestQuit();

		}

		if (i->GetName()=="ARM_GOHOME" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Jaco go to home position \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->GoHome();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_GOHOME", "false");
					
		}

		if (i->GetName()=="ARM_OFFSECUENCE" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Off secuence \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->OffSecuence();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_OFFSECUENCE", "false");
					

		}

		if (i->GetName()=="ARM_RETRACT" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Retract \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->Retract();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_RETRACT", "false");
					
		}

		if (i->GetName()=="ARM_READYTOPICK" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Ready to pick \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");
			Jaco->ReadyToPick();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_READYTOPICK", "false");
					
		}

		if (i->GetName()=="ARM_READYTOPUSH" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Ready to push \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->ReadyToPush();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_READYTOPUSH", "false");
					
		}

		if (i->GetName()=="ARM_READYTOCLAMP" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Ready to push \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->ReadyToClamp();
			
			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_READYTOCLAMP", "false");
					
		}

		if (i->GetName()=="ARM_READYTOCALIBRATE" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Ready to calibrate \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->ReadyToCalibrate();
			
			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_READYTOCALIBRATE", "false");
					
		}

		if (i->GetName()=="ARM_PICKOBJECT")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->PickObject (atof (lista[1].c_str()),atof (lista[2].c_str()),atof (lista[3].c_str()),atoi (lista[4].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}

		if (i->GetName()=="ARM_DROPOBJETC")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->DropObject (atof (lista[1].c_str()),atof (lista[2].c_str()),atof (lista[3].c_str()));
			
			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}

		if (i->GetName()=="ARM_MOVETOANGLE")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->MoveToAngle (atoi (lista[1].c_str()),atoi (lista[2].c_str()),atoi (lista[3].c_str()),atoi (lista[4].c_str()),atoi (lista[5].c_str()),atoi (lista[6].c_str()),atoi (lista[7].c_str()),atoi (lista[8].c_str()),atoi (lista[9].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}
		
		if (i->GetName()=="ARM_MOVETO" )
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->MoveTo (atof (lista[1].c_str()),atof (lista[2].c_str()),atof (lista[3].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_MOVETO", "");

		}

		if (i->GetName()=="ARM_ROTATIONTO")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->RotationTo (atof (lista[1].c_str()),atof (lista[2].c_str()),atof (lista[3].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}

		if (i->GetName()=="ARM_FINGERTO") 
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "True");

			Jaco->FingerTo (atoi (lista[1].c_str()),atoi (lista[2].c_str()),atoi (lista[3].c_str()));
			//!  @moos_publish   

			m_Comms.Notify("ARM_INUSE", "False");

		}
		
		if (i->GetName()=="ARM_POSITIONDISPLACE") 
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->PositionDisplace (atof (lista[1].c_str()),atof (lista[2].c_str()),atof (lista[3].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}

		if (i->GetName()=="ARM_ROTATIONDISPLACE")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->RotationDisplace (atof (lista[1].c_str()),atof (lista[2].c_str()),atof (lista[3].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}

		if (i->GetName()=="ARM_FINGERDISPLACE")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->FingerDisplace (atoi (lista[1].c_str()),atoi (lista[2].c_str()),atoi (lista[3].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		}

		if (i->GetName()=="ARM_PICK")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->Pick (atoi (lista[0].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");
					
		}

		if (i->GetName()=="ARM_DROP" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Pick \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->Drop();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_DROP", "false");
					
		}

		if (i->GetName()=="ARM_GIVEMEYOURHAND" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Give me your hand \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->GiveMeYourHand();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_GIVEMEYOURHAND", "false");
					
		}

		if (i->GetName()=="ARM_TURNHANDLEFT")
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->TurnHandLeft (atof (lista[0].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");
					
		}

		if (i->GetName()=="ARM_TURNHANDRIGHT" && (MOOSStrCmp(i->GetString(),"true")))
		{
			std::deque<std::string> lista;
			mrpt::utils::tokenize(i->GetString().c_str()," ",lista);

			std::string goal;
			for (size_t j=3;j<lista.size();j++)
			{
				goal=goal+lista[j]+" ";
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->TurnHandRight (atof (lista[0].c_str()));

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

					
		}

		if (i->GetName()=="ARM_DEMOSTRATION3" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Give me your hand \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->Demostration3();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_DEMOSTRATION3", "false");
					
		}

		if (i->GetName()=="ARM_BLOCK" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Arm block \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->Block();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_BLOCK", "false");
					
		}

		if (i->GetName()=="ARM_DEMOSTRATIONVIDEO" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Give me your hand \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->DemostrationVideo();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_DEMOSTRATIONVIDEO", "false");
					
		}

		if (i->GetName()=="ARM_CALIBRATEARM" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->CalibrateArm();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_CAPTURE", "false");
					
		}

		if (i->GetName()=="ARM_CALIBRATEARMDEPTH" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->CalibrateArmDepth();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_CALIBRATEARMDEPTH", "false");
					
		}

		
		if (i->GetName()=="ARM_CALCULATEMATRIX" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			calibration->CalculateMatrix();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_CALCULATEMATRIX", "false");
		}
	

		if (i->GetName()=="ARM_CALCULATEMATRIX2" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				calibration->CalculateMatrix2();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_CALCULATEMATRIX2", "false");
			}

		if (i->GetName()=="ARM_INTRINSICCALIBRATION" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				Jaco->IntrinsicCalibration();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INTRINSICCALIBRATION", "false");
			}

		if (i->GetName()=="ARM_TRANSFORMATIONPOINT" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				Jaco->TransformationPoint();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_TRANSFORMATIONPOINT", "false");
			}
		
		if (i->GetName()=="ARM_LOCATEPATTERN" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				Jaco->LocatePattern();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_LOCATEPATTERN", "false");
			}
		if (i->GetName()=="ARM_LOCATEONEPATTERN" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				Jaco->LocateOnePattern();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_LOCATEONEPATTERN", "false");
			}

		if (i->GetName()=="ARM_REPRODUCEMOVEMENT" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				Jaco->ReproduceMovement();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_REPRODUCEMOVEMENT", "false");
			}

		if (i->GetName()=="ARM_REPRODUCEONEMOVEMENT" && (MOOSStrCmp(i->GetString(),"true")))
			{
				MOOSTrace("Capture \n");

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "true");

				Jaco->ReproduceOneMovement();

				//!  @moos_publish   
				m_Comms.Notify("ARM_INUSE", "false");

				//!  @moos_publish   
				m_Comms.Notify("ARM_REPRODUCEONEMOVEMENT", "false");
			}

		if (i->GetName()=="ARM_TAKEDEPTH" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			Jaco->TakeDepth();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_TAKEDEPTH", "false");
		}

		if (i->GetName()=="ARM_MARIANOMETHOD" && (MOOSStrCmp(i->GetString(),"true")))
		{
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			calibration->MarianoMethod();

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

			//!  @moos_publish   
			m_Comms.Notify("ARM_MARIANOMETHOD", "false");
		}

		if (i->GetName()=="RESET_ODO" && (MOOSStrCmp(i->GetString(),"true")))
		{
			char c;
			
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			//Jaco->GoHome(5000);
			Jaco->ReadyToOdometry();

			cout << endl << "Is Kinect in the arm?";
			cin >> c;

			if('y' != c)
			{
				Jaco->Drop();
				cout << endl << "Push botton when Kinect is ready";
				if(cin >>c)
					Jaco->Pick (50);
						
			}

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");

		
		}


		if (i->GetName()=="WORKING" && (MOOSStrCmp(i->GetString(),"true")))
		{
			
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			state = 0;
		
		}

		if (i->GetName()=="ARM_JOYSTICK" )
		{
			
			MOOSTrace("Capture \n");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "true");

			joystick_Mode = MOOSStrCmp(i->GetString(),"true");

			//!  @moos_publish   
			m_Comms.Notify("ARM_INUSE", "false");
		
		}

	}

    UpdateMOOSVariables(NewMail);
    return true;
}

//-------------------------------------
// Protection()
//-------------------------------------
void CArm::Protection (int dato)
{

	while (1)
	{

		Jaco->GetHandPosition();
		//puts ( "PROTECTION\n" );
		Thread::Sleep(50);
	}

}

void CArm::JoystickController (int dato)
{
	while (1)
	{
		Jaco->GetHandPosition();

		Jaco->DisplayHandPosition();


		//puts ( "JoystickController" );
		if (joystick_Mode)
		{
			short esc = 0;

			esc = GetAsyncKeyState ( VK_ESCAPE );

			if ( GetAsyncKeyState ( VK_UP ) & SHRT_MAX )
			{
				puts ( "Up arrow is pressed" );
				Jaco->joystick->InclineFB = -1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( VK_DOWN ) & SHRT_MAX )
			{
				puts ( "Down arrow is pressed" );
				Jaco->joystick->InclineFB = 1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( VK_LEFT ) & SHRT_MAX )
			{
				puts ( "Left arrow is pressed" );
				Jaco->joystick->InclineLR = 1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( VK_RIGHT ) & SHRT_MAX )
			{
				puts ( "Right arrow is pressed" );
				Jaco->joystick->InclineLR = -1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( VK_OEM_PLUS ) & SHRT_MAX )
			{
				puts ( "+ key is pressed" );
				Jaco->joystick->Rotate = 1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( VK_OEM_MINUS ) & SHRT_MAX )
			{
				puts ( "- key is pressed" );
				Jaco->joystick->Rotate = -1;
				Jaco->SendJoystickFunctionality();
			}

			else if ( GetAsyncKeyState ( 0x41 ) & SHRT_MAX )
			{
				puts ( "A key is pressed" );
				Jaco->joystick->PushPull = 1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( 0x53 ) & SHRT_MAX )
			{
				puts ( "S key is pressed" );
				Jaco->joystick->PushPull = -1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( 0x44 ) & SHRT_MAX )
			{
				puts ( "D key is pressed" );
				Jaco->joystick->MoveFB = 1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( 0x46 ) & SHRT_MAX )
			{
				puts ( "F key is pressed" );
				Jaco->joystick->MoveFB = -1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( 0x47 ) & SHRT_MAX )
			{
				puts ( "G key is pressed" );
				Jaco->joystick->MoveLR = 1;
				Jaco->SendJoystickFunctionality();
			}
			else if ( GetAsyncKeyState ( 0x48 ) & SHRT_MAX )
			{
				puts ( "H key is pressed" );
				Jaco->joystick->MoveLR = -1;
				Jaco->SendJoystickFunctionality();
			}
			else
			{
				//Jaco->joystick->ButtonValue[2] = 0;
				Jaco->joystick->InclineFB = 0;
				Jaco->joystick->InclineLR = 0;
				Jaco->joystick->Rotate = 0;
				Jaco->joystick->PushPull = 0;
				Jaco->joystick->MoveFB = 0;
				Jaco->joystick->MoveLR = 0;
				Jaco->SendJoystickFunctionality();
			}
		}

		// Movement Control
		if (Jaco->activeJoystick)
		{
			puts ( "activeJoystick" );

			if(Jaco->yTarget-Jaco->YPosition>0.01)
				Jaco->joystick->InclineFB = 1;
			else if (Jaco->yTarget-Jaco->YPosition<-0.01)
				Jaco->joystick->InclineFB = -1;
			else
				Jaco->joystick->InclineFB = 0;

			if(Jaco->xTarget-Jaco->XPosition>0.01)
				Jaco->joystick->InclineLR = 1;
			else if (Jaco->xTarget-Jaco->XPosition<-0.01)
				Jaco->joystick->InclineLR = -1;
			else
				Jaco->joystick->InclineLR = 0;

			if(Jaco->zTarget-Jaco->ZPosition>0.01)
				Jaco->joystick->Rotate = 1;
			else if (Jaco->zTarget-Jaco->ZPosition<-0.01)
				Jaco->joystick->Rotate = -1;
			else
				Jaco->joystick->Rotate = 0;
	
			if(Jaco->tXTarget-Jaco->ThetaX>0.01)
				Jaco->joystick->MoveLR = 1;
			else if (Jaco->tXTarget-Jaco->ThetaX<-0.01)
				Jaco->joystick->MoveLR = -1;
			else
				Jaco->joystick->MoveLR = 0;

			if(Jaco->tYTarget-Jaco->ThetaY>0.01)
				Jaco->joystick->MoveFB = 1;
			else if (Jaco->tYTarget-Jaco->ThetaY<-0.01)
				Jaco->joystick->MoveFB = -1;
			else
				Jaco->joystick->MoveFB = 0;

			if(Jaco->tZTarget-Jaco->ThetaZ>0.01)
				Jaco->joystick->PushPull = -1;
			else if (Jaco->tZTarget-Jaco->ThetaZ<-0.01)
				Jaco->joystick->PushPull = 1;
			else
				Jaco->joystick->PushPull = 0;

			Jaco->SendJoystickFunctionality();

			if (Jaco->TargetReached (Jaco->xTarget, Jaco->yTarget, Jaco->zTarget, Jaco->tXTarget, Jaco-> tYTarget, Jaco->tZTarget))
			{
				Jaco->joystick->InclineFB = 0;
				Jaco->joystick->InclineLR = 0;
				Jaco->joystick->Rotate = 0;

				Jaco->joystick->MoveLR = 0;
				Jaco->joystick->MoveFB = 0;
				Jaco->joystick->PushPull = 0;

				Jaco->SendJoystickFunctionality();
				Jaco->activeJoystick = 0;
			}
		}

		Thread::Sleep(50);
	}


}