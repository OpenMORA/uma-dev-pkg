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


#include "CKinova.h"
#include "CArm.h"
#include <mrpt/base.h>
#include <mrpt/gui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace System;
using namespace System::Threading;
using namespace mrpt::math;


// Constructor
CKinova::CKinova()
{
	// Kinova objects
	jaco = gcnew CJacoArm (Crypto::GetInstance()->Encrypt("C6H12O6h2so4")); //An Object that represents the robotic arm Jaco.
	joystick = gcnew CJoystickValue();
	pointsTrajectory = gcnew CPointsTrajectory();
	trajectory = gcnew CTrajectoryInfo();
	cVector= gcnew CVectorEuler();
	zoneList = gcnew CZoneList();
	trajectoryInfo = gcnew CTrajectoryInfo();
	cartesianInfo = gcnew CCartesianInfo();
	handPosition = gcnew CVectorEuler();
	jointPosition = gcnew CVectorAngle();
	config = gcnew CClientConfigurations();
	mappingCharts = gcnew CControlMappingCharts();

	// Calibration object
	calibration = new CCalibration();

}

// Destructor
CKinova::~CKinova()
{
	
	delete jaco;
	delete joystick;
	delete pointsTrajectory;
	delete trajectory;
	delete cVector;
	delete zoneList;
	delete trajectoryInfo;
	delete cartesianInfo;
	delete handPosition;
	delete jointPosition;
	delete config;
	delete mappingCharts;

	delete calibration;
	
}

//------------------------------------
// () NEW FUNCTIONS
//------------------------------------

///////////////////////////// BASIC FUNCTIONS ////////////////////////////////

void CKinova::Inicialize () //Sequence of actions to initialize the arm. Is required to run at the beginning.
{

	//From now on, API can move JACO and we assume the active mapping is GUI.
	StartControlAPI();

	if(IsApiInControl()) 
	{
		System::Console::WriteLine("API is in control\n"); 
		
		GoHome ();

		// Position vector initialization
		DisplayHandPosition();
		xTarget = XPosition;
		yTarget = YPosition;
		zTarget = ZPosition;
		tXTarget = ThetaX;
		tYTarget = ThetaY;
		tZTarget = ThetaZ;

	}
	else 
		System::Console::WriteLine("API is NOT in control");

}

void CKinova::OffSecuence () // Sequence of actions to turn off the arm
{
	
	Retract ();

	Console::WriteLine("Jaco go to rectract position");

}

void CKinova::MoveTo (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z) // Moves in absolute cartesian coordinates
{
	Console::WriteLine("Move to");

	activeJoystick = 1;
	xTarget = COORDINATE_X;
	yTarget = COORDINATE_Y;
	zTarget = COORDINATE_Z;

	while(!TargetReached (xTarget, yTarget, zTarget, tXTarget, tYTarget, tZTarget))
		Thread::Sleep(50);
}

void CKinova::PositionDisplace (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z) // Displacement in cartesian coordinates
{
	Console::WriteLine("Position Displace");

	activeJoystick = 1;
	xTarget = XPosition + COORDINATE_X;
	yTarget = YPosition + COORDINATE_Y;
	zTarget = ZPosition + COORDINATE_Z;

	while(!TargetReached (xTarget, yTarget, zTarget, tXTarget, tYTarget, tZTarget))
		Thread::Sleep(50);

}

void CKinova::Pick (int diameter) // Pick. Only moves the fingers
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	trajectory->UserPosition->FingerPosition[0] = diameter;
	trajectory->UserPosition->FingerPosition[1] = diameter;
	trajectory->UserPosition->FingerPosition[2] = 0;

	pointsTrajectory->Add(trajectory);

	trajectory->LimitationActive = false;

	SendTrajectoryFunctionnality();

	Console::WriteLine("Pick Object");

	WaitTrajectory ();

}

void CKinova::Drop () // Drop. Only moves the fingers
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	trajectory->UserPosition->FingerPosition[0] = 0;
	trajectory->UserPosition->FingerPosition[1] = 0;
	trajectory->UserPosition->FingerPosition[2] = 0;

	pointsTrajectory->Add(trajectory);

	trajectory->LimitationActive = false;

	SendTrajectoryFunctionnality();

	Console::WriteLine("Drop Object");

	WaitTrajectory ();

}

void CKinova::DisplayHandPosition () //This functionality display the Cartesian position of the robot’s hand.
{	

	handPosition = GetHandPosition (); 

	//Display the values
 	System::Console::WriteLine("Position X = " + XPosition); 
	System::Console::WriteLine("Position Y = " + YPosition); 
	System::Console::WriteLine("Position Z = " + ZPosition); 
 
	System::Console::WriteLine("Theta X = " + ThetaX); 
	System::Console::WriteLine("Theta Y = " + ThetaY); 
	System::Console::WriteLine("Theta Z = " + ThetaZ);


}

///////////////////////////// CONFIGURATION FUNCTIONS ////////////////////////////////

void CKinova::ChangeSpeed (float ang_Speed, float lin_Speed) // Change speed configured
{
	//Declaration and initialization of the object.
	CClientConfigurations^ Client = gcnew CClientConfigurations();

	m_semaphore.waitForSignal();
	Client = GetClientConfigurations();
	m_semaphore.release();

	Client->MaxAngularSpeed = ang_Speed;
	Client->MaxLinearSpeed = lin_Speed;
	Client->Sensibility = 100; 

	m_semaphore.waitForSignal();
	jaco->ConfigurationsManager->SetClientConfigurations(Client); 
	m_semaphore.release();
}

void CKinova::MyClientConfigurations () // Change all client configurations (Don't use if you're not sure that all parameters are corrects)
{

	//Declaration and initialization of the object.
	CClientConfigurations^ Client = gcnew CClientConfigurations();

	Client = GetClientConfigurations();
	
	//Here we set the value we need to configurate. Make sure that every value is valid with the CThreshold class.
	Client->AngleRetractedPosition = 40;
	Client->ClientName = "PseudoClient";
	Client->ClientNo = "3.14159";
	Client->ComplexRetractActive = 1;
	//Client->DeletePreProgrammedPositionsAtRetract = ?;
	Client->DrinkingDistance = 0.04f;
	Client->DrinkingHeight = 0.03f;
	//Client->DrinkingLength = ?;
	//Client->Fingers2and3inverted = ?;
	Client->Laterality = CJacoStructures::ArmLaterality::LeftHandedness;
	//Client->MaxAngularAcceleration = ?;
	Client->MaxAngularSpeed = 0.79f;
	Client->MaxForce = 15;
	//Client->MaxLinearAcceleration = ?;
	Client->MaxLinearSpeed = 0.15f;
	Client->ModelNo = "666";
	Client->Organization = "S.C.R.S.";
	//Client->RetractPositions = ?;
	//Client->Sensibility = ?; //100-1000
	//Client->SerialNo = ?;

	jaco->ConfigurationsManager->SetClientConfigurations(Client); 

}

///////////////////////////// ACTION FUNCTIONS ////////////////////////////////

void CKinova::PickObject (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, int Diameter) // Sequence of actions to pick a object (ReadytoPick start position is required)
{

	Drop ();

	MoveTo (cVector->Position[CVectorEuler::COORDINATE_X], cVector->Position[CVectorEuler::COORDINATE_Y], COORDINATE_Z +0.2f );

	MoveTo (COORDINATE_X, COORDINATE_Y, cVector->Position[CVectorEuler::COORDINATE_Z]);

	MoveTo (COORDINATE_X, COORDINATE_Y, COORDINATE_Z);

	Pick (Diameter);

	MoveTo (cVector->Position[CVectorEuler::COORDINATE_X], cVector->Position[CVectorEuler::COORDINATE_Y], COORDINATE_Z +0.12f );

}

void CKinova::DropObject (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z) // Sequence of actions to drop a object
{
	
	MoveTo (COORDINATE_X, COORDINATE_Y, COORDINATE_Z + 0.1f);

	MoveTo (COORDINATE_X, COORDINATE_Y, COORDINATE_Z);

	Drop ();

	MoveTo (cVector->Position[CVectorEuler::COORDINATE_X], cVector->Position[CVectorEuler::COORDINATE_Y], COORDINATE_Z +0.2f );

}

void CKinova::SmellObject (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, int SmellTime) // Sequence of actions to smell a object (ReadytoClamp start position is required)
{

	FingerTo (100,100,100);

	MoveTo (cVector->Position[CVectorEuler::COORDINATE_X], cVector->Position[CVectorEuler::COORDINATE_Y], COORDINATE_Z +0.1f );

	MoveTo (COORDINATE_X, COORDINATE_Y, cVector->Position[CVectorEuler::COORDINATE_Z]);

	MoveTo (COORDINATE_X, COORDINATE_Y, COORDINATE_Z);

	Console::WriteLine("Smelling");
	Thread::Sleep(SmellTime);

	MoveTo (cVector->Position[CVectorEuler::COORDINATE_X], cVector->Position[CVectorEuler::COORDINATE_Y], COORDINATE_Z +0.1f );

}

void CKinova::GiveMeYourHand () // Jaco does the action to give the hand
{

	ReadyToPick();

	Drop();

	PositionDisplace(0.0f,-0.3f,0.0f);

	Pick(90);

	PositionDisplace(0.0f,0.0f,0.075f);
	PositionDisplace(0.0f,0.0f,-0.15f);
	PositionDisplace(0.0f,0.0f,0.15f);
	PositionDisplace(0.0f,0.0f,-0.15f);
	PositionDisplace(0.0f,0.0f,0.15f);
	PositionDisplace(0.0f,0.0f,-0.075f);

	Drop();

	ReadyToPick();	

}

void CKinova::TurnHandLeft (float rad) // Roll left
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	while (rad > 3.14159265)
	{
		cVector = GetHandPosition();

		cVector->Rotation[CVectorEuler::THETA_Z] = cVector->Rotation[CVectorEuler::THETA_Z] - 3;

		Console::WriteLine("THETA_Z " + cVector->Rotation[CVectorEuler::THETA_Z]);

		rad = rad - 3;

		Console::WriteLine("RAD " + rad);

		trajectory->UserPosition->Position = cVector;

		trajectory->LimitationActive = false;

		pointsTrajectory->Add(trajectory);

		SendTrajectoryFunctionnality();

		Console::WriteLine("Rotation Displace");

		WaitTrajectory ();

	}


	cVector = GetHandPosition();

	cVector->Rotation[CVectorEuler::THETA_Z] = cVector->Rotation[CVectorEuler::THETA_Z] - rad;

	Console::WriteLine("THETA_Z " + cVector->Rotation[CVectorEuler::THETA_Z]);

	Console::WriteLine("RAD " + rad);

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Rotation Displace");

	WaitTrajectory ();

}

void CKinova::TurnHandRight (float rad) // Roll right
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	while (rad > 3.14159265)
	{
		cVector = GetHandPosition();

		cVector->Rotation[CVectorEuler::THETA_Z] = cVector->Rotation[CVectorEuler::THETA_Z] + 3;

		Console::WriteLine("THETA_Z " + cVector->Rotation[CVectorEuler::THETA_Z]);

		rad = rad - 3;

		Console::WriteLine("RAD " + rad);

		trajectory->UserPosition->Position = cVector;

		trajectory->LimitationActive = false;

		pointsTrajectory->Add(trajectory);

		SendTrajectoryFunctionnality();

		Console::WriteLine("Rotation Displace");

		WaitTrajectory ();
	}

	cVector = GetHandPosition();

	cVector->Rotation[CVectorEuler::THETA_Z] = cVector->Rotation[CVectorEuler::THETA_Z] + rad;

	Console::WriteLine("THETA_Z " + cVector->Rotation[CVectorEuler::THETA_Z]);

	Console::WriteLine("RAD " + rad);

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Rotation Displace");

	WaitTrajectory ();

}

/////////////////////////// STARTING POSITIONS ////////////////////////////////

void CKinova::GoHome () //Arm go to home position (It's necessary run at least once to move the arm)
{

	Console::WriteLine("Jaco go to home position");

	GetHandPosition();

	while (!TargetReached(0.137f,-0.515f,0.215f,-1.559f,0.508f,-3.258f))
	{
		//Default mapping for the API is that the third button(index = 2) control the retract&ready movement.
		joystick->ButtonValue[2] = 2;

		//Send a PRESS event on the third button(index 2). 
		SendJoystickFunctionality();
	
		//We wait to simulate someone holding the button. 
		Thread::Sleep(50);

		GetHandPosition();
	}

	//Send a RELEASE event on the third button(index 2).
	joystick->ButtonValue[2] = 0;
	SendJoystickFunctionality();

}

void CKinova::Retract () //Arm go to retract position
{
	
	PositionDisplace (0.1,0,0);

	GoHome ();

	MoveToAngle (90, 43, 20, 0, 0, 0, 100, 100, 100);

}

void CKinova::ReadyToPick () // Arm configuration to pick an object
{

	AddCompleteTrajectory (0.0f,-0.5f,0.2f, -1.5136f, 0.47108f, -3.1852f, 100, 100, 100);

}

void CKinova::ReadyToOdometry () // Arm configuration to pick an object
{

	AddCompleteTrajectory (-0.032f, -0.545f, 0.23f, -2.62, 1.1f, -2.13f, 50, 50, 50);

}

void CKinova::ReadyToPush () // Arm configuration to push something
{

	AddCompleteTrajectory (0.0f,-0.5f,0.4f, -1.5136f, 0.47108f, -3.1852f, 0, 100, 0);

}

void CKinova::ReadyToClamp () // Arm configuration to clamp
{

	AddCompleteTrajectory (0.0f,-0.5f,0.2f, 3.14f, 0.0f, -3.14f, 100, 100, 100);

	Console::WriteLine("Clamp Position");

}

void CKinova::ReadyToCalibrate () // Arm configuration to calibrate arm with camera
{

	AddCompleteTrajectory (0.0f,-0.5f,0.2f, -1.57f, 0.47f, -3.14f, 100, 100, 0);
	AddCompleteTrajectory (0.0f,-0.5f,0.2f, -1.57f, 0.47f, -1.0f, 100, 100, 0);
	AddCompleteTrajectory (0.0f,-0.5f,0.2f, -1.57f, 0.0f, -1.0f, 100, 100, 0);


	Console::WriteLine("Calibrate Position");

}

///////////////////////////// OTHER FUNCTIONS ////////////////////////////////

bool CKinova::TargetReached (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, float THETA_X, float THETA_Y, float THETA_Z) // Return true if absolute position is reached
{
	bool target = false;
	float error_traslation;
	float error_rotation;
	float erot1, erot2, erot3;

	error_traslation = abs(COORDINATE_X - XPosition) + abs(COORDINATE_Y - YPosition) + abs(COORDINATE_Z - ZPosition);
	erot1 = abs(THETA_X - ThetaX);
	if (erot1 > 3.1415)
		erot1 = 6.283 - erot1;
	erot2 = abs(THETA_Y - ThetaY);
	if (erot2 > 3.1415)
		erot2 = 6.283 - erot2;
	erot3 = abs(THETA_Z - ThetaZ);
	if (erot3 > 3.1415)
		erot3 = 6.283 - erot3;

	error_rotation = erot1 + erot2 + erot3;

	cout << endl << "Error traslation " << error_traslation << endl;
	cout << endl << "Error rotation " << error_rotation << endl;

	if ((error_traslation < 0.02) && (error_rotation < 0.1))
		target = true;

	return target;
}

bool CKinova::TargetReachedCartesian (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z) // Return true if absolute cartesian position is reached
{
	bool target = false;
	float error_traslation;

	error_traslation = abs(COORDINATE_X - XPosition) + abs(COORDINATE_Y - YPosition) + abs(COORDINATE_Z - ZPosition);

	cout << endl << "Error traslation " << error_traslation << endl;

	if (error_traslation < 0.02)
		target = true;

	return target;
}

void CKinova::Block () // Block the arm
{
	
	System::Console::WriteLine("-------------BLOCK-------------");

	StopControlAPI();
	
}

void CKinova::DisplayJointPositions () //This functionality display the values of all Jaco’s actuators angles.
{
	System::Console::WriteLine("ENTRA");
	jointPosition = GetJointPositions ();

	//Display the values
	System::Console::WriteLine("Angle value of Joint #1 = " + jointPosition->Angle[CVectorAngle::JOINT_1]);
	System::Console::WriteLine("Angle value of Joint #2 = " + jointPosition->Angle[CVectorAngle::JOINT_2]);
	System::Console::WriteLine("Angle value of Joint #3 = " + jointPosition->Angle[CVectorAngle::JOINT_3]);
	System::Console::WriteLine("Angle value of Joint #4 = " + jointPosition->Angle[CVectorAngle::JOINT_4]);
	System::Console::WriteLine("Angle value of Joint #5 = " + jointPosition->Angle[CVectorAngle::JOINT_5]);
	System::Console::WriteLine("Angle value of Joint #6 = " + jointPosition->Angle[CVectorAngle::JOINT_6]);

}

///////////////////////////// USE KINOVA FUNCTIONS ////////////////////////////////

void CKinova::AddCompleteTrajectory (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, float THETA_X, float THETA_Y, float THETA_Z, int FingerPosition_0, int FingerPosition_1, int FingerPosition_2) // Moves all in cartesian mode (with Kinova Functions)
{

	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	cVector->Position[CVectorEuler::COORDINATE_X]=COORDINATE_X;	
	cVector->Position[CVectorEuler::COORDINATE_Y]=COORDINATE_Y;	
	cVector->Position[CVectorEuler::COORDINATE_Z]=COORDINATE_Z;

	cVector->Rotation[CVectorEuler::THETA_X]=THETA_X;	
	cVector->Rotation[CVectorEuler::THETA_Y]=THETA_Y;	
	cVector->Rotation[CVectorEuler::THETA_Z]=THETA_Z;

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;

	trajectory->UserPosition->FingerPosition[0] = FingerPosition_0;
	trajectory->UserPosition->FingerPosition[1] = FingerPosition_1;
	trajectory->UserPosition->FingerPosition[2] = FingerPosition_2;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Move To Path");

	WaitTrajectory (); 

}

void CKinova::PositionDisplaceK (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z) // Displace in cartesian coordinates (with Kinova Functions)
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	cVector->Position[CVectorEuler::COORDINATE_X] = cVector->Position[CVectorEuler::COORDINATE_X] + COORDINATE_X;	
	cVector->Position[CVectorEuler::COORDINATE_Y] = cVector->Position[CVectorEuler::COORDINATE_Y] + COORDINATE_Y;	
	cVector->Position[CVectorEuler::COORDINATE_Z] = cVector->Position[CVectorEuler::COORDINATE_Z] + COORDINATE_Z;

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Position Displace");

	WaitTrajectory ();

}

void CKinova::MoveToK (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z) // Moves in absolute cartesian coordinates (with Kinova Functions)
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	cVector->Position[CVectorEuler::COORDINATE_X]=COORDINATE_X;	
	cVector->Position[CVectorEuler::COORDINATE_Y]=COORDINATE_Y;	
	cVector->Position[CVectorEuler::COORDINATE_Z]=COORDINATE_Z;

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Move To Path");

	WaitTrajectory ();

}

void CKinova::MoveToAngle (int JOINT_1, int JOINT_2, int JOINT_3, int JOINT_4, int JOINT_5, int JOINT_6, int FingerPosition_0, int FingerPosition_1, int FingerPosition_2)
{
	
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	jointPosition->Angle[CVectorAngle::JOINT_1] = JOINT_1;	
	jointPosition->Angle[CVectorAngle::JOINT_2] = JOINT_2;	
	jointPosition->Angle[CVectorAngle::JOINT_3] = JOINT_3;
	jointPosition->Angle[CVectorAngle::JOINT_4] = JOINT_4;	
	jointPosition->Angle[CVectorAngle::JOINT_5] = JOINT_5;	
	jointPosition->Angle[CVectorAngle::JOINT_6] = JOINT_6;

	trajectory->UserPosition->AnglesJoints = jointPosition;

	trajectory->LimitationActive = false;

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::AngularPosition;

	trajectory->UserPosition->FingerPosition[0] = FingerPosition_0;
	trajectory->UserPosition->FingerPosition[1] = FingerPosition_1;
	trajectory->UserPosition->FingerPosition[2] = FingerPosition_2;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Move To Path");

	WaitTrajectory ();

}

void CKinova::RotationTo (float THETA_X, float THETA_Y, float THETA_Z) //  Moves all in angular coordinates (with Kinova Functions)
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	cVector->Rotation[CVectorEuler::THETA_X]=THETA_X;	
	cVector->Rotation[CVectorEuler::THETA_Y]=THETA_Y;	
	cVector->Rotation[CVectorEuler::THETA_Z]=THETA_Z;

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Rotation To Path");

	WaitTrajectory ();

}

void CKinova::FingerTo (int FingerPosition_0, int FingerPosition_1, int FingerPosition_2) // Moves fingers
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	trajectory->UserPosition->FingerPosition[0] = FingerPosition_0;
	trajectory->UserPosition->FingerPosition[1] = FingerPosition_1;
	trajectory->UserPosition->FingerPosition[2] = FingerPosition_2;

	pointsTrajectory->Add(trajectory);

	trajectory->LimitationActive = false;

	SendTrajectoryFunctionnality();

	Console::WriteLine("Finger To Path");

	WaitTrajectory ();

}

void CKinova::RotationDisplace (float THETA_X, float THETA_Y, float THETA_Z) // Displacement in angular coordinates
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	cVector->Rotation[CVectorEuler::THETA_X] = cVector->Rotation[CVectorEuler::THETA_X] + THETA_X;	
	cVector->Rotation[CVectorEuler::THETA_Y] = cVector->Rotation[CVectorEuler::THETA_Y] + THETA_Y;	
	cVector->Rotation[CVectorEuler::THETA_Z] = cVector->Rotation[CVectorEuler::THETA_Z] + THETA_Z;

	trajectory->UserPosition->Position = cVector;

	trajectory->LimitationActive = false;

	pointsTrajectory->Add(trajectory);

	SendTrajectoryFunctionnality();

	Console::WriteLine("Rotation Displace");

	WaitTrajectory ();

}

void CKinova::FingerDisplace (int FingerPosition_0, int FingerPosition_1, int FingerPosition_2) // Displacement
{

	trajectory->UserPosition->PositionType = CJacoStructures::PositionType::CartesianPosition;
	trajectory->UserPosition->HandMode = CJacoStructures::HandMode::PositionMode;

	trajectory->UserPosition->FingerPosition[0] = trajectory->UserPosition->FingerPosition[0] + FingerPosition_0;
	trajectory->UserPosition->FingerPosition[1] = trajectory->UserPosition->FingerPosition[1] + FingerPosition_1;
	trajectory->UserPosition->FingerPosition[2] = trajectory->UserPosition->FingerPosition[2] + FingerPosition_2;

	pointsTrajectory->Add(trajectory);

	trajectory->LimitationActive = false;

	SendTrajectoryFunctionnality();

	Console::WriteLine("Finger Displace");

	WaitTrajectory ();

}

bool CKinova::WaitTrajectory () // Waits for a movement finished
{

	bool result = false;

	Console::WriteLine("Wait Trajectory");

		m_semaphore.waitForSignal();

	while(GetInfoFIFOTrajectory()->StillInFIFO > 0) 
	{
		m_semaphore.release();

		Thread::Sleep(50);

		m_semaphore.waitForSignal();
	}

	m_semaphore.release();

	Console::WriteLine("Path Reached");

	Thread::Sleep(10);

	result = true;

	return result;
	
}

/////////////////////////// DEMOSTRATIONs ////////////////////////////////

void CKinova::Demostration1 () // Example to switch betweeen Cartesian and Angular modes
{
	
	AddCompleteTrajectory (0.0f,-0.6f,0.2f, -1.5136f, 0.47108f, -3.1852f, 100, 100, 100);

	Thread::Sleep(500);

	MoveToAngle (180, 90, 45, 0, 0, 0, 100, 100, 100);
				
	Retract ();

	System::Console::WriteLine("Retract Position"); 
				
	MoveToAngle (90, 90, 45, 0, 0, 0, 100, 100, 100);
	MoveToAngle (90, 90, 45, 90 , 0, 0, 100, 100, 100);
				
	GoHome();

	AddCompleteTrajectory (0.0f,-0.6f,0.2f, -1.5136f, 0.47108f, -3.1852f, 100, 100, 100);
	AddCompleteTrajectory (0.0f,-0.6f,0.2f, -1.5136f, 0.47108f, -3.1852f, 100, 100, 100);

}

void CKinova::Demostration2 () //Example of pick and drop object
{

	ReadyToPick ();
	PickObject (-0.6f,0.0,-0.05f, 30);
	MoveTo (0.0f,-0.5,0.1f);
	DropObject (0.6f,0.0,-0.05f);
	ReadyToPick ();	

}

void CKinova::Demostration3 () //Example to push a button
{

	//ReadyToPush ();	
	PositionDisplace (0.0f,-0.1f,0.0f);
	PositionDisplace (0.0f,0.1f,0.0f);

}

void CKinova::Demostration4 () //Example to displaces
{

	RotationDisplace (0.0f,0.0f,20.0f);
	RotationDisplace (0.0f,0.0f,-20.0f);

	FingerDisplace (0.0f,0.0f,100.0f);
	FingerDisplace (0.0f,0.0f,-100.0f);

	PositionDisplace (0.0f,0.0f,0.1f);
	PositionDisplace (0.0f,0.0f,-0.1f);

}

void CKinova::Demostration5 () // Example
{

	ReadyToPick ();

	PickObject (-0.6f,-0.1f,-0.05f, 30);
	DropObject (-0.6f,0.1f,0.15f);

	PickObject (-0.6f,0.3f,-0.05f, 30);
	DropObject (-0.6f,0.1f,0.15f);

	PickObject (-0.6f,0.5f,0.08f, 30);
	DropObject (-0.6f,0.1f,0.15f);

	MoveTo (0.0f,-0.5f,0.1f);
	PickObject (0.6f,0.0,-0.05f, 30);
	MoveTo (0.0f,-0.5f,0.1f);
	DropObject (-0.6f,0.1f,0.15f);

	PickObject (-0.6f,0.1f,-0.05f, 30);
	DropObject (-0.4f,-0.3f,0.0f);

}

bool CKinova::TestFingers () // Example to see finger positions
{

	FingerTo (0,0,0);
	FingerTo (10,10,10);
	FingerTo (0,0,0);
	FingerTo (20,20,20);
	FingerTo (0,0,0);
	FingerTo (30,30,30);
	FingerTo (0,0,0);
	FingerTo (40,40,40);
	FingerTo (0,0,0);
	FingerTo (50,50,50);
	FingerTo (0,0,0);
	FingerTo (60,60,60);
	FingerTo (0,0,0);
	FingerTo (70,70,70);
	FingerTo (0,0,0);
	FingerTo (80,80,80);
	FingerTo (0,0,0);
	FingerTo (90,90,90);
	FingerTo (0,0,0);
	FingerTo (100,100,100);
	FingerTo (0,0,0);
	
	return true;

}

void CKinova::DemostrationVideo () //CAMBIAR POR LA FUNCION EN RHODON
{
	//Se prepara para oler
	ReadyToClamp();
	
	SmellObject (0.4f, -0.5f, 0.05f,3000);

	SmellObject (0.2f, -0.5f, 0.05f,3000);

	//Coge el primer vaso 
	ReadyToPick();

	PickObject (0.2f, -0.5f, -0.01f, 30);

	MoveTo (-0.54f,-0.5f, 0.13f);

	//Vuelca el contenido

	TurnHandRight (1.8);

	Thread::Sleep(1500);

	TurnHandLeft (1.8);

	//Suelta el vaso

	DropObject (0.2f, -0.5f, -0.01f);

	ReadyToClamp();

	//Huele los siguientes
	
	SmellObject (0.0f, -0.5f, 0.05f,3000);

	SmellObject (-0.2f, -0.5f, 0.05f,3000);

	SmellObject (-0.4f, -0.5f, 0.05f,3000);

	//Coge el siguiente, lo vuelca y lo deja
	ReadyToPick();

	PickObject (-0.4f, -0.5f, -0.01f, 30);

	MoveTo (-0.54f,-0.5f, 0.13f);

	TurnHandRight (1.8);

	Thread::Sleep(1500);

	TurnHandLeft (1.8);

	DropObject (-0.4f, -0.5f, -0.0f);

	//Coge la pajita y la suelta

	PickObject (0.0f, -0.65f, 0.06f, 100);

	DropObject (-0.57f,-0.5f, 0.13f);

	//Coge el vaso para llevarselo

	PickObject (-0.6f,-0.5f, 0.01f, 30);

	MoveTo (-0.0f,-0.4f, 0.2f);


}

/////////////////////////// ARM CALIBRATION ////////////////////////////////

int CKinova::CalibrateArm ()
{
	//-------------------------------------
	// Variable declarations
	//-------------------------------------
	
	VideoCapture capture;			//Class for video capturing from video files or cameras

	Mat view, view2, view3;			//OpenCV C++ matrix class.

	Size boardSize, imageSize;		//Template class for specfying image or rectangle size
	float aspectRatio = 1.f;
	boardSize.width = boardSizewidth;
	boardSize.height = boardSizeheight;

	bool writeExtrinsics = true, writePoints = true;
	const char* outputFilename = "out_camera_data.yml";

    int flags = 0;
    bool showUndistorted = false;
    bool videofile = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F), distCoeffs = Mat::zeros(5, 1, CV_64F);
	int i, nframes = framesEX, j = 0;
	bool found;
	bool blink = false;

	//-------------------------------------
	// Calibrate Positions
	//-------------------------------------
	float Calib_Pos [81] [9] = {	{ 0.20f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
	 
									{ 0.20f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},

									{ 0.20f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0}};


	//-------------------------------------
	// Load & show 
	//-------------------------------------

	ifstream fp("CameraMatrixAndDistortion.txt");

	// Load Camera Matrix
	for (int l=0;l<3;l++)
		for (int m=0;m<3;m++)
			fp >> cameraMatrix.at<double>(l,m);
	
	// Load distorsion coefficients
	for (int l=0;l<5;l++)
		fp >> distCoeffs.at<double>(l,0);

	fp.close();
	
	cout << "\n\nCamera Matrix:\n" << cameraMatrix <<  "\n";
	cout << "\nDistorsion coefficients:\n" << distCoeffs <<  "\n";


	//-------------------------------------
	// Open and Set Camera
	//-------------------------------------

	ReadyToCalibrate();			//Arm go to calibrate position

	capture.open(cameraId);		//Open video file or a capturing device for video capturing
								//The method first call VideoCapture::release() to close the already opened file or camera.

	if(!capture.isOpened())		//Returns true if video capturing has been initialized already.
		 return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
	else
    {
		namedWindow( "Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.
		namedWindow( "3D Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.

		//Sets a property in the VideoCapture. (Property identifier, Value of the property) 
		capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

		//Display cameras settings 
		cout << "\nDepth generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
				"FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

		cout << "\nImage generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;		
	}
	
	// This loop is for remove the first captures of the cameras (The lighting setting is bad)
	for (i = 0; i < 4; i++)
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
				return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone();
				undistort(view2, view, cameraMatrix, distCoeffs);

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}
		cvWaitKey(20);
	}

	//-------------------------------------
	// Loop
	//-------------------------------------

	AddCompleteTrajectory (Calib_Pos[j][0],Calib_Pos[j][1],Calib_Pos[j][2],Calib_Pos[j][3],Calib_Pos[j][4],Calib_Pos[j][5],Calib_Pos[j][6],Calib_Pos[j][7],Calib_Pos[j][8]);
	j++;


	for (i = 1;; i++) 
	{
		bool blink = false;
		int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

		if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
			return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone(); 
				undistort(view2, view, cameraMatrix, distCoeffs); ///// Se puede optimizar con otro método

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
				{
					imshow("3D Image View", view3);	//Displays the image in the specified window.
					//imageSize = view.size();
					/*cout << view3.at<Vec3f>(100,100)[0] << endl;
					cout << view3.at<Vec3f>(100,100)[1] << endl;
					cout << view3.at<Vec3f>(100,100)[2] << endl;*/
					//cout << view3 << endl;
				}
		}

		cvWaitKey(20); //This is necesary to given time to process the draw requests from cv::imshow().
	
		imageSize = view.size();

        vector<Point2f> pointbuf;

		cvWaitKey(20);


		found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		//Improve the found corners' coordinate accuracy
        if(found)
		{
			cornerSubPix(view, pointbuf, Size(11,11),Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
		}

        if( mode == CAPTURING && found)
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
			cout << "Image " << j << " is captured\n\n";				
        }

        string msg = mode == CAPTURING ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        
		int baseLine = 0;

        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);        
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
			msg = cv::format( "%d/%d", (int)imagePoints.size(), nframes );

        putText( view, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

		imshow("Image View", view);

        if( (key & 255) == 27 )
            break;
 

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
			cout << "\n\nCollecting data and calculating the matrix. Please be patient.\n\n";
            if( runAndSave(outputFilename, imagePoints, imageSize,
                       boardSize, pattern, squareSize, aspectRatio,
                       flags, cameraMatrix, distCoeffs,
                       writeExtrinsics, writePoints))
			{
                mode = CALIBRATED;		
				destroyWindow("Image View");	// Destroys the window with the given name.
				destroyWindow("3D Image View"); // Destroys the window with the given name.
				calibration->CalculateMatrix();	
				TransformationPoint();
			}

            if( mode == CALIBRATED)
                break;
        }

		if( mode == CAPTURING && found && j!= framesEX)
        {
			AddCompleteTrajectory (Calib_Pos[j][0],Calib_Pos[j][1],Calib_Pos[j][2],Calib_Pos[j][3],Calib_Pos[j][4],Calib_Pos[j][5],Calib_Pos[j][6],Calib_Pos[j][7],Calib_Pos[j][8]);
			j++;
		}

    }

	GoHome();
	
	return 0;
	
}

int CKinova::CalibrateArmDepth ()
{
	//-------------------------------------
	// Variable declarations
	//-------------------------------------
	
	VideoCapture capture;			//Class for video capturing from video files or cameras

	Mat view, view2, view3;			//OpenCV C++ matrix class.

	Size boardSize, imageSize;		//Template class for specfying image or rectangle size
	float aspectRatio = 1.f;
	boardSize.width = boardSizewidth;
	boardSize.height = boardSizeheight;

	bool writeExtrinsics = true, writePoints = true;
	const char* outputFilename = "out_camera_data.yml";
	ofstream fp2("ArmCamPoints.txt");

    int flags = 0;
    bool showUndistorted = false;
    bool videofile = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F), distCoeffs = Mat::zeros(5, 1, CV_64F);
	int i, nframes = framesEX, j = 0;
	bool found;
	bool blink = false;

	double camPoints[180][4];

	//-------------------------------------
	// Calibrate Positions
	//-------------------------------------
	float Calib_Pos [74] [9] = {	{ 0.20f,-0.4f, 0.11f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.18f,-0.42f, 0.12f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.16f,-0.4f, 0.13f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.42f, 0.12f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.02f,-0.4f, 0.11f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.42f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.09f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.42f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									//{-0.20f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									//{-0.20f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.11f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.09f,-0.43f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.01f,-0.43f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.02f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.43f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.38f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.43f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									//{-0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
	 
									//{ 0.20f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.51f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.07f,-0.52f, 0.12f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.53f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.54f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.55f, 0.12f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.56f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.56f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.55f, 0.12f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.54f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.53f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.53f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.53f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.53f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.53f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.53f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.55f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.55f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.55f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.55f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.55f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.55f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									//{-0.20f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},

									//{ 0.20f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0}
									//{-0.20f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0}
									};


	//-------------------------------------
	// Load & show 
	//-------------------------------------

	ifstream fp("CameraMatrixAndDistortion.txt");

	// Load Camera Matrix
	for (int l=0;l<3;l++)
		for (int m=0;m<3;m++)
			fp >> cameraMatrix.at<double>(l,m);
	
	// Load distorsion coefficients
	for (int l=0;l<5;l++)
		fp >> distCoeffs.at<double>(l,0);

	fp.close();
	
	cout << "\n\nCamera Matrix es:\n" << cameraMatrix <<  "\n";
	cout << "\nDistorsion coefficients:\n" << distCoeffs <<  "\n";


	//-------------------------------------
	// Open and Set Camera
	//-------------------------------------

	ReadyToCalibrate();			//Arm go to calibrate position

	capture.open(cameraId);		//Open video file or a capturing device for video capturing
								//The method first call VideoCapture::release() to close the already opened file or camera.

	if(!capture.isOpened())		//Returns true if video capturing has been initialized already.
		 return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
	else
    {
		namedWindow( "Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.
		namedWindow( "3D Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.

		//Sets a property in the VideoCapture. (Property identifier, Value of the property) 
		capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

		//Display cameras settings 
		cout << "\nDepth generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
				"FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

		cout << "\nImage generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;		
	}
	
	// This loop is for remove the first captures of the cameras (The lighting setting is bad)
	for (i = 0; i < 4; i++)
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
				return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone();
				undistort(view2, view, cameraMatrix, distCoeffs);

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}
		cvWaitKey(20);
	}

	//-------------------------------------
	// Loop
	//-------------------------------------

	AddCompleteTrajectory (Calib_Pos[j][0],Calib_Pos[j][1],Calib_Pos[j][2],Calib_Pos[j][3],Calib_Pos[j][4],Calib_Pos[j][5],Calib_Pos[j][6],Calib_Pos[j][7],Calib_Pos[j][8]);
	j++;


	for (i = 1;; i++) 
	{
		bool blink = false;
		int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

		if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
			return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone(); 
				undistort(view2, view, cameraMatrix, distCoeffs); ///// Se puede optimizar con otro método

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}

		cvWaitKey(20); //This is necesary to given time to process the draw requests from cv::imshow().
	
		imageSize = view.size();

        vector<Point2f> pointbuf;

		cvWaitKey(20);


		found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		//Improve the found corners' coordinate accuracy
        if(found)
		{
			cornerSubPix(view, pointbuf, Size(11,11),Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			drawChessboardCorners( view, boardSize, Mat(pointbuf), found );

			//cout << endl << "POINTBUF" << pointbuf << endl;

			//cout << (int)pointbuf[0].x << endl;
			
			if (mode == CAPTURING)
			{
				camPoints[j-1][0] = view3.at<Vec3f>((int)pointbuf[0].y,(int)pointbuf[0].x)[0];
				camPoints[j-1][1] = view3.at<Vec3f>((int)pointbuf[0].y,(int)pointbuf[0].x)[1];
				camPoints[j-1][2] = view3.at<Vec3f>((int)pointbuf[0].y,(int)pointbuf[0].x)[2];
				camPoints[j-1][3] = 1;

				cout << endl << "The camPoint is: " << camPoints[j-1][0] << " "<< camPoints[j-1][1] << " " << camPoints[j-1][2] << endl;
			}
		}

        if( mode == CAPTURING && found)
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
			cout << "Image " << j << " is captured\n\n";				
        }

        string msg = mode == CAPTURING ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        
		int baseLine = 0;

        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);        
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
			msg = cv::format( "%d/%d", (int)imagePoints.size(), nframes );

        putText( view, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

		imshow("Image View", view);

        if( (key & 255) == 27 )
            break;
 

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
			cout << "\n\nCollecting data and calculating the matrix. Please be patient.\n\n";
            
			
            mode = CALIBRATED;		
			destroyWindow("Image View");	// Destroys the window with the given name.
			destroyWindow("3D Image View"); // Destroys the window with the given name.
		
			for (int p = 0; p<j ; p++)
			{
				for (int q = 0; q<3 ; q++)
					cout << camPoints[p][q] << " ";

				cout << endl;
			}

			for (int x = 0; x < framesEX ;x++)
			{
				fp2 << Calib_Pos[x][0] << ' ';
				fp2 << Calib_Pos[x][1] << ' ';
				fp2 << Calib_Pos[x][2] << ' ';
				fp2 << camPoints[x][0] << ' ';
				fp2 << camPoints[x][1] << ' ';
				fp2 << camPoints[x][2] << endl;
			}
			cout << "Sale del for" << endl;
			cvWaitKey(20);

			fp2.close();

			calibration->CalculateMatrix();	
			TransformationPoint();

            if( mode == CALIBRATED)
                break;
        }

		if( mode == CAPTURING && found && j!= framesEX)
        {
			AddCompleteTrajectory (Calib_Pos[j][0],Calib_Pos[j][1],Calib_Pos[j][2],Calib_Pos[j][3],Calib_Pos[j][4],Calib_Pos[j][5],Calib_Pos[j][6],Calib_Pos[j][7],Calib_Pos[j][8]);
			j++;
		}

    }

	GoHome();
	
	return 0;
	
}

int CKinova::IntrinsicCalibration ()
{
	//-------------------------------------
	// Variable declarations
	//-------------------------------------
	
	VideoCapture capture;		//Class for video capturing from video files or cameras

	Mat view, view2;			//OpenCV C++ matrix class.

	IplImage image = view;

	Size boardSize, imageSize;	//Template class for specfying image or rectangle size
	float aspectRatio = 1.f;
	boardSize.width = boardSizewidth;
	boardSize.height = boardSizeheight;

	bool writeExtrinsics = true, writePoints = true;
	const char* outputFilename = "out_camera_data.yml";

	ofstream fp("CameraMatrixAndDistortion.txt");

	int flags = 0;
    bool showUndistorted = false;
    bool videofile = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
	Mat cameraMatrix, distCoeffs;
	int i, nframes = framesIC, j = 0;
	bool found;

	bool blink = false;

	//-------------------------------------
	// Open and Set Camera
	//-------------------------------------

	capture.open(cameraId);		//Open video file or a capturing device for video capturing
								//The method first call VideoCapture::release() to close the already opened file or camera.

	if(!capture.isOpened())		//Returns true if video capturing has been initialized already.
		 return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
	else
    {
		namedWindow( "Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.

		//Sets a property in the VideoCapture. (Property identifier, Value of the property) 
		capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ  );




		cout << "\nDepth generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
				"FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

		cout << "\nImage generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;		
	}
	
	for (i = 0; i < 4; i++)
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
				return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
			else
			{
				if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE))//CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
				{
					imshow("Image View", view);	//Displays the image in the specified window.
					imageSize = view.size();

					

				}
			}

			cvWaitKey(20);
	}

	//-------------------------------------
	// Loop
	//-------------------------------------

	for (i = 1;; i++) //// Debe acabar con la calibración
	{
		bool blink = false;

		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
			return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}
		}

		cvWaitKey(20); //This is necesary to given time to process the draw requests from cv::imshow().
	
		imageSize = view.size();

        vector<Point2f> pointbuf;

		cvWaitKey(20);

		
		found = findChessboardCorners( view, boardSize, pointbuf,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);


       // improve the found corners' coordinate accuracy
        if(found) 
			cornerSubPix(view, pointbuf, Size(11,11),Size(-1,-1), 
						 TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

        if( mode == CAPTURING && found && (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();

			image = view;

			// GUARDAR IMAGEN
			std::string str = mrpt::utils::format("img_%d.png",j);
			const char * c = str.c_str();
			cvSaveImage(c, &image);

			j++;

			System::Console::WriteLine("CAPTURE");			
        }
        
		// Draw the corners.
        if(found)
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
		
		//----------------------------- Output Text -------------------------
        string msg = mode == CAPTURING ? "100/100" :
            mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        
		int baseLine = 0;

        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);        
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
			msg = cv::format( "%d/%d", (int)imagePoints.size(), nframes );

        putText( view, msg, textOrigin, 1, 1,
                 mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

		imshow("Image View", view);
		int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

        if( (key & 255) == 27 )
            break;
        
        if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
            if( runAndSave(outputFilename, imagePoints, imageSize,
                       boardSize, pattern, squareSize, aspectRatio,
                       flags, cameraMatrix, distCoeffs,
                       writeExtrinsics, writePoints))
			{
                mode = CALIBRATED;
				fp << cameraMatrix.at<double>(0,0) << " ";
				fp << cameraMatrix.at<double>(0,1) << " ";
				fp << cameraMatrix.at<double>(0,2) << " ";
				fp << cameraMatrix.at<double>(1,0) << " ";
				fp << cameraMatrix.at<double>(1,1) << " ";
				fp << cameraMatrix.at<double>(1,2) << " ";
				fp << cameraMatrix.at<double>(2,0) << " ";
				fp << cameraMatrix.at<double>(2,1) << " ";
				fp << cameraMatrix.at<double>(2,2) << "\n";
				fp << distCoeffs.at<double>(0,0) << " ";
				fp << distCoeffs.at<double>(1,0) << " ";
				fp << distCoeffs.at<double>(2,0) << " ";
				fp << distCoeffs.at<double>(3,0) << " ";
				fp << distCoeffs.at<double>(4,0) << "\n";
				fp.close();
				printf("Ha entrado\n");
				destroyWindow("Image View"); // Destroys the window with the given name.
			}
        }


		//------------------------- Video capture  output  undistorted ------------------------------
		if( mode == CALIBRATED )//&& s.showUndistorsed )
		{
			view2 = view.clone();
			undistort(view, view2, cameraMatrix, distCoeffs);
		
			//------------------------------ Show image and check for input commands -------------------
			imshow("Image undistorted", view2);

		}

    }
	
	return 0;
	
}

void CKinova::TransformationPoint ()
{
	float transMatrix [4][4];
	double camera_Pos [4][100];
	double arm_Pos [4][100];
	double result [4][100];

	ifstream fp("TransformationMatrix.txt");
	ifstream fp2("ArmCamPoints.txt");
	ofstream fp3("EquivalenceTable.txt");
	
	
	//Load the transformation matrix
	for (int i = 0; i < 4 ;i++)
	{
		for (int j = 0; j < 4 ;j++)
		{
			fp >> transMatrix[i][j];
		}
	}

	// Load the camera and arm positions
	for (int i = 0; i < framesEX ; i++)
	{
		for (int j = 0; j < 3 ;j++)
		{
			fp2 >> arm_Pos[j][i];
		}
		arm_Pos[3][i]=1;

		for (int j = 0; j < 3 ;j++)
		{
			fp2 >> camera_Pos[j][i];
		}
		camera_Pos[3][i]=1;
	}

	fp.close();
	fp2.close();

	//Calculation of the resulting points 
	for (int i = 0; i < framesEX ; i++)
		for (int j = 0; j < 4 ; j++)
		{
			result[j][i] = transMatrix[j][0]*camera_Pos[0][i]+ transMatrix[j][1]*camera_Pos[1][i]+transMatrix[j][2]*camera_Pos[2][i]+ transMatrix[j][3]*camera_Pos[3][i];
		}

	//Save equivalence table
	for (int i = 0; i < framesEX ;i++)
	{
		for (int j = 0; j < 3 ;j++)
		{
			fp3 << arm_Pos[j][i] << "\t";
			fp3 << result[j][i] << "\t";
		}
		fp3 << "\n";
	}

	fp3.close();

	
	//Display Points
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;
	mrpt::gui::CDisplayWindow3D	win("Points",640,480);
	mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();

	mrpt::opengl::CPointCloudPtr arm_points = mrpt::opengl::CPointCloud::Create();
	arm_points->setColor(1,0,0);
	arm_points->enablePointSmooth();
	arm_points->setPointSize(6.0);
	
	for (int i=0; i<framesEX; i++)
	{
		arm_points->insertPoint(arm_Pos[0][i], arm_Pos[1][i], arm_Pos[2][i]);
	}
	scene->insert( arm_points );

	mrpt::opengl::CPointCloudPtr camera_points = mrpt::opengl::CPointCloud::Create();
	camera_points->setColor(0,0,1);
	camera_points->enablePointSmooth();
	camera_points->setPointSize(6.0);
	
	for (int i=0; i<framesEX; i++)
	{
		camera_points->insertPoint(result[0][i], result[1][i], result[2][i]);
	}
	scene->insert( camera_points );

	mrpt::opengl::CGridPlaneXYPtr grid = mrpt::opengl::CGridPlaneXY::Create(-2,2,-2,2,0,0.4);
	scene->insert( grid );

	mrpt::system::sleep(10);
	
	win.unlockAccess3DScene();
	win.repaint();

	mrpt::system::os::getch();

}

int CKinova::LocatePattern ()
{
	//-------------------------------------
	// Variable declarations
	//-------------------------------------
	
	VideoCapture capture;			//Class for video capturing from video files or cameras

	Mat view, view2, view3;			//OpenCV C++ matrix class.

	Size boardSize, imageSize;		//Template class for specfying image or rectangle size
	float aspectRatio = 1.f;
	boardSize.width = boardSizewidth;
	boardSize.height = boardSizeheight;

	bool writeExtrinsics = true, writePoints = true;
	const char* outputFilename = "out_camera_data.yml";

    int flags = 0;
    bool showUndistorted = false;
    bool videofile = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F), distCoeffs = Mat::zeros(5, 1, CV_64F);
	int i, nframes = 10, j = 0, l = 0;
	bool found;
	bool blink = false;

	ofstream fp2("CamPoints.txt");


	
	//-------------------------------------
	// Load & show 
	//-------------------------------------

	ifstream fp("CameraMatrixAndDistortion.txt");

	// Load Camera Matrix
	for (int l=0;l<3;l++)
		for (int m=0;m<3;m++)
			fp >> cameraMatrix.at<double>(l,m);
	
	// Load distorsion coefficients
	for (int l=0;l<5;l++)
		fp >> distCoeffs.at<double>(l,0);

	fp.close();
	
	cout << "\n\nCamera Matrix es:\n" << cameraMatrix <<  "\n";
	cout << "\nDistorsion coefficients son:\n" << distCoeffs <<  "\n";


	//-------------------------------------
	// Open and Set Camera
	//-------------------------------------

	//ReadyToCalibrate();		//Arm go to calibrate position

	capture.open(cameraId);		//Open video file or a capturing device for video capturing
								//The method first call VideoCapture::release() to close the already opened file or camera.

	if(!capture.isOpened())		//Returns true if video capturing has been initialized already.
		 return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
	else
    {
		namedWindow( "Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.
		namedWindow( "3D Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.

		//Sets a property in the VideoCapture. (Property identifier, Value of the property) 
		capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

		//Display cameras settings 
		cout << "\nDepth generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
				"FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

		cout << "\nImage generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;		
	}
	
	// This loop is for remove the first captures of the cameras (The lighting setting is bad)
	for (i = 0; i < 4; i++)
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
				return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone();
				undistort(view2, view, cameraMatrix, distCoeffs);

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}
		cvWaitKey(20);
	}

	//-------------------------------------
	// Loop
	//-------------------------------------

	//ReadyToCalibrate();
	//AddCompleteTrajectory (0.20f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0);
	//AddCompleteTrajectory (-0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0);
	//j++;


	for (i = 1;; i++) 
	{
		bool blink = false;
		int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

		if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }


		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
			return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone(); 
				undistort(view2, view, cameraMatrix, distCoeffs); 

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
				{
					imshow("3D Image View", view3);	//Displays the image in the specified window.
					//imageSize = view.size();
					/*cout << view3.at<Vec3f>(100,100)[0] << endl;
					cout << view3.at<Vec3f>(100,100)[1] << endl;
					cout << view3.at<Vec3f>(100,100)[2] << endl;*/
					//cout << view3 << endl;
				}
		}

		cvWaitKey(20); //This is necesary to given time to process the draw requests from cv::imshow().
	
		imageSize = view.size();

        vector<Point2f> pointbuf;

		cvWaitKey(20);


		found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		//Improve the found corners' coordinate accuracy
        if(found)
		{
			cornerSubPix(view, pointbuf, Size(11,11),Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
		}

        if( mode == CAPTURING && found)
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
			cout << "Image " << l << " is captured\n\n";
			l++;

			IplImage image = view;

			// Save image
			std::string str = mrpt::utils::format("img_%d.png",l);
			const char * c = str.c_str();
			cvSaveImage(c, &image);
        }

        string msg = mode == CAPTURING ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        
		int baseLine = 0;

        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);        
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING)
		{
			msg = cv::format( "%d/%d", (int)imagePoints.size(), nframes );

		}
        putText( view, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

		imshow("Image View", view);

		Thread::Sleep(1000);

        if( (key & 255) == 27 )
            break;
 

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
			cout << "\n\nCollecting data and calculating the matrix. Please be patient.\n\n";
            //if( runAndSave(outputFilename, imagePoints, imageSize,
            //           boardSize, pattern, squareSize, aspectRatio,
            //           flags, cameraMatrix, distCoeffs,
            //           writeExtrinsics, writePoints))
			//{
			vector<Mat> rvecs, tvecs;
			vector<float> reprojErrs;
			double totalAvgErr = 0;

    
			bool ok = runCalibration(imagePoints, imageSize, boardSize, pattern, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
				
			printf("%s. avg reprojection error = %.2f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

            mode = CALIBRATED;		
			destroyWindow("Image View");	// Destroys the window with the given name.
			destroyWindow("3D Image View"); // Destroys the window with the given name.
			//calibration->CalculateMatrix();	
			//TransformationPoint();
			//}

			for (int j = 0; j < 10 ;j++)
			{
				fp2 << tvecs[j].at<double>(0,0) << ' ';
				fp2 << tvecs[j].at<double>(0,1) << ' ';
				fp2 << tvecs[j].at<double>(0,2) << endl;
			}


			fp2.close();


            if( mode == CALIBRATED)
                break;
        }

		if( mode == CAPTURING && found && j!= framesEX)
        {
			//AddCompleteTrajectory (Calib_Pos[j][0],Calib_Pos[j][1],Calib_Pos[j][2],Calib_Pos[j][3],Calib_Pos[j][4],Calib_Pos[j][5],Calib_Pos[j][6],Calib_Pos[j][7],Calib_Pos[j][8]);
			//j++;
		}

    }

	//GoHome(8000);
	
	return 0;
  
}

int CKinova::LocateOnePattern ()
{
	//-------------------------------------
	// Variable declarations
	//-------------------------------------
	
	VideoCapture capture;			//Class for video capturing from video files or cameras

	Mat view, view2, view3;			//OpenCV C++ matrix class.

	Size boardSize, imageSize;		//Template class for specfying image or rectangle size
	float aspectRatio = 1.f;
	boardSize.width = boardSizewidth;
	boardSize.height = boardSizeheight;

	bool writeExtrinsics = true, writePoints = true;
	const char* outputFilename = "out_camera_data.yml";

    int flags = 0;
    bool showUndistorted = false;
    bool videofile = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F), distCoeffs = Mat::zeros(5, 1, CV_64F);
	int i, nframes = 1, j = 0;
	bool found;
	bool blink = false;

	ofstream fp2("CamPoints.txt");


	
	//-------------------------------------
	// Load & show 
	//-------------------------------------

	ifstream fp("CameraMatrixAndDistortion.txt");

	// Load Camera Matrix
	for (int l=0;l<3;l++)
		for (int m=0;m<3;m++)
			fp >> cameraMatrix.at<double>(l,m);
	
	// Load distorsion coefficients
	for (int l=0;l<5;l++)
		fp >> distCoeffs.at<double>(l,0);

	fp.close();
	
	cout << "\n\nLa matriz de la camara es:\n" << cameraMatrix <<  "\n";
	cout << "\nLos coeficientes de distorsión son:\n" << distCoeffs <<  "\n";


	//-------------------------------------
	// Open and Set Camera
	//-------------------------------------

	//ReadyToCalibrate();		//Arm go to calibrate position

	capture.open(cameraId);		//Open video file or a capturing device for video capturing
								//The method first call VideoCapture::release() to close the already opened file or camera.

	if(!capture.isOpened())		//Returns true if video capturing has been initialized already.
		 return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
	else
    {
		namedWindow( "Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.
		namedWindow( "3D Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.

		//Sets a property in the VideoCapture. (Property identifier, Value of the property) 
		capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

		//Display cameras settings 
		cout << "\nDepth generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
				"FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

		cout << "\nImage generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;		
	}
	
	// This loop is for remove the first captures of the cameras (The lighting setting is bad)
	for (i = 0; i < 4; i++)
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
				return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone();
				undistort(view2, view, cameraMatrix, distCoeffs);

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}
		cvWaitKey(20);
	}

	//-------------------------------------
	// Loop
	//-------------------------------------

	AddCompleteTrajectory (-0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0);
	//j++;


	for (i = 1;; i++) //// 
	{
		bool blink = false;
		int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);

		if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }


		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
			return fprintf(stderr, "Could not grab image in Camera (%d)\n",cameraId), -1;
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone(); 
				undistort(view2, view, cameraMatrix, distCoeffs); ///// Se puede optimizar con otro método

				imshow("Image View", view);	//Displays the image in the specified window.
				imageSize = view.size();
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
				{
					imshow("3D Image View", view3);	//Displays the image in the specified window.
					//imageSize = view.size();
					/*cout << view3.at<Vec3f>(100,100)[0] << endl;
					cout << view3.at<Vec3f>(100,100)[1] << endl;
					cout << view3.at<Vec3f>(100,100)[2] << endl;*/
					//cout << view3 << endl;
				}
		}

		cvWaitKey(20); //This is necesary to given time to process the draw requests from cv::imshow().
	
		imageSize = view.size();

        vector<Point2f> pointbuf;

		cvWaitKey(20);


		found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		//Improve the found corners' coordinate accuracy
        if(found)
		{
			cornerSubPix(view, pointbuf, Size(11,11),Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
		}

        if( mode == CAPTURING && found)
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
			cout << "Image " << j << " is captured\n\n";				
        }

        string msg = mode == CAPTURING ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        
		int baseLine = 0;

        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);        
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING)
		{
			msg = cv::format( "%d/%d", (int)imagePoints.size(), nframes );

		}
        putText( view, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

		imshow("Image View", view);

		Thread::Sleep(1000);

        if( (key & 255) == 27 )
            break;
 

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
			cout << "\n\nCollecting data and calculating the matrix. Please be patient.\n\n";
            //if( runAndSave(outputFilename, imagePoints, imageSize,
            //           boardSize, pattern, squareSize, aspectRatio,
            //           flags, cameraMatrix, distCoeffs,
            //           writeExtrinsics, writePoints))
			//{
			vector<Mat> rvecs, tvecs;
			vector<float> reprojErrs;
			double totalAvgErr = 0;

    
			bool ok = runCalibration(imagePoints, imageSize, boardSize, pattern, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
				
			printf("%s. avg reprojection error = %.2f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

            mode = CALIBRATED;		
			destroyWindow("Image View");	// Destroys the window with the given name.
			destroyWindow("3D Image View"); // Destroys the window with the given name.
			//calibration->CalculateMatrix();	
			//TransformationPoint();
			//}

			for (int j = 0; j < 10 ;j++)
			{
				fp2 << tvecs[j].at<double>(0,0) << ' ';
				fp2 << tvecs[j].at<double>(0,1) << ' ';
				fp2 << tvecs[j].at<double>(0,2) << endl;
			}


			fp2.close();


            if( mode == CALIBRATED)
                break;
        }

		if( mode == CAPTURING && found && j!= framesEX)
        {
			//AddCompleteTrajectory (Calib_Pos[j][0],Calib_Pos[j][1],Calib_Pos[j][2],Calib_Pos[j][3],Calib_Pos[j][4],Calib_Pos[j][5],Calib_Pos[j][6],Calib_Pos[j][7],Calib_Pos[j][8]);
			//j++;
		}

    }

	//GoHome(8000);
	
	return 0;
  
}

void CKinova::ReproduceMovement ()
{
	ifstream fp("CamPoints.txt");
	ifstream fp2("TransformationMatrix.txt");

	double camPoints [100][4];
	double resPoints [100][4];
	double transformationMatrix [4][4]; 
	int k = 0;
	char c;

	//Load the transformation matrix
	for (int i = 0; i < 4 ;i++)
	{
		for (int j = 0; j < 4 ;j++)
			fp2 >> transformationMatrix[i][j];
	}

	while (!fp.eof())
	{
		fp>>camPoints[k][0];
		fp>>camPoints[k][1];
		fp>>camPoints[k][2];
		camPoints[k][3] = 1;
		k++;
	}
	
	k--;

	SysRefConversion (camPoints, resPoints, k, transformationMatrix);

	cout << endl << "The number of points is: " << k << endl;

	cout << endl << "The secuence points is:" << endl;

	for (int i = 0; i < k ;i++)
		cout << endl << resPoints[i][0] << " " << resPoints[i][1] << " " << resPoints[i][2] << endl;

	cout << endl << "Correct? y/n: " << endl;
	
	if((c = getchar())=='y')
	{
		//ReadyToPush();
		ReadyToClamp();
		for (int i=0; i<k ; i++)
		{
			MoveTo (resPoints[i][0], resPoints[i][1], resPoints[i][2]);
		}
	}
			
	else
		cout << endl << "Launch again ReproduceMovement" << k << endl;


}

void CKinova::ReproduceOneMovement ()
{
	char c;

	c=getchar();
	while (c=='y')
	{
		LocateOnePattern();
		ReproduceMovement();
		cout << endl << "Again? y/n" << endl;
		c=getchar();
	}

}

void CKinova::SysRefConversion (double inPoints[100][4], double outPoints[100][4], int pointsNum, double transformationMatrix[4][4])
{
	//Calculation of the resulting points 
	for (int i = 0; i < pointsNum ; i++)
		for (int j = 0; j < 4 ; j++)
		{
			outPoints[i][j] = transformationMatrix[j][0]*inPoints[i][0]+ transformationMatrix[j][1]*inPoints[i][1]+transformationMatrix[j][2]*inPoints[i][2]+ transformationMatrix[j][3]*inPoints[i][3];
		}


}

struct TMyPoint
{
	TMyPoint():x(0),y(0),rb(0){}
	int x;
	int y;
	bool rb;
};

void MyMouseCallback (int event, int x, int y, int flags, void* param)
{
	TMyPoint *Point = static_cast<TMyPoint*>(param);
	
	//cout << "The pixel coordinates are: " << x << ", " << y << endl;

	if (event== CV_EVENT_LBUTTONDOWN)
	{
		cout << "Mouse Push in: " << x << ", " << y <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		
		Point->x = x;
		Point->y = y;

	}

	if (event== CV_EVENT_RBUTTONDOWN)
	{
		cout << "RIGHT " << x << ", " << y <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		
		Point->x = x;
		Point->y = y;
		Point->rb = 1;
	}
}

void CKinova::TakeDepth ()
{

	VideoCapture capture;			//Class for video capturing from video files or cameras

	Mat view, view2, view3;			//OpenCV C++ matrix class.

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F), distCoeffs = Mat::zeros(5, 1, CV_64F);

	double camPoints [100][4];
	double resPoints [100][4];
	double transformationMatrix [4][4]; 
	char c;
	bool first = false;

	ReadyToPick();
	Drop();
	FingerTo (100,100,100);
	//MoveTo (0.2f, -0.4f, 0.1f);

	//-------------------------------------
	// Load & show 
	//-------------------------------------

	ifstream fp("CameraMatrixAndDistortion.txt");
	ifstream fp2("TransformationMatrix.txt");

	// Load Camera Matrix
	for (int l=0;l<3;l++)
		for (int m=0;m<3;m++)
			fp >> cameraMatrix.at<double>(l,m);
	
	// Load distorsion coefficients
	for (int l=0;l<5;l++)
		fp >> distCoeffs.at<double>(l,0);

	//Load the transformation matrix
	for (int i = 0; i < 4 ;i++)
	{
		for (int j = 0; j < 4 ;j++)
			fp2 >> transformationMatrix[i][j];
	}

	fp.close();
	fp2.close();
	
	cout << "\n\nLa matriz de la camara es:\n" << cameraMatrix <<  "\n";
	cout << "\nLos coeficientes de distorsión son:\n" << distCoeffs <<  "\n";


	//-------------------------------------
	// Open and Set Camera
	//-------------------------------------

	capture.open(cameraId);		//Open video file or a capturing device for video capturing
								//The method first call VideoCapture::release() to close the already opened file or camera.

	if(!capture.isOpened())		//Returns true if video capturing has been initialized already.
	{}
	else
    {
		namedWindow( "Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.
		namedWindow( "3D Image View", CV_WINDOW_AUTOSIZE ); //Creates a window.

		//Sets a property in the VideoCapture. (Property identifier, Value of the property) 
		capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

		//Display cameras settings 
		cout << "\nDepth generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
				"FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

		cout << "\nImage generator output mode:" << endl <<
				"FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
				"FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
				"FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;		
	}
	
	// This loop is for remove the first captures of the cameras (The lighting setting is bad)
	for (int i = 0; i < 4; i++)
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
		{}
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone();
				undistort(view2, view, cameraMatrix, distCoeffs);

				imshow("Image View", view);	//Displays the image in the specified window.
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}
		cvWaitKey(20);
	}

	TMyPoint *myPoint = new TMyPoint();

	cvSetMouseCallback ("Image View", MyMouseCallback, myPoint);

	//delete myPoint;

	//-------------------------------------
	// Loop
	//-------------------------------------

	

	for (int i = 1;; i++) //// Debe acabar con la calibración
	{
		if(!capture.grab()) //Grabs the next frame from video file or capturing device.
		{}
		else
		{
			if(capture.retrieve(view, CV_CAP_OPENNI_GRAY_IMAGE)) //Decodes and returns the grabbed video frame.
			{
				view2 = view.clone(); 
				undistort(view2, view, cameraMatrix, distCoeffs); ///// Se puede optimizar con otro método

				imshow("Image View", view);	//Displays the image in the specified window.
			}

			if(capture.retrieve(view3, CV_CAP_OPENNI_POINT_CLOUD_MAP)) //Decodes and returns the grabbed video frame.
			{
				imshow("3D Image View", view3);	//Displays the image in the specified window.
			}
		}
		cvWaitKey(20);

		//cout << "The pixel in out code is: " << myPoint->x << " " << myPoint->y << endl;
		cout << "The 3d coordinates are: " <<view3.at<Vec3f>(myPoint->y,myPoint->x)[0] << " ";
		cout <<								 view3.at<Vec3f>(myPoint->y,myPoint->x)[1] << " ";
		cout <<								 view3.at<Vec3f>(myPoint->y,myPoint->x)[2] << endl;


		if(myPoint->rb)
		{
			
			vector<Point2f> pointbuf;
			Size boardSize, imageSize;		//Template class for specfying image or rectangle size
			float aspectRatio = 1.f;
			boardSize.width = boardSizewidth;
			boardSize.height = boardSizeheight;


			imageSize = view.size();
			int found = 0;

			found = findChessboardCorners( view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			//Improve the found corners' coordinate accuracy
			if(found)
			{
				cornerSubPix(view, pointbuf, Size(11,11),Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
				//drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
			
				camPoints[0][0] = view3.at<Vec3f>((int)pointbuf[0].y,(int)pointbuf[0].x)[0];
				camPoints[0][1] = view3.at<Vec3f>((int)pointbuf[0].y,(int)pointbuf[0].x)[1];
				camPoints[0][2] = view3.at<Vec3f>((int)pointbuf[0].y,(int)pointbuf[0].x)[2];
				camPoints[0][3] = 1;

				cout << endl << "The pixel for camPoint is: " <<  (int)pointbuf[0].y << " " << (int)pointbuf[0].x << endl;
				cout << endl << "The camPoint is: " << camPoints[0][0] << " "<< camPoints[0][1] << " " << camPoints[0][2] << endl;

			}

			/////
			camPoints[0][0] = view3.at<Vec3f>(myPoint->y,myPoint->x)[0];
			camPoints[0][1] = view3.at<Vec3f>(myPoint->y,myPoint->x)[1];
			camPoints[0][2] = view3.at<Vec3f>(myPoint->y,myPoint->x)[2];
			camPoints[0][3] = 1;
			SysRefConversion (camPoints, resPoints, 1, transformationMatrix);

			cout << endl << "The pixel for camPoint is: " <<  myPoint->y << " " << myPoint->x << endl;
			cout << "The ARM coordinates are: " << resPoints[0][0] << " " << resPoints[0][1] << " " << resPoints[0][2] << " ";
			myPoint->rb = 0;

			cout << endl << "Correct? y/n: " << endl;
	
			if((c = getchar())=='y')
			{
				

				if (!first)
				{
					//ReadyToPush();
					ReadyToClamp();
					MoveTo (resPoints[0][0], resPoints[0][1]-0.02, resPoints[0][2]+0.1);
					Drop();
					MoveTo (resPoints[0][0], resPoints[0][1], resPoints[0][2]);
					FingerTo (50,50,50);
					MoveTo (resPoints[0][0], resPoints[0][1]-0.02, resPoints[0][2]+0.1);
					GoHome();
					first = true;
				}
				else
				{
					ReadyToClamp();
					MoveTo (resPoints[0][0], resPoints[0][1]-0.02, resPoints[0][2]+0.1);
					Drop();
					first = false;
				}

			}
			
			else
				cout << endl << "Launch again ReproduceMovement" << endl;
		}

	}
}

// Project Points 
double CKinova::computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

	ofstream fp("ProjectPoints.txt");
    
    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);

		fp << objectPoints[i];

        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

	
    fp.close();
    
	return std::sqrt(totalErr/totalPoints);
}

void CKinova::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType /*= CHESSBOARD*/)
{
    corners.resize(0);
    
    switch(patternType)
    {
      case CHESSBOARD:
      case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

      case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

      default:
        CV_Error(CV_StsBadArg, "Unknown pattern type\n");
    }
}

bool CKinova::runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize, Pattern patternType,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    
    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);


    
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);
    
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
    
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

//Similar to OpenCV's method
void CKinova::saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );
    
    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;
    
    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;
    
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }
    
    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);
    
    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }
    
    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }




}


bool CKinova::runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
	

	ofstream fp("ArmCamPoints.txt");
	ofstream fp2("TransformationTuples.txt");

	//-------------------------------------
	// Calibrate Positions
	//-------------------------------------

	 float Calib_Pos [81] [9] = {	{ 0.20f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.4f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.4f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.4f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
	 
									{ 0.20f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.5f, 0.10f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.5f, 0.20f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.5f, 0.25f,-1.57f, 0.0f,-1.0f, 100, 100, 0},

									{ 0.20f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.05f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.00f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.45f, 0.15f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.20f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.15f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.10f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.05f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{ 0.00f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.05f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.10f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.15f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0},
									{-0.20f,-0.45f, 0.22f,-1.57f, 0.0f,-1.0f, 100, 100, 0}};


    
    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
		printf("%s. avg reprojection error = %.2f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);
    
    if( ok )
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
	
	for (int j = 0; j < framesEX ;j++)
	{
		fp << Calib_Pos[j][0] << ' ';
		fp << Calib_Pos[j][1] << ' ';
		fp << Calib_Pos[j][2] << ' ';
		fp << tvecs[j].at<double>(0,0) << ' ';
		fp << tvecs[j].at<double>(0,1) << ' ';
		fp << tvecs[j].at<double>(0,2) << endl;
	}

	for (int j = 0; j < framesEX ;j++)
	{
		fp2 << tvecs[j].at<double>(0,0) << ' ';
		fp2 << tvecs[j].at<double>(0,1) << ' ';
		fp2 << tvecs[j].at<double>(0,2) << ' ';
		fp2 << rvecs[j].at<double>(0,0) << ' ';
		fp2 << rvecs[j].at<double>(0,1) << ' ';
		fp2 << rvecs[j].at<double>(0,2) << endl;
	}
	
	fp.close();
	fp2.close();

    return ok;
}

//------------------------------------
// () CONFIGURATION MANAGERS
//------------------------------------

//////// GENERAL CONFIGURATION ////////

CVectorAngle^ CKinova::GetJointPositions() //This functionality provides the values of all Jaco’s actuators angles.
{

	m_semaphore.waitForSignal();
	jointPosition = jaco->ConfigurationsManager->GetJointPositions();
	m_semaphore.release();

	return jointPosition;

}

CVectorEuler^ CKinova::GetHandPosition () //This functionality provides the Cartesian position of the robot’s hand.
{

	m_semaphore.waitForSignal();

	handPosition = jaco->ConfigurationsManager->GetHandPosition(); 

	XPosition = handPosition->Position[CVectorEuler::COORDINATE_X]; 
	YPosition = handPosition->Position[CVectorEuler::COORDINATE_Y]; 
	ZPosition = handPosition->Position[CVectorEuler::COORDINATE_Z];

	ThetaX = handPosition->Rotation[CVectorEuler::THETA_X]; 
	ThetaY = handPosition->Rotation[CVectorEuler::THETA_Y]; 
	ThetaZ = handPosition->Rotation[CVectorEuler::THETA_Z]; 

	m_semaphore.release();

	return handPosition;

}

/*
String^ CKinova::GetCodeVersion() //This  functionality  provides  the  code  version  of  all  firmware  that  are  inside  Jaco.
{

	array<int>^ CodeVersion; 
	String^ DSPVersion = ""; 
             
	CodeVersion = jaco->ConfigurationsManager->GetCodeVersion(); 
                       
	//We convert the hexadecimal number into a decimal number. 
	DSPVersion = CodeVersion[CJacoArm::VERSION_DSP].ToString("x1"); 
                      
	//Output of this is: DSP Code version: 040105 (if version 4.01.05) 
	System::Console::WriteLine("DSP Code version : " + DSPVersion);

	return DSPVersion;

}
*/

bool CKinova::JacoIsReady () //This functionality indicates if Jaco is ready to communicate.
{

	bool Result;

	Result = jaco->JacoIsReady();

	return Result; //When the method returns FALSE, check if your cable is unplugged, the power is off or Jacosoft it's run.

}

//////// CLIENT CONFIGURATION ////////

void CKinova::SetClientConfigurations () //This functionality send to Jaco the client’s configuration specified in the input parameters.
{

	jaco->ConfigurationsManager->SetClientConfigurations(config); 

}

CClientConfigurations^ CKinova::GetClientConfigurations () //This functionality gets the actual client’s configuration from Jaco.
{
	
	config = jaco->ConfigurationsManager->GetClientConfigurations();

	return config;

}

void CKinova::SerializeClientConfiguration () //This functionality provides a way to serialize on disk a CClientConfigurations object in a binary file.
{

	jaco->ConfigurationsManager->SerializeClientConfiguration("MyClientProfile"); 
	//Your  client  profile  has  been  saved  in  a  binary  file  at  location  :       
	//PathCatalog.BASE_API_PROFILES_PATH.

}

void CKinova::DeserializeClientConfiguration () //This functionality provides a way to deserialize a CClientConfigurations object from a binary file on disk.
{

	jaco->ConfigurationsManager->DeserializeClientConfiguration("MyClientProfile"); 
	//Your client profile has been load from a binary file at the location :    
	//PathCatalog.BASE_API_CLIENTCONFIG_PATH.

}

//////// PROTECTION ZONES ////////

void CKinova::SetProtectionZones () //This  functionality sends and  set  the  protection  zones  to  Jaco.
{

	jaco->ConfigurationsManager->SetProtectionZones(zoneList);  
	
}

CZoneList^ CKinova::GetProtectionZones () //This functionality  gets  the  protection  zones from Jaco.
{

	zoneList = jaco->ConfigurationsManager->GetProtectionZones(); 

	return zoneList;

}

void CKinova::DeleteProtectionsZones () //This functionality erases all active protection zones in Jaco. 
{

	jaco->ConfigurationsManager->DeleteAllProtectionsZones();  
	
}

void CKinova::SerializeZoneConfiguration () //This functionality  provides  a  way  to  serialize  on  disk  a  CZoneList  object  in  a  binary  file.
{

	jaco->ConfigurationsManager->SerializeZoneConfiguration("MyZoneProfile"); 
	//Your  zone  profile  has  been  saved  in  a  binary  file  at  location  : 
	//PathCatalog.BASE_API_PROFILES_PATH.

}

void CKinova::DeserializeZoneConfiguration () //This functionality provides a way to deserialize a CZoneList object from a binary file on disk.
{

	jaco->ConfigurationsManager->DeserializeZoneConfiguration("MyZoneProfile"); 
	//Your  zone  profile  has  been  load  from  a  binary  file  at  location  : 
	//PathCatalog.BASE_API_PROFILES_PATH.

}

//////// CONTROL MAPPING ////////

void CKinova::SetControlMappingCharts () //This  functionality  sends  and  set  the  control  mapping  of  Jaco.
{

	jaco->ConfigurationsManager->SetControlMappingCharts(mappingCharts);

}

CControlMappingCharts^ CKinova::GetControlMappingCharts () //This  functionality  gets  the  protection  zones  from  Jaco.
{

	mappingCharts = jaco->ConfigurationsManager->GetControlMappingCharts();

	return mappingCharts;

}

void CKinova::SerializeControlMappingCharts () //This functionality provides a way to serialize on disk a CControlMappingCharts object in a binary file.
{

	jaco->ConfigurationsManager->SerializeControlMappingCharts();

}

void CKinova::DeserializeControlMappingCharts () //This functionality  provides  a  way  to deserialize  a  CControlMappingCharts  object from  a  binary file on disk.
{

	jaco->ConfigurationsManager->DeserializeControlMappingCharts();

}

//////// PROFILE ////////

void CKinova::LoadProfileBackup () //This  functionality provides  a  way  to  load  configuration  profile  on  disk.
{

	jaco->ConfigurationsManager->LoadProfileBackup("MyJACOProfile"); 

}

void CKinova::CreateProfileBackup () //This functionality provides a way to save a configuration profile on disk.
{

	jaco->ConfigurationsManager->CreateProfileBackup("MyJACOProfile"); 

}

//------------------------------------
// () CONFIGURATION MANAGERS
//------------------------------------

//////// DIAGNOSTIC DATA ////////

//////// CPOSITION ////////

long long unsigned int CKinova::GetErrorLogCount() //Every  time  that  a major error  occurs  in  Jaco,  it  is  logged  inside  a  memory  on  the  main  board.
{

	long long unsigned int Count = jaco->DiagnosticManager->DataManager->GetErrorLogCount();

	System::Console::WriteLine("Error count : " + Count);

	return Count;

}

int CKinova::GetPositionLogCount() //When  the  robot  is  powered,  there  is  process  that  store  information  over time.
{

	int Count = jaco->DiagnosticManager->DataManager->GetPositionLogCount();

	System::Console::WriteLine("CPosition count : " + Count);

	return Count;

}

List<CPosition^>^ CKinova::GetPositionFromJaco(int index) //This  functionality  returns  a  specific  position  from  Jaco.
{

	List<CPosition^>^ list = gcnew List<CPosition^>(); 
 
	list->Add(jaco->DiagnosticManager->DataManager->GetPositionFromJaco(index)); 

	return list;

}

CPeripheralInformation^ CKinova::GetPeripheralInformationFromJaco() //This functionality returns information about the peripheral that is connected to Jaco.
{

	CPeripheralInformation^ info = jaco->DiagnosticManager->DataManager->GetPeripheralInformationFromJaco();

	System::Console::WriteLine("Device ID : " + info->DeviceID);

	return info;

}

CPosition^ CKinova::GetPositionLogLiveFromJaco() //This functionality returns the actual position of the robot along with all information relative to it
{

	CPosition^ Position = jaco->DiagnosticManager->DataManager->GetPositionLogLiveFromJaco(); 

	return Position;

}

void CKinova::DeleteErrorLog() //This functionality deletes all major errors listed inside Jaco.
{

	jaco->DiagnosticManager->DataManager->DeleteErrorLog();

}

//////// TOOLS ////////

void CKinova::RestoreFactorySettings () //This functionality restores Jaco to its factory default settings.
{

	jaco->DiagnosticManager->ToolManager->RestoreFactorySettings();

}

//------------------------------------
// () CONTROL MANAGER
//------------------------------------

//////// CONTROL ////////

void CKinova::StartControlAPI() //This functionality tells Jaco that from now on, it is the API that will control it.
{
	//From now on, API can move the arm.
	jaco->ControlManager->StartControlAPI();

	//The API will lose control if the StopControlAPI is called, if the USB connection 
	//broke or if another interface with a higher priority move the arm. 
	//For example, the 3 axis joystick directly connected to JACO have an higher priority. 

}

void CKinova::StopControlAPI() //This functionality tells Jaco that from now on, the API is no longer controlling Jaco.
{

	jaco->ControlManager->StopControlAPI();

}

CInfoFIFOTrajectory^ CKinova::GetInfoFIFOTrajectory () //This functionality returns information about the trajectory FIFO inside Jaco.
{

	//Declaration of the data structure that will hold information about Jaco's trajectories FIFO. 
	CInfoFIFOTrajectory^ info; 
 
	//We get the information from the robotic arm Jaco. 
	info = jaco->ControlManager->GetInfoFIFOTrajectory(); 
 
	//System::Console::WriteLine("Quantity of trajectories still waiting to be executed : " + info->StillInFIFO); 
	//System::Console::WriteLine("FIFO max size : " + info->MaxSize); 

	return info;

}

void CKinova::SendTrajectoryFunctionnality() //This functionality adds a trajectory to Jaco’s trajectory FIFO.
{
	cout << endl << "Entra a la función sendt" << endl;
	m_semaphore.waitForSignal();

	cout << endl << "Semaforo capturado" << endl;
	Thread::Sleep(10);
		
	jaco->ControlManager->SendTrajectoryFunctionnality(pointsTrajectory);

	Thread::Sleep(10);

	m_semaphore.release();
	cout << endl << "Semaforo liberado" << endl;

}

void CKinova::SendJoystickFunctionality() //This functionality sends a virtual joystick command which will executed by Jaco according to the active mapping in effect.
{
	m_semaphore.waitForSignal();

	Thread::Sleep(100);

	jaco->ControlManager->SendJoystickFunctionality(joystick);

	Thread::Sleep(100);

	m_semaphore.release();
	
}

bool CKinova::IsApiInControl() //This functionality  ask Jaco  if  it  is  still  the  API  that  is  controlling  Jaco.
{

	bool result;

	result = jaco->ControlManager->IsApiInControl();

	return result;

}

void CKinova::SetCartesianControl() //This functionality switches the arm to cartesian control.
{	

	jaco->ControlManager->SetCartesianControl(); 

	System::Console::WriteLine("JACO is now in cartesian mode");

}

void CKinova::SetAngularControl() //This functionality switches the arm to angular control.
{	
	jaco->ControlManager->SetAngularControl(); 

	System::Console::WriteLine("JACO is now in angular mode");
}

CTrajectoryInfo^ CKinova::GetActualTrajectoryInfo() //This functionality returns information about the trajectory that Jaco is currently executing. 
{

	CTrajectoryInfo^ info = jaco->ControlManager->GetActualTrajectoryInfo();

	return info;

}

void CKinova::EraseTrajectories() //This functionality erase all trajectory from Jaco’s trajectory FIFO. 
{

	jaco->ControlManager->EraseTrajectories();

	//We wait a bit... 
	Thread::Sleep(4000);

}

CCartesianInfo^ CKinova::GetPositioningCartesianInfo() //This functionality returns information specific to the cartesian position. 
{

	CCartesianInfo^ info;
	info = jaco->ControlManager->GetPositioningCartesianInfo();

	return info;

}

CAngularInfo^ CKinova::GetPositioningAngularInfo() //This functionality returns information specific to the angular position. 
{

	CAngularInfo^ info;
	info = jaco->ControlManager->GetPositioningAngularInfo();

	return info;

}

CCartesianInfo^ CKinova::GetCommandCartesianInfo() //This functionality returns information specific to the cartesian command. 
{

	CCartesianInfo^ info;
	info = jaco->ControlManager->GetCommandCartesianInfo();

	return info;

}

CAngularInfo^ CKinova::GetCommandAngularInfo() //This functionality returns information specific to the angular command.
{

	CAngularInfo^ info;
	info = jaco->ControlManager->GetCommandAngularInfo();

	return info;

}

CCartesianInfo^ CKinova::GetForceCartesianInfo() //This functionality returns information specific to the cartesian force. 
{

	CCartesianInfo^ info;
	info = jaco->ControlManager->GetForceCartesianInfo();

	return info;

}

CAngularInfo^ CKinova::GetForceAngularInfo() //This functionality returns information specific to the angular force.
{

	CAngularInfo^ info;
	info = jaco->ControlManager->GetForceAngularInfo();

	return info;

}

CAngularInfo^ CKinova::GetCurrentAngularInfo() //This functionality returns information specific to the angular current. 
{

	CAngularInfo^ info;
	info = jaco->ControlManager->GetCurrentAngularInfo();

	return info;

}

//------------------------------------
// () OTHER FUNCTIONALITIES
//------------------------------------

void CKinova::GetAPIVersion() //This functionality returns the version of the API. 
{

	System::Console::WriteLine("API Version = " + jaco->GetAPIVersion());

}
