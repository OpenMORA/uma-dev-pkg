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

#ifndef CKinova_H
#define CKinova_H

#include "CCalibration.h"
#include <CMapirMOOSApp.h>
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace System::Collections::Generic;
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

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

ref class CKinova 
{

private:

	// An Object that represents the robotic arm Jaco.
	CJacoArm^ jaco;
	
	// Others Kinova objects
	CPointsTrajectory^ pointsTrajectory;
	CTrajectoryInfo^ trajectory;
	CVectorEuler^ cVector;
	CZoneList^ zoneList;
	CTrajectoryInfo^ trajectoryInfo;
	CCartesianInfo^ cartesianInfo;
	CVectorEuler^ handPosition;
	CVectorAngle^ jointPosition;
	CClientConfigurations^ config;
	CControlMappingCharts^ mappingCharts;

protected:

	// Other Kinova object
	CCalibration* calibration;

public:

	// Constructor & Destructor
	CKinova::CKinova();
	virtual CKinova::~CKinova();

	// Other Kinova object
	CJoystickValue^ joystick;
	
	//------------------------------------
	// () NEW FUNCTIONS
	//------------------------------------

	/////// BASIC FUNCTIONS ////////////////////////////////

	void Inicialize (); // Sequence of actions to initialize the arm. Is required to run at the beginning.
	void OffSecuence (); // Sequence of actions to turn off the arm
	void MoveTo (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z); // Moves in absolute cartesian coordinates 
	void PositionDisplace (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z); // Displacement in cartesian coordinates
	void Pick (int Diameter); // Pick. Only moves the fingers
	void Drop (); // Drop. Only moves the fingers
	void DisplayHandPosition (); //This functionality display the Cartesian position of the robot’s hand.
	
	/////// CONFIGURATION FUNCTIONS ////////////////////////

	void ChangeSpeed (float ang_Speed, float lin_Speed); // Change speed configured
	void MyClientConfigurations (); // Change all client configurations (Don't use if you're not sure that all parameters are corrects)

	/////// ACTION FUNCTIONS ///////////////////////////////

	void PickObject (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, int Diameter); // Sequence of actions to pick a object (ReadytoPick start position is required)
	void DropObject (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z); // Sequence of actions to drop a object
	void SmellObject (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, int SmellTime); // Sequence of actions to smell a object (ReadytoClamp start position is required)
	void GiveMeYourHand (); // Jaco does the action to give the hand
	void TurnHandLeft (float rad); // Roll left
	void TurnHandRight (float rad); // Roll right

	/////// STARTING POSITIONS /////////////////////////////

	void GoHome (); // Arm goes to home position (It's necessary run at least once to move the arm)
	void Retract (); // Arm goes to retract position
	void ReadyToPick (); // Arm configuration to pick an object
	void ReadyToOdometry (); // Arm configuration to pick an object
	void ReadyToPush (); // Arm configuration to push something
	void ReadyToClamp (); // Arm configuration to clamp
	void ReadyToCalibrate (); // Arm configuration to calibrate arm with camera

	/////// OTHER FUNCTIONS ////////////////////////////////

	bool TargetReached (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, float THETA_X, float THETA_Y, float THETA_Z); // Return true if absolute position is reached
	bool TargetReachedCartesian (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z); // Return true if absolute cartesian position is reached
	
	void Block (); // Block the arm
	void DisplayJointPositions (); //This functionality provides the values of all Jaco’s actuators angles.

	/////// KINOVA FUNCTIONS ////////////////////////////////

	void AddCompleteTrajectory (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z, float THETA_X, float THETA_Y, float THETA_Z, int FingerPosition_0, int FingerPosition_1, int FingerPosition_2); // Moves all in cartesian mode (with Kinova Functions)
	void PositionDisplaceK (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z); // Displace in cartesian coordinates (with Kinova Functions)
	void MoveToK (float COORDINATE_X, float COORDINATE_Y, float COORDINATE_Z); // Moves in absolute cartesian coordinates (with Kinova Functions)
	void MoveToAngle (int JOINT_1, int JOINT_2, int JOINT_3, int JOINT_4, int JOINT_5, int JOINT_6, int FingerPosition_0, int FingerPosition_1, int FingerPosition_2); //  Moves all in angular coordinates (with Kinova Functions)
	void RotationTo (float THETA_X, float THETA_Y, float THETA_Z); // Moves euler angles 
	void FingerTo (int FingerPosition_0, int FingerPosition_1, int FingerPosition_2); // Moves fingers
	void RotationDisplace (float THETA_X, float THETA_Y, float THETA_Z); // Displacement in angular coordinates
	void FingerDisplace (int FingerPosition_0, int FingerPosition_1, int FingerPosition_2); // Displacement
	bool WaitTrajectory (); // Waits for a movement finished
	
	/////// DEMOSTRATIONS ///////////////////////////////////

	void Demostration1 (); // Example to switch betweeen Cartesian and Angular modes
	void Demostration2 (); // Example of pick and drop object
	void Demostration3 (); // Example to push a button
	void Demostration4 (); // Example to displace
	void Demostration5 (); // Example
	bool TestFingers (); // Example to see finger positions
	void DemostrationVideo ();
	
	/////// ARM CALIBRATION /////////////////////////////////
	
	int CalibrateArm ();
	int CalibrateArmDepth ();
	int IntrinsicCalibration ();
	void TransformationPoint ();
	int LocatePattern ();
	int LocateOnePattern ();
	void ReproduceMovement ();
	void ReproduceOneMovement ();
	void SysRefConversion (double inPoints[100][4], double outPoints[100][4], int pointsNum, double transformationMatrix[4][4]);

	//void MouseCallback (int event, int x, int y, int flags, void* param);
	void TakeDepth ();

	//------------------------------------
	// () OPENCV FUNCTIONS MODIFIED
	//------------------------------------

	double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors);
	void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType);
	bool runCalibration( vector<vector<Point2f> > imagePoints,
		Size imageSize, Size boardSize, Pattern patternType,
		float squareSize, float aspectRatio,
		int flags, Mat& cameraMatrix, Mat& distCoeffs,
		vector<Mat>& rvecs, vector<Mat>& tvecs,
		vector<float>& reprojErrs,
		 double& totalAvgErr);
	void saveCameraParams(const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr );
	bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints);

	//------------------------------------
	// () CONFIGURATION MANAGERS
	//------------------------------------

	//////// GENERAL CONFIGURATION ////////

	CVectorAngle^ GetJointPositions (); //OK //This functionality provides the values of all Jaco’s actuators angles.
	CVectorEuler^ GetHandPosition (); //OK //This functionality display the Cartesian position of the robot’s hand.
	//String^ GetCodeVersion (); //OK //This  functionality  provides  the  code  version  of  all  firmware  that  are  inside  Jaco.
	bool JacoIsReady (); //OK //This functionality indicates if Jaco is ready to communicate.

	//////// CLIENT CONFIGURATION ////////

	void SetClientConfigurations (); //This functionality send to Jaco the client’s configuration specified in the input parameters.
	CClientConfigurations^ GetClientConfigurations (); //This functionality gets the actual client’s configuration from Jaco.
	void SerializeClientConfiguration (); //OK //This functionality provides a way to serialize on disk a CClientConfigurations object in a binary file.
	void DeserializeClientConfiguration (); //OK //This functionality provides a way to deserialize a CClientConfigurations object from a binary file on disk.

	//////// PROTECTION ZONES ////////

	void SetProtectionZones (); //This  functionality sends and  set  the  protection  zones  to  Jaco.
	CZoneList^ GetProtectionZones (); //This functionality  gets  the  protection  zones from Jaco.
	void DeleteProtectionsZones (); //This functionality erases all active protection zones in Jaco. 
	void SerializeZoneConfiguration (); //This functionality  provides  a  way  to  serialize  on  disk  a  CZoneList  object  in  a  binary  file.
	void DeserializeZoneConfiguration (); //This functionality provides a way to deserialize a CZoneList object from a binary file on disk.

	//////// CONTROL MAPPING ////////

	void SetControlMappingCharts (); //This  functionality  sends  and  set  the  control  mapping  of  Jaco.
	CControlMappingCharts^ GetControlMappingCharts (); //This  functionality  gets  the  protection  zones  from  Jaco.
	void SerializeControlMappingCharts (); //This functionality provides a way to serialize on disk a CControlMappingCharts object in a binary file.
	void DeserializeControlMappingCharts (); //This functionality  provides  a  way  to deserialize  a  CControlMappingCharts  object from  a  binary file on disk.

	//////// PROFILE ////////

	void LoadProfileBackup (); //This  functionality provides  a  way  to  load  configuration  profile  on  disk.
	void CreateProfileBackup (); //This functionality provides a way to save a configuration profile on disk.

	//------------------------------------
	// () CONFIGURATION MANAGERS
	//------------------------------------

	//////// DIAGNOSTIC DATA ////////

	//////// CPOSITION ////////

	long long unsigned int GetErrorLogCount (); //OK //Every  time  that  a major error  occurs  in  Jaco,  it  is  logged  inside  a  memory  on  the  main  board.
	int GetPositionLogCount (); //OK //When  the  robot  is  powered,  there  is  process  that  store  information  over time.
	List<CPosition^>^ GetPositionFromJaco (int index); //This  functionality  returns  a  specific  position  from  Jaco.
	CPeripheralInformation^ GetPeripheralInformationFromJaco (); //OK //This functionality returns information about the peripheral that is connected to Jaco.
	CPosition^ GetPositionLogLiveFromJaco (); //This functionality returns the actual position of the robot along with all information relative to it
	void DeleteErrorLog (); //This functionality deletes all major errors listed inside Jaco.

	//////// TOOLS ////////

	void RestoreFactorySettings (); //This functionality restores Jaco to its factory default settings

	//------------------------------------
	// () CONTROL MANAGER
	//------------------------------------

	//////// CONTROL ////////

	void StartControlAPI(); //OK //This functionality tells Jaco that from now on, it is the API that will control it.
	void StopControlAPI(); //OK //This functionality tells Jaco that from now on, the API is no longer controlling Jaco.
	CInfoFIFOTrajectory^ GetInfoFIFOTrajectory (); //OK //This functionality returns information about the trajectory FIFO inside Jaco.
	void SendTrajectoryFunctionnality(); //OK //This functionality adds a trajectory to Jaco’s trajectory FIFO.
	void SendJoystickFunctionality(); //OK //This functionality sends a virtual joystick command which will executed by Jaco according to the active mapping in effect.
	bool IsApiInControl(); //OK //This functionality  ask Jaco  if  it  is  still  the  API  that  is  controlling  Jaco.
	void SetCartesianControl(); //OK //This functionality switches the arm to cartesian control.
	void SetAngularControl(); //OK //This functionality switches the arm to angular control.
	CTrajectoryInfo^ GetActualTrajectoryInfo(); //This functionality returns information about the trajectory that Jaco is currently executing. 
	void EraseTrajectories(); //This functionality erase all trajectory from Jaco’s trajectory FIFO. 
	CCartesianInfo^ GetPositioningCartesianInfo(); //This functionality returns information specific to the cartesian position. 
	CAngularInfo^ GetPositioningAngularInfo(); //This functionality returns information specific to the angular position. 
	CCartesianInfo^ GetCommandCartesianInfo(); //This functionality returns information specific to the cartesian command.
	CAngularInfo^ GetCommandAngularInfo(); //This functionality returns information specific to the angular command.
	CCartesianInfo^ GetForceCartesianInfo(); //This functionality returns information specific to the cartesian force. 
	CAngularInfo^ GetForceAngularInfo(); //This functionality returns information specific to the angular force.
	CAngularInfo^ GetCurrentAngularInfo(); //This functionality returns information specific to the angular current. 

	//------------------------------------
	// () OTHER FUNCTIONALITIES
	//------------------------------------

	void GetAPIVersion(); //OK //This functionality returns the version of the API. 

	//------------------------------------
	// () DATA
	//------------------------------------

	bool debugMode;
	int cameraId; //Primesense ID
	int framesIC;
	int framesEX;
	float squareSize;
	int boardSizewidth;
	int boardSizeheight;

	float XPosition;	// Actual x position
	float YPosition;	// Actual y position
	float ZPosition;	// Actual z position
	float ThetaX;		// Actual ThetaX position
	float ThetaY;		// Actual ThetaY position
	float ThetaZ;		// Actual ThetaZ position

	float xTarget;		// Target's x position
	float yTarget;		// Target's y position
	float zTarget;		// Target's z position
	float tXTarget;		// Target's ThetaX position
	float tYTarget;		// Target's ThetaY position
	float tZTarget;		// Target's ThetaZ position
	
	bool activeJoystick; // Active the joystick mode

};

#endif