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


/**  @moos_module This module implements the adquisition of depth and infrared images from an RGB-D camera (PrimeSense/Asus/Kinect)
  *  and processes these data to send them to the 3D reactive module. The module sorts the points in height bands (according to the height sections
  *  used to model the robot shape) and finds the most restrictive set of obstacles for each height band. This module should always be
  *  launched together with the "NavigatorReactivePTG3D" because some of its configuration parameters need to be read.
  */

#include "CRGBD_App.h"
#include <sstream>
#include <iostream>
#include <PS1080.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


bool CRGBD_App::OnStartUp()
{
	try
	{
		bool open_ok;
		
		// Read configuration params from ini file
		ReadConfiguration();

		if (m_new_sensor)
			open_ok = OpenPSdevice();
		else
			open_ok = OpenKinect();

		if (!open_ok)
		{
			cout << endl << "[RGBD]: ERROR The device can't be openned";
			system::sleep(500);
			return MOOSFail( "Closing the module." );
		}

		// Show debug window with the detected points?
		if (m_visualization)
			InitializeScene();
		
		DoRegistrations();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CRGBD_App::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("This module only accepts string command messages\n");

    std::string sCmd = Msg.GetString();
//  MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}


bool CRGBD_App::Iterate()
{
	try
	{	
		// Get obstacles from images
		if (m_new_sensor)
			GetClosestPointsPS();		//For "newer" range cameras (Primesense or Asus) using OpenNI2
		else
			GetClosestPointsKinect();	//For the old Kinect sensor (Microsoft) using OpenCV


		// Publish detected poitns as OpenMORA variable
		string sKinect1 = ObjectToString(&kinect_points);
		m_Comms.Notify("KINECT1", sKinect1 );


		if (m_visualization)
			ShowPoints();

		cout << endl << "[RGBD]: Iteration runtime (approximate): " << GetTimeSinceIterate() << " seconds";
		return true;

	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CRGBD_App::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CRGBD_App::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
    RegisterMOOSVariables();

    return true;
}


bool CRGBD_App::OnNewMail(MOOSMSG_LIST &NewMail)
{
    UpdateMOOSVariables(NewMail);

    for (MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
    {
    	try
    	{
			if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
			{
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


//----------------------------------------------------------------
// ReadConfiguration: Read parameters of the RGBD camera to use
//----------------------------------------------------------------
void CRGBD_App::ReadConfiguration()
{
	// PROCESS PARAMS
	//-----------------
	m_ini.enableSectionNames();

	//Read number of height levels from the "NavigatorReactivePTG3D" module
	levels.resize(m_ini.read_int("NavigatorReactivePTG3D","HEIGHT_LEVELS",1,true));

	//Read the 3D height sections that models the robot from the "NavigatorReactivePTG3D" module
	for (unsigned int i=0;i<levels.size();i++)
	{
		levels[i] = m_ini.read_float("NavigatorReactivePTG3D",format("LEVEL%d_HEIGHT",i+1), 1, true);
		if (i > 0)
			levels[i] += levels[i-1];
	}

	//! @moos_param  floor_limit  Points detected with height < floor_limit will be considered as floor and discarded as obstacles
	floor_limit = m_ini.read_float("","floor_limit", 0.05, true);

	//! @moos_param  Opengl_scene	Whether or not to display an opengl scene with the detected obstacles (used for debug)
	m_visualization = m_ini.read_bool("","Opengl_scene", 0, false);
			
	//! @moos_param  discard_high_ir_points  If enable, all points with IR value > IR_threshold will be discarded in the depth measurements (to avoid spurious due to light sources)
	discard_high_ir_points = m_ini.read_bool("","discard_high_ir_points", false, false);

	//! @moos_param  IR_threshold  Infrared threshold to consider a point affected by an IR light source
	ir_threshold = m_ini.read_int("","IR_threshold", 750, false);

	//! @moos_param  detect_close_obstacles_with_ir  If enable, points with IR value > IR_threshold will increase the counter of high_ir_px (see Count_threshold param) 
	detect_close_obstacles_with_ir = m_ini.read_bool("","detect_close_obstacles_with_ir", false, false);

	//! @moos_param  Count_threshold  [if detect_close_obstacles_with_ir=true] Number of pixels with IR value > IR_threshold to send the Reactive Module a warning (something may be close)
	count_threshold = m_ini.read_int("","Count_threshold", 50, false);

	// CAMERA PARAMS
	//--------------
	//! @moos_param  New_sensor  Indicates the type of RGBD camera: TRUE=PrimeSense or Asus, FALSE=Kinect
	m_new_sensor = m_ini.read_bool("","New_sensor", 0, false);
	if (m_new_sensor)
		lens_disp = 0.05f;
	else
		lens_disp = -0.015f; //Different sign because of mirroring of the image

	//! @moos_param  Res_width  Camera resolution(px) - Shared for depth and infrared images (320 or 640). Only used if New_sensor == true
	res_width = m_ini.read_float("","Res_width", 320.0, false);

	//! @moos_param  Res_height  Camera resolution(px) - Shared for depth and infrared images (240 or 480). Only used if New_sensor == true
	res_height = m_ini.read_float("","Res_height", 240.0, false);

	//! @moos_param  Downsample  Use it to consider coarser images (computationally lighter). It must be integer
	downsample = m_ini.read_int("","Downsample", 1, false);

	//! @moos_param  min_depth  The minimum depth (m) the camera is able to detect
	min_depth = m_ini.read_float("","min_depth",0.3,true);
	
	//! @moos_param  max_depth  The max depth (m) the camera is able to detect
	max_depth = m_ini.read_float("","max_depth",5,true);

	// CAMERA POSE
	//------------
	//! @moos_param  pose_x  The X position (m) of the camera on the robot
	kinect_pose.x(m_ini.read_float("","pose_x", 0, true));

	//! @moos_param  pose_y  The Y position (m) of the camera on the robot
	kinect_pose.y(m_ini.read_float("","pose_y", 0, true));
	
	//! @moos_param pose_z  The Z position (m) of the camera on the robot
	kinect_pose.z(m_ini.read_float("","pose_z", 0, true));
	
	//! @moos_param  pose_yaw  The yaw angle (degrees) of the camera on the robot (-180,180)
	//! @moos_param  pose_pitch  The pitch angle (degrees) of the camera on the robot (-80,80) (something logical...)
	//! @moos_param  pose_roll  The roll angle (degrees) of the camera on the robot (-90,90)
	kinect_pose.setYawPitchRoll(DEG2RAD(m_ini.read_float("","pose_yaw", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_pitch", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_roll", 0, false)));
}


//--------------------------------------------------------------
// Open a PrimeSense/Asus device using OpenNI2, and configure it
//--------------------------------------------------------------
bool CRGBD_App::OpenPSdevice()
{
	//==============================================================================================
	//									Initialize range Camera
	//==============================================================================================
	const char* deviceURI = openni::ANY_DEVICE;
	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK) { printf("[RGBD]: After initialization:\n %s\n", openni::OpenNI::getExtendedError()); }
	rc = device.open(deviceURI);

	if (rc != openni::STATUS_OK)
	{
		printf("[RGBD]: PS device failed to open:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 0;
	}

	//								Create Depth and IR channels
	//========================================================================================
	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK) { printf("[RGBD]: PS sensor couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError()); }

	rc = ir.create(device, openni::SENSOR_IR);
	if (rc != openni::STATUS_OK) { printf("[RGBD]: PS sensor couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError()); }


	//								Configure video properties
	//========================================================================================
	
	// DEPTH
	options = depth.getVideoMode();
	options.setResolution(res_width,res_height);
	rc = depth.setVideoMode(options);
	rc = depth.setMirroringEnabled(false);
	options = depth.getVideoMode();
	printf("[RGBD] Depth resolution set to: (%d, %d) \n", options.getResolutionX(), options.getResolutionY());
	const int depth_resx = options.getResolutionX();
	const int depth_resy = options.getResolutionY();
	//depth.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	
	// IR
	options = ir.getVideoMode();
	options.setResolution(res_width,res_height);
	rc = ir.setVideoMode(options);
	rc = ir.setMirroringEnabled(false);
	options = depth.getVideoMode();
	printf("[RGBD] Infrared resolution set to: (%d, %d) \n", options.getResolutionX(), options.getResolutionY());

	// Check channels consistency (Depth vs IR)
	if( (depth_resx != options.getResolutionX()) || (depth_resy != options.getResolutionY()) )
	{
		cout << "[RGBD]: ERROR - IR and Depth frames don't have the same size." << endl;
		openni::OpenNI::shutdown();
		return 0;
	}

	//									Start channels
	//========================================================================================	
	rc = depth.start();
	if (rc != openni::STATUS_OK)
	{
		printf("[RGBD]: PS sensor couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		depth.destroy();
	}	
	
	rc = ir.start();
	if (rc != openni::STATUS_OK)
	{
		printf("[RGBD]: PS sensor couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
		ir.destroy();
	}
	
	// CHECK BOTH CHANNELS
	if (!depth.isValid() || !ir.isValid())
	{
		printf("[RGBD]: ERROR - No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 0;
	}
	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 0;
	}

	//						Uncomment this to see the video modes available
	//========================================================================================
	//Infrared modes
	//openni::VideoMode vm;
	//for(unsigned int i = 0; i<infrared.getSensorInfo().getSupportedVideoModes().getSize(); i++)
	//{
	//	vm = infrared.getSensorInfo().getSupportedVideoModes()[i];
	//	printf("\n Depth mode %d: %d x %d, fps - %d Hz, pixel format - ",i, vm.getResolutionX(), vm.getResolutionY(), vm.getFps());
	//	cout << vm.getPixelFormat();
	//}	

	return 1;
}


//-----------------------------------------
//				Open Kinect
//-----------------------------------------
bool CRGBD_App::OpenKinect()
{
	printf("\nKinect resolution: (640, 480).");
	if (downsample == 1)
		printf("\nRecommendation: Downsample the depth images to increase speed \n");
	printf("\nKinect opening ...");
	capture = new cv::VideoCapture(CV_CAP_OPENNI);
	cout << "done." << endl;
	if( !capture->isOpened() )
	{
		cout << "Can not open a capture object." << endl;
		return 0;
	}

	//Set the Kinect grabbing properties
	capture->set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ ); // default

	return 1;
}


void CRGBD_App::GetClosestPointsKinect()
{
	//					Read new frame
	//=======================================================
	if( !capture->grab() )
    {
		std::cout << "Cannot grab images." << std::endl;
        return;
    }

	cv::Mat depthImage;
    capture->retrieve( depthImage, CV_CAP_OPENNI_DEPTH_MAP );
    depthImage.convertTo( depthImage, CV_32FC1, 1./1000 );

	//					Process data
	//=======================================================
	const float inv_f = 1.f/525.f;
	const float ox = 319.5f;
	const float oy = 239.5f;

	unsigned int height = depthImage.rows;
	unsigned int width = depthImage.cols;

	std::vector <float> x_sel;
	std::vector <float> y_sel;
	std::vector <float> z_sel;
	kinect_points.clear();

	math::CMatrixDouble44 pose_trans;
	kinect_pose.getHomogeneousMatrix(pose_trans);

	if (abs(kinect_pose[5]) < 0.4f*M_PI)
	{
		// Obtain the nearest point of each colum at each height level
		//------------------------------------------------------------
		for ( int col = 0; col < width; col += downsample )
		{
			x_sel.assign(levels.size(),20);
			y_sel.assign(levels.size(),20);
			z_sel.assign(levels.size(),20);

			for ( int row = 0; row < height; row += downsample )
			{			
				//Obtain the depth of a pixel
				const float xloc = depthImage.at<float>(row,col); 
				
				//Consider the point if it has valid depth information
				if(mrpt::math::isFinite(xloc)&&(xloc >= min_depth && xloc <= max_depth)) 
				{		
					//Fast transformation
					const float yloc = (ox - col)*xloc*inv_f + lens_disp;
					const float zloc = (oy - row)*xloc*inv_f;

					const float xtrans = xloc*pose_trans(0,0) + yloc*pose_trans(0,1) + zloc*pose_trans(0,2) + pose_trans(0,3);
					const float ytrans = xloc*pose_trans(1,0) + yloc*pose_trans(1,1) + zloc*pose_trans(1,2) + pose_trans(1,3);
					const float ztrans = xloc*pose_trans(2,0) + yloc*pose_trans(2,1) + zloc*pose_trans(2,2) + pose_trans(2,3);
			
					for (unsigned int i=0; i<levels.size(); i++)
					{
						if (i == 0)
						{
							if ((ztrans < levels[i])&&(ztrans >= floor_limit))
							{
								if (xloc < x_sel[0])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
						else
						{
							if ((ztrans < levels[i])&&(ztrans >= levels[i-1]))
							{
								if (xloc < x_sel[i])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
					}
				}
			}

			//Insert the most restrictive points in "kinect_points"
			for (unsigned int i = 0; i < levels.size(); i++)
				if (x_sel[i] != 20)
					kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);

		}
	}
	else
	{
		// Obtain the nearest point of each row at each height level
		//-------------------------------------------------------------
		for ( int row = 0; row < height; row += downsample )
		{
			x_sel.assign(levels.size(),20);
			y_sel.assign(levels.size(),20);
			z_sel.assign(levels.size(),20);

			for ( int col = 0; col < width; col += downsample )
			{			
				//Obtain the depth of a pixel
				const float xloc = depthImage.at<float>(row,col); 
				
				//Consider the point if it has valid depth information
				if(mrpt::math::isFinite(xloc)&&(xloc >= min_depth && xloc <= max_depth)) 
				{			
					//Fast transformation
					const float yloc = (ox - col)*xloc*inv_f + lens_disp;
					const float zloc = (oy - row)*xloc*inv_f;

					const float xtrans = xloc*pose_trans(0,0) + yloc*pose_trans(0,1) + zloc*pose_trans(0,2) + pose_trans(0,3);
					const float ytrans = xloc*pose_trans(1,0) + yloc*pose_trans(1,1) + zloc*pose_trans(1,2) + pose_trans(1,3);
					const float ztrans = xloc*pose_trans(2,0) + yloc*pose_trans(2,1) + zloc*pose_trans(2,2) + pose_trans(2,3);
			
					for (unsigned int i=0; i<levels.size(); i++)
					{
						if (i == 0)
						{
							if ((ztrans < levels[i])&&(ztrans >= floor_limit))
							{
								if (xloc < x_sel[0])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
						else
						{
							if ((ztrans < levels[i])&&(ztrans >= levels[i-1]))
							{
								if (xloc < x_sel[i])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
					}
				}
			}

			//Insert the most restrictive points in "kinect_points"
			for (unsigned int i = 0; i < levels.size(); i++)
				if (x_sel[i] != 20)
					kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);

		}
	}
}


//-------------------------------------------------------------------------------------
// Obtains the locations of the closests points based on depth and infrared images
//-------------------------------------------------------------------------------------
void CRGBD_App::GetClosestPointsPS()
{
	openni::VideoFrameRef framed, frameir;
	depth.readFrame(&framed);
	ir.readFrame(&frameir);

	// DEPTH
	const int height = framed.getHeight();
	const int width = framed.getWidth();

	const float hFov = depth.getHorizontalFieldOfView();
	const float vFov = depth.getVerticalFieldOfView();
	const float inv_f = (2.0f * tan (hFov / 2.0f))/width;
	const float ox = 0.5f*float(width-1);
	const float oy = 0.5f*float(height-1);

	unsigned int close_count = 0;

	math::CMatrixFloat depthmat;
	depthmat.setSize(height,width);
	

	//						Read new frames and check infrared saturation
	//=================================================================================================
	const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
	const openni::Grayscale16Pixel* pInfraredRow = (const openni::Grayscale16Pixel*)frameir.getData();
	int rowSize = framed.getStrideInBytes() / sizeof(openni::DepthPixel);
		
	// Process all px in the image
	for (int yc = height-1; yc >= 0; --yc)
	{
		const openni::DepthPixel* pDepth = pDepthRow;
		const openni::Grayscale16Pixel* pInfrared = pInfraredRow;
		for (int xc = width-1; xc >= 0; --xc, ++pDepth, ++pInfrared)
		{
			// check IR value
			if( discard_high_ir_points && (*pInfrared > ir_threshold) )
				depthmat(yc,xc) = 0.f;			//Discard point
			else
				depthmat(yc,xc) = 0.001f*(*pDepth);

			// Check if detecting close obstacles is enabled
			if( detect_close_obstacles_with_ir && (*pInfrared > ir_threshold) && (*pDepth == 0.f) )
				close_count++;				
		}
		pDepthRow += rowSize;
		pInfraredRow += rowSize;
	}

	//Publish proximity warning to the Reactive Module
	if (close_count <= count_threshold)
		m_Comms.Notify("IRD_WARNING", "false" );
	else
		m_Comms.Notify("IRD_WARNING", "true" );


	//---------------------------------------------------------------------------
	//						Process the point set
	//---------------------------------------------------------------------------
	kinect_points.clear();
	
	std::vector <float> x_sel;
	std::vector <float> y_sel;
	std::vector <float> z_sel;

	math::CMatrixDouble44 pose_trans;
	kinect_pose.getHomogeneousMatrix(pose_trans);

	if ( abs(kinect_pose[5]) < 0.4f*M_PI)
	{
		// Obtain the nearest point of each colum at each height level
		//------------------------------------------------------------
		for ( unsigned int col = 0; col < width; col += downsample )
		{
			x_sel.assign(levels.size(),20);
			y_sel.assign(levels.size(),20);
			z_sel.assign(levels.size(),20);

			for ( unsigned int row = 0; row < height; row += downsample )
			{			
				//Obtain the depth of a pixel
				const float xloc = depthmat(row,col); 

				//Consider the point if it has valid depth information
				if(xloc >= min_depth && xloc <= max_depth)
				{
					//Fast transformation
					const float yloc = (col - ox)*xloc*inv_f + lens_disp;
					const float zloc = (row - oy)*xloc*inv_f;

					const float xtrans = xloc*pose_trans(0,0) + yloc*pose_trans(0,1) + zloc*pose_trans(0,2) + pose_trans(0,3);
					const float ytrans = xloc*pose_trans(1,0) + yloc*pose_trans(1,1) + zloc*pose_trans(1,2) + pose_trans(1,3);
					const float ztrans = xloc*pose_trans(2,0) + yloc*pose_trans(2,1) + zloc*pose_trans(2,2) + pose_trans(2,3);
			
					for (unsigned int i=0; i<levels.size(); i++)
					{
						if (i == 0)
						{
							if ((ztrans < levels[i])&&(ztrans >= floor_limit))
							{
								if (xloc < x_sel[0])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
						else
						{
							if ((ztrans < levels[i])&&(ztrans >= levels[i-1]))
							{
								if (xloc < x_sel[i])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
					}
				}
			}

			//Insert the most restrictive points in "kinect_points"
			for (unsigned int i = 0; i < levels.size(); i++)
				if (x_sel[i] != 20)
					kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);

		}
	}
	else
	{
		// Obtain the nearest point of each row at each height level
		//-------------------------------------------------------------
		for ( int row = 0; row < height; row += downsample )
		{
			x_sel.assign(levels.size(),20);
			y_sel.assign(levels.size(),20);
			z_sel.assign(levels.size(),20);

			for ( unsigned int col = 0; col < width; col += downsample )
			{			
				//Obtain the depth of a pixel
				const float xloc = depthmat(row,col); 

				//Consider the point if it has valid depth information
				if(xloc >= min_depth && xloc <= max_depth)
				{
					//Fast transformation
					const float yloc = (col - ox)*xloc*inv_f + lens_disp;
					const float zloc = (row - oy)*xloc*inv_f;

					const float xtrans = xloc*pose_trans(0,0) + yloc*pose_trans(0,1) + zloc*pose_trans(0,2) + pose_trans(0,3);
					const float ytrans = xloc*pose_trans(1,0) + yloc*pose_trans(1,1) + zloc*pose_trans(1,2) + pose_trans(1,3);
					const float ztrans = xloc*pose_trans(2,0) + yloc*pose_trans(2,1) + zloc*pose_trans(2,2) + pose_trans(2,3);
			
					for (unsigned int i=0; i<levels.size(); i++)
					{
						if (i == 0)
						{
							if ((ztrans < levels[i])&&(ztrans >= floor_limit))
							{
								if (xloc < x_sel[0])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
						else
						{
							if ((ztrans < levels[i])&&(ztrans >= levels[i-1]))
							{
								if (xloc < x_sel[i])
								{
									x_sel[i] = xtrans;
									y_sel[i] = ytrans;
									z_sel[i] = ztrans;
								}
							}
						}
					}
				}
			}

			//Insert the most restrictive points in "kinect_points"
			for (unsigned int i = 0; i < levels.size(); i++)
				if (x_sel[i] != 20)
					kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);

		}
	}
}



void CRGBD_App::InitializeScene()
{	
	m_window = gui::CDisplayWindow3D::Create();
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;  //I don't know how this influences the scene...

	m_window->setWindowTitle("Kinect points");
	m_window->resize(800,600);
	m_window->setPos(50,0);
	m_window->setCameraPointingToPoint(0,0,1.5);
	m_window->setCameraZoom(12);
	m_window->setCameraAzimuthDeg(190);
	m_window->setCameraElevationDeg(15);
	m_scene = m_window->get3DSceneAndLock();

	opengl::CGridPlaneXYPtr obj2 = opengl::CGridPlaneXY::Create(-2,2,-2,2,0);
	m_scene->insert( obj2 );

	opengl::CPointCloudPtr obj3 = opengl::CPointCloud::Create();
	obj3->insertPoint(0,0,0);
	obj3->setColor(0,0,1);
	obj3->setPointSize(3.0);
	obj3->enablePointSmooth();
	m_scene->insert( obj3 );

	m_window->unlockAccess3DScene();
	m_window->repaint();
}


void CRGBD_App::ShowPoints()
{
	m_scene = m_window->get3DSceneAndLock();

	opengl::CPointCloudPtr obj2 = opengl::CPointCloud::Create();
	obj2 = m_scene->getByClass <mrpt::opengl::CPointCloud> (0);
	obj2->loadFromPointsMap<mrpt::maps::CSimplePointsMap> (&kinect_points);

	m_window->unlockAccess3DScene();
	m_window->repaint();
}