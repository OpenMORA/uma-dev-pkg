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


/**  @moos_module This module implementes the adquisition of color and depth images from a RGBD sensor (PrimeSense/Kinect) and process it.
  *  The processing step is focused in the detection of obstacles for reactive navigation. The module published a list of points
  *  where obstacles have been detected at different heights. Since processing all the points in the image will be computationally expensive
  *  the parameters allow defining at which heights the processing will be carried out.
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


CRGBD_App::CRGBD_App()
{
}

CRGBD_App::~CRGBD_App()
{
}


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
			GetClosestPointsPS();
		else
			GetClosestPointsKinect();


		// Publish detected poitns as OpenMORA variable
		string sKinect1 = ObjectToString(&kinect_points);
		m_Comms.Notify("KINECT1", sKinect1 );


		if (m_visualization)
			ShowPoints();

		cout << endl << "[RGBD]: Iteration period (approximate): " << GetTimeSinceIterate() << " seconds";
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
	//! @moos_param  levels  The number of levels (heights) to process the image from the RGBD camera
	levels.resize(m_ini.read_int("","levels",1,true));

	//! @moos_param  heights  Vector of cumulative heights (cm) where the processing will be carried out.
	m_ini.read_vector("","heights", levels, levels, true);

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
	//! @moos_param  New_sensor  Indicates the type of RGBD camera: TRUE=PrimeSens, FALSE=Kinect
	m_new_sensor = m_ini.read_bool("","New_sensor", 0, false);

	//! @moos_param  Res_width  Camera resolution(px) - Shared for color, depth and infrared images (320 or 640)
	res_width = m_ini.read_float("","Res_width", 320.0, false);

	//! @moos_param  Res_height  Camera resolution(px) - Shared for color, depth and infrared images (240 or 480)
	res_height = m_ini.read_float("","Res_height", 240.0, false);

	//! @moos_param  min_depth  The minimum depth (m) the camera is able to detect
	min_depth = m_ini.read_float("","min_depth",0.3,true);
	
	//! @moos_param  max_depth  The max depth (m) the camera is able to detect
	max_depth = m_ini.read_float("","max_depth",5,true);

	// CAMERA POSE
	//------------
	//! @moos_param  pose_x  The X position (m) of the camera on the robot
	kinect_pose.x(m_ini.read_float("","pose_x", 0, true));

	//! @moos_param  pose_y  The Y position (m) of the camera on the robot
	//! @moos_param  lens_displacement  Distance (m) between the infrared emiter and receiver in the camera
	kinect_pose.y(m_ini.read_float("","pose_y", 0, true) + m_ini.read_float("","lens_displacement", 0.012, true));
	
	//! @moos_param pose_z  The Z position (m) of the camera on the robot
	kinect_pose.z(m_ini.read_float("","pose_z", 0, true));
	
	//! @moos_param  pose_yaw  The yaw angle (degreees) of the camera on the robot
	//! @moos_param  pose_pitch  The pitch angle (degreees) of the camera on the robot
	//! @moos_param  pose_roll  The roll angle (degreees) of the camera on the robot
	kinect_pose.setYawPitchRoll(DEG2RAD(m_ini.read_float("","pose_yaw", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_pitch", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_roll", 0, false)));
}


//-------------------------------------------------------------
// Open a PrimeSense device using OpenNI2, and configure it
//--------------------------------------------------------------
bool CRGBD_App::OpenPSdevice()
{
	//==============================================================================================
	//									Open Carmine 1.08 or 1.09
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


	//						Configure some properties (resolution)
	//========================================================================================
	rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	
	// DEPTH
	options = depth.getVideoMode();
	options.setResolution(res_width,res_height);
	rc = depth.setVideoMode(options);
	rc = depth.setMirroringEnabled(false);
	options = depth.getVideoMode();
	printf("[RGBD] Depth resolution set to: (%d, %d)px \n", options.getResolutionX(), options.getResolutionY());
	//depth.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	
	// IR
	options = ir.getVideoMode();
	options.setResolution(res_width,res_height);
	rc = ir.setVideoMode(options);
	rc = ir.setMirroringEnabled(false);
	options = depth.getVideoMode();
	printf("[RGBD] Infrared resolution set to: (%d, %d)px \n", options.getResolutionX(), options.getResolutionY());


	//								Start channels
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
// Open a Kinect device
//-----------------------------------------
bool CRGBD_App::OpenKinect()
{

	//==============================================================================================
	//											Open kinect
	//==============================================================================================

	cout << "Kinect opening ...";
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
	if( !capture->grab() )
    {
		std::cout << "Can not grab images." << std::endl;
        return;
    }

	//Capture the current frame
	cv::Mat depthImage;
    capture->retrieve( depthImage, CV_CAP_OPENNI_DEPTH_MAP );
    depthImage.convertTo( depthImage, CV_32FC1, 1./1000 );

	
	const float inv_fx = 1.f/525.f;
	const float inv_fy = 1.f/525.f;
	const float ox = 319.5;
	const float oy = 239.5;
	float		point_height;

	unsigned int height = depthImage.rows;
	unsigned int width = depthImage.cols;

	CPose3D pose_point;
	CPose3D point_transformed;

	std::vector <float> x_sel;
	std::vector <float> y_sel;
	std::vector <float> z_sel;
	std::vector <float> d_sel;
	kinect_points.clear();


	//---------------------------------------------------------------------------
	//						Kinect in horizontal position
	//---------------------------------------------------------------------------
	if (kinect_pose.roll() == 0)
	{
		// Obtain the nearest point of each colum at each height level
		for ( unsigned int col = 0; col < width; col++ )// col = x
		{
			x_sel.assign(levels.size(),20);
			y_sel.assign(levels.size(),20);
			z_sel.assign(levels.size(),20);
			d_sel.assign(levels.size(),20);

			for ( unsigned int row = 0; row < height; row++ ) //row = y
			{			
				//Obtain the depth of a pixel
				float z = depthImage.at<float>(row,col); 

				
				//Consider the point if it has valid depth information
				if(mrpt::math::isFinite(z)&&(z >= min_depth && z <= max_depth)) 
				{
					// Transform points to the absolute reference of coordinates
					point_height = -(row - oy) * z * inv_fy + kinect_pose[2];
			
					for (unsigned int i=0; i<levels.size(); i++)
					{
						if (i == 0)
						{
							if ((point_height < levels[i])&&(point_height>=floor_limit))
							{
								if (z < d_sel[0])
								{
									pose_point.setFromValues(z, -(col - ox) * z * inv_fx, -(row - oy) * z * inv_fy, 0, 0, 0);
									point_transformed = kinect_pose + pose_point;
									x_sel[i] = point_transformed[0];
									y_sel[i] = point_transformed[1];
									z_sel[i] = point_transformed[2];									
									d_sel[i] = z;
								}
							}
						}
						else
						{
							if ((point_height < levels[i])&&(point_height>=levels[i-1]))
							{
								if (z < d_sel[i])
								{
									pose_point.setFromValues(z, -(col - ox) * z * inv_fx, -(row - oy) * z * inv_fy, 0, 0, 0);
									point_transformed = kinect_pose + pose_point;
									x_sel[i] = point_transformed[0];
									y_sel[i] = point_transformed[1];
									z_sel[i] = point_transformed[2];									
									d_sel[i] = z;
								}
							}
						}
					}
				}
			}

			//Insert the most restrictive points in "kinect_points"
			for (unsigned int i = 0; i < levels.size(); i++)
			{
				if (x_sel[i] != 20)
					kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);
			}
		}
	}

	//---------------------------------------------------------------------------
	//						Kinect in vertical position
	//---------------------------------------------------------------------------
	else
	{
		// Obtain the nearest point of each colum at each height level
		for ( unsigned int row = 0; row < height; row++ )// col = x
		{
			x_sel.assign(levels.size(),20);
			y_sel.assign(levels.size(),20);
			z_sel.assign(levels.size(),20);
			d_sel.assign(levels.size(),20);

			for ( unsigned int col = 0; col < width; col++ ) //row = y
			{			
				//Obtain the depth of a pixel
				float z = depthImage.at<float>(row,col); 

				//Consider the point if it has valid depth information
				if(mrpt::math::isFinite(z)&&(z >= min_depth && z <= max_depth)) 
				{
					// Transform points to the absolute reference of coordinates
					point_height = -(col - ox) * z * inv_fx + kinect_pose[2];
			
					for (unsigned int i=0; i<levels.size(); i++)
					{
						if (i == 0)
						{
							if ((point_height < levels[i])&&(point_height>=floor_limit))
							{
								if (z < d_sel[0])
								{
									pose_point.setFromValues(z, -(col - ox) * z * inv_fx, -(row - oy) * z * inv_fy, 0, 0, 0);
									point_transformed = kinect_pose + pose_point;
									x_sel[i] = point_transformed[0];
									y_sel[i] = point_transformed[1];
									z_sel[i] = point_transformed[2];
									d_sel[i] = z;
								}
							}
						}
						else
						{
							if ((point_height < levels[i])&&(point_height>=levels[i-1]))
							{
								if (z < d_sel[i])
								{
									pose_point.setFromValues(z, -(col - ox) * z * inv_fx, -(row - oy) * z * inv_fy, 0, 0, 0);
									point_transformed = kinect_pose + pose_point;
									x_sel[i] = point_transformed[0];
									y_sel[i] = point_transformed[1];
									z_sel[i] = point_transformed[2];
									d_sel[i] = z;
								}
							}
						}
					}
				}
			}

			//Insert the most restrictive points in "kinect_points"
			for (unsigned int i = 0; i < levels.size(); i++)
			{
				if (x_sel[i] != 20)
					kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);
			}
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

	float hFov = depth.getHorizontalFieldOfView();
	float fx = width / (2.0f * tan (hFov / 2.0f));
	float vFov = depth.getVerticalFieldOfView();
	float fy = height / (2.0f * tan (vFov / 2.0f)); 

	const float inv_fx = 1.0/fx;
	const float inv_fy = 1.0/fy;
		
	const float ox = 0.5*float(width-1);
	const float oy = 0.5*float(height-1);

	float	point_height;
	float	close_count;
	CPose3D pose_point;
	CPose3D point_transformed;

	std::vector <float> x_sel;
	std::vector <float> y_sel;
	std::vector <float> z_sel;
	std::vector <float> d_sel;

	math::CMatrixFloat depthmat;
	depthmat.setSize(height,width);
	
	kinect_points.clear();
	close_count = 0;


	// Check frames consistency (Depth vs IR)
	if( (framed.getWidth() != frameir.getWidth()) || (framed.getHeight() != frameir.getHeight()) )	
		cout << "[RGBD]: ERROR - IR and Depth frams don't have the same size." << endl;	
	else
	{
		//Read one frame
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
					depthmat(yc,xc) = 0.0;			//Discard point
				else
					depthmat(yc,xc) = 0.001*(*pDepth);

				// Check if detecting close obstacles is enabled
				if( detect_close_obstacles_with_ir && (*pInfrared > ir_threshold) && (*pDepth == 0) )
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
		//						Kinect in horizontal position
		//---------------------------------------------------------------------------
		if ( kinect_pose.roll() == 0)
		{
			for ( unsigned int col = 0; col < width; col++ )// col = x
			{
				x_sel.assign(levels.size(),20);
				y_sel.assign(levels.size(),20);
				z_sel.assign(levels.size(),20);
				d_sel.assign(levels.size(),20);

				for ( unsigned int row = 0; row < height; row++ ) //row = y
				{			
					//Obtain the depth of a pixel
					float z = depthmat(row,col); 

					//Consider the point if it has valid depth information
					if(z >= min_depth && z <= max_depth)
					{
						// Transform points to the absolute reference of coordinates
						point_height = (row - oy) * z * inv_fy + kinect_pose[2];
			
						for (unsigned int i=0; i<levels.size(); i++)
						{
							if (i == 0)
							{
								if ((point_height < levels[i])&&(point_height>=floor_limit))
								{
									if (z < d_sel[0])
									{
										pose_point.setFromValues(z, (col - ox) * z * inv_fx, (row - oy) * z * inv_fy, 0, 0, 0);
										point_transformed = kinect_pose + pose_point;
										x_sel[i] = point_transformed[0];
										y_sel[i] = point_transformed[1];
										z_sel[i] = point_transformed[2];
										d_sel[i] = z;
									}
								}
							}
							else
							{
								if ((point_height < levels[i])&&(point_height>=levels[i-1]))
								{
									if (z < d_sel[i])
									{
										pose_point.setFromValues(z, (col - ox) * z * inv_fx, (row - oy) * z * inv_fy, 0, 0, 0);
										point_transformed = kinect_pose + pose_point;
										x_sel[i] = point_transformed[0];
										y_sel[i] = point_transformed[1];
										z_sel[i] = point_transformed[2];
										d_sel[i] = z;
									}
								}
							}
						}
					}
				}

				//Insert the most restrictive points in "kinect_points"
				for (unsigned int i = 0; i < levels.size(); i++)
				{
					if (x_sel[i] != 20)
						kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);
				}
			}
		}

		//---------------------------------------------------------------------------
		//						Kinect in vertical position
		//---------------------------------------------------------------------------
		else
		{
			// Obtain the nearest point of each colum at each height level
			for ( unsigned int row = 0; row < height; row++ )// col = x
			{
				x_sel.assign(levels.size(),20);
				y_sel.assign(levels.size(),20);
				z_sel.assign(levels.size(),20);
				d_sel.assign(levels.size(),20);

				for ( unsigned int col = 0; col < width; col++ ) //row = y
				{			
					//Obtain the depth of a pixel
					float z = depthmat(row,col); 

					//Consider the point if it has valid depth information
					if(mrpt::math::isFinite(z)&&(z >= min_depth && z <= max_depth)) 
					{
						// Transform points to the absolute reference of coordinates
						point_height = (col - ox) * z * inv_fx + kinect_pose[2];
			
						for (unsigned int i=0; i<levels.size(); i++)
						{
							if (i == 0)
							{
								if ((point_height < levels[i])&&(point_height>=floor_limit))
								{
									if (z < d_sel[0])
									{
										pose_point.setFromValues(z, (col - ox) * z * inv_fx, (row - oy) * z * inv_fy, 0, 0, 0);
										point_transformed = kinect_pose + pose_point;
										x_sel[i] = point_transformed[0];
										y_sel[i] = point_transformed[1];
										z_sel[i] = point_transformed[2];
										d_sel[i] = z;
									}
								}
							}
							else
							{
								if ((point_height < levels[i])&&(point_height>=levels[i-1]))
								{
									if (z < d_sel[i])
									{
										pose_point.setFromValues(z, (col - ox) * z * inv_fx, (row - oy) * z * inv_fy, 0, 0, 0);
										point_transformed = kinect_pose + pose_point;
										x_sel[i] = point_transformed[0];
										y_sel[i] = point_transformed[1];
										z_sel[i] = point_transformed[2];
										d_sel[i] = z;
									}
								}
							}
						}
					}
				}

				//Insert the most restrictive points in "kinect_points"
				for (unsigned int i = 0; i < levels.size(); i++)
				{
					if (x_sel[i] != 20)
						kinect_points.insertPointFast(x_sel[i],y_sel[i],z_sel[i]);
				}
			}
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