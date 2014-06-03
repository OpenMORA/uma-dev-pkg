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

#include "CRGBD_App.h"
#include <sstream>
#include <iostream>
#include <PS1080.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


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
		
		DoRegistrations();
		ReadConfiguration();

		if (m_new_sensor)
			open_ok = OpenPSdevice();
		else
			open_ok = OpenKinect();

		if (!open_ok)
		{
			cout << endl << "The device can't be openned";
			system::sleep(500);
			return MOOSFail( "Closing the module." );
		}

		if (m_visualization)
			InitializeScene();
		
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
		if (m_new_sensor)
			GetClosestPointsPS();
		else
			GetClosestPointsKinect();


		string sKinect1 = ObjectToString(&kinect_points);
		m_Comms.Notify("KINECT1", sKinect1 );

		if (m_visualization)
			ShowPoints();

		cout << endl << "Iteration period (approximate): " << GetTimeSinceIterate() << " seconds"; 

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


void CRGBD_App::ReadConfiguration()
{
	levels.resize(m_ini.read_int("","levels",1,true));
	m_ini.read_vector("","heights", levels, levels, true);
	floor_limit = m_ini.read_float("","floor_limit", 0.05, true);
	min_depth = m_ini.read_float("","min_depth",0.3,true);
	max_depth = m_ini.read_float("","max_depth",5,true);
	kinect_pose.x(m_ini.read_float("","pose_x", 0, true));
	kinect_pose.y(m_ini.read_float("","pose_y", 0, true) + m_ini.read_float("","lens_displacement", 0.012, true));
	kinect_pose.z(m_ini.read_float("","pose_z", 0, true));
	kinect_pose.setYawPitchRoll(DEG2RAD(m_ini.read_float("","pose_yaw", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_pitch", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_roll", 0, false)));
	m_visualization = m_ini.read_bool("","Opengl_scene", 0, false);
	m_new_sensor = m_ini.read_bool("","New_sensor", 0, false);
	count_threshold = m_ini.read_int("","Count_threshold", 5, false);
	ir_threshold = m_ini.read_int("","IR_threshold", 750, false);
}

bool CRGBD_App::OpenPSdevice()
{
	//==============================================================================================
	//									Open Carmine 1.08 or 1.09
	//==============================================================================================

	const char* deviceURI = openni::ANY_DEVICE;
	rc = openni::OpenNI::initialize();

	printf("After initialization:\n %s\n", openni::OpenNI::getExtendedError());
	printf("No error detected\n");
	rc = device.open(deviceURI);
	printf("Opened OK\n");
	if (rc != openni::STATUS_OK)
	{
		printf("PrimeSense sensor: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 0;
	}

	//								Create RGB and Depth channels
	//========================================================================================

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("PrimeSense sensor: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("PrimeSense sensor: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}


	rc = ir.create(device, openni::SENSOR_IR);
	if (rc == openni::STATUS_OK)
	{
		rc = ir.start();
		if (rc != openni::STATUS_OK)
		{
			printf("PrimeSense sensor: Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			ir.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!depth.isValid() || !ir.isValid())
	{
		printf("Camera: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 0;
	}
	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 0;
	}

	//						Configure some properties (resolution)
	//========================================================================================

	rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	//options = ir.getVideoMode();		
	//options.setResolution(640,480);
	//rc = ir.setVideoMode(options);
	rc = ir.setMirroringEnabled(false);

	//options = depth.getVideoMode();
	//options.setResolution(640,480);
	//rc = depth.setVideoMode(options);	
	rc = depth.setMirroringEnabled(false);
	depth.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);

	options = depth.getVideoMode();
	const int width = options.getResolutionX();
	const int height = options.getResolutionY();
	printf("\nResolution (%d, %d) \n", options.getResolutionX(), options.getResolutionY());

	return 1;
}

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
				if(utils::isFinite(z)&&(z >= min_depth && z <= max_depth)) 
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
				if(utils::isFinite(z)&&(z >= min_depth && z <= max_depth)) 
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

void CRGBD_App::GetClosestPointsPS()
{
	openni::VideoFrameRef framed, frameir;
	depth.readFrame(&framed);
	ir.readFrame(&frameir);

	const int height = framed.getHeight();
	const int width = framed.getWidth();

	float hFov = depth.getHorizontalFieldOfView();
	float fx = width / (2.0f * tan (hFov / 2.0f));
	float vFov = depth.getVerticalFieldOfView();
	float fy = height / (2.0f * tan (vFov / 2.0f)); 

	const float inv_fx = 1.0/fx;
	const float inv_fy = 1.0/fy;

	//const float inv_fx = 1.f/525.f;
	//const float inv_fy = 1.f/525.f;
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


	if ((framed.getWidth() != frameir.getWidth()) || (framed.getHeight() != frameir.getHeight()))
	{
		cout << endl << "Both frames don't have the same size.";
	}
	else
	{
		//Read one frame
		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
		const openni::Grayscale16Pixel* pInfraredRow = (const openni::Grayscale16Pixel*)frameir.getData();
		int rowSize = framed.getStrideInBytes() / sizeof(openni::DepthPixel);
		
		for (int yc = height-1; yc >= 0; --yc)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			const openni::Grayscale16Pixel* pInfrared = pInfraredRow;
			for (int xc = width-1; xc >= 0; --xc, ++pDepth, ++pInfrared)
			{
				depthmat(yc,xc) = 0.001*(*pDepth);
				if ((*pInfrared > ir_threshold)&&(*pDepth == 0))
					close_count++;
			}
			pDepthRow += rowSize;
			pInfraredRow += rowSize;
		}

		//Publish proximity warning
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
					if(utils::isFinite(z)&&(z >= min_depth && z <= max_depth)) 
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
	obj2 = m_scene->getByClass <CPointCloud> (0);
	obj2->loadFromPointsMap<CSimplePointsMap> (&kinect_points);

	m_window->unlockAccess3DScene();
	m_window->repaint();
}

