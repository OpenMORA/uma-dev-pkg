/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2012  University of Malaga                                |
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


/**  @moos_module Kinect data adquisition and processing.*/

#include "CKinect_v1_App.h"
#include <sstream>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


CKinect_v1_App::CKinect_v1_App()
{
}

CKinect_v1_App::~CKinect_v1_App()
{
}


bool CKinect_v1_App::OnStartUp()
{
	try
	{
		DoRegistrations();
		
		cout << "Kinect opening ..." << endl;
		capture = new cv::VideoCapture(CV_CAP_OPENNI);
		cout << "done." << endl;
		if( !capture->isOpened() )
		{
			cout << "Can not open a capture object." << endl;
			return false;
		}

		//Set the Kinect grabbing properties
		capture->set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ ); // default

		ReadConfiguration();
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

bool CKinect_v1_App::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("This module only accepts string command messages\n");

    std::string sCmd = Msg.GetString();
//  MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CKinect_v1_App::Iterate()
{
	try
	{
		if( !capture->grab() )
        {
            std::cout << "Can not grab images." << std::endl;
            return false;
        }
        else
        {
			//Capture the current frame
            capture->retrieve( depthImage, CV_CAP_OPENNI_DEPTH_MAP );
			depthImage.convertTo( depthImage, CV_32FC1, 1./1000 );

			getPointsMap();

			//if (kinect_pose.roll() != 0)
			//	getPointsMap90Roll();
			//else
			//	getPointsMapHorizontal();

			string sKinect1 = ObjectToString(&kinect_points);
			m_Comms.Notify("KINECT1", sKinect1 );

			if (m_visualization)
				ShowPoints();


			cout << endl << "Iteration period (approximate): " << GetTimeSinceIterate() << " seconds"; 
			return true;
		}
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CKinect_v1_App::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CKinect_v1_App::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
    RegisterMOOSVariables();

    return true;
}


bool CKinect_v1_App::OnNewMail(MOOSMSG_LIST &NewMail)
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


void CKinect_v1_App::ReadConfiguration()
{
	levels.resize(m_ini.read_int("","levels",1,true));
	m_ini.read_vector("","heights", levels, levels, true);
	floor_limit = m_ini.read_float("","floor_limit", 0.05, true);
	min_depth = m_ini.read_float("","min_depth",0.3,true);
	max_depth = m_ini.read_float("","max_depth",5,true);
	kinect_pose.x(m_ini.read_float("","pose_x", 0, true));
	kinect_pose.y(m_ini.read_float("","pose_y", 0, true));
	kinect_pose.z(m_ini.read_float("","pose_z", 0, true));
	kinect_pose.setYawPitchRoll(DEG2RAD(m_ini.read_float("","pose_yaw", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_pitch", 0, true)),
								DEG2RAD(m_ini.read_float("","pose_roll", 0, false)));
	m_visualization = m_ini.read_bool("","Opengl_scene", 0, false);
	downsample = m_ini.read_int("", "downsample", 2, false);
	ignore_region = m_ini.read_bool("", "ignore_region", 0, false);
	if (ignore_region == true)
	{
		ignored_rows_first = m_ini.read_int("", "ignored_rows_first", 0, true);
		ignored_rows_last = m_ini.read_int("", "ignored_rows_last", 0, true);
		ignored_cols_first = m_ini.read_int("", "ignored_cols_first", 0, true);
		ignored_cols_last = m_ini.read_int("", "ignored_cols_last", 0, true);
	}
}

void CKinect_v1_App::InitializeScene()
{
	
	m_window = gui::CDisplayWindow3D::Create();
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;  //I don't know how this influences the scene...

	m_window->setWindowTitle("Kinect points");
	m_window->resize(800,600);
	m_window->setPos(50,0);

	m_scene = m_window->get3DSceneAndLock();
	m_window->setCameraPointingToPoint(0,0,1.5);
	m_window->setCameraZoom(12);
	m_window->setCameraAzimuthDeg(190);
	m_window->setCameraElevationDeg(15);

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

void CKinect_v1_App::ShowPoints()
{
	m_scene = m_window->get3DSceneAndLock();


	opengl::CPointCloudPtr obj2 = opengl::CPointCloud::Create();
	obj2 = m_scene->getByClass <CPointCloud> (0);
	obj2->clear();

	vector<float> x;
	vector<float> y;
	vector<float> z;
	x.resize(kinect_points.size());
	y.resize(kinect_points.size());
	z.resize(kinect_points.size());
	kinect_points.getAllPoints(x,y,z);	
	obj2->setAllPointsFast(x,y,z);

	m_window->unlockAccess3DScene();
	m_window->repaint();
}


// Obtain the nearest point of each colum at each height level from the depth image
void CKinect_v1_App::getPointsMapHorizontal()
{
	const float inv_fx = 1.f/525.;
	const float inv_fy = 1.f/525.;
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

void CKinect_v1_App::getPointsMap90Roll()
{
	const float inv_fx = 1.f/525.;
	const float inv_fy = 1.f/525.;
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

void CKinect_v1_App::getPointsMap()
{
	const float inv_f = 1.f/525.;
	const float oy = 319.5f;
	const float oz = 239.5f;

	unsigned int height = depthImage.rows;
	unsigned int width = depthImage.cols;

	CPose3D point_local;
	CPose3D point_transformed;

	std::vector <float> x_sel;
	std::vector <float> y_sel;
	std::vector <float> z_sel;
	std::vector <float> d_sel;
	kinect_points.clear();

	// Obtain the nearest point of each colum at each height level
	for ( unsigned int col = 0; col < width; col += downsample )// col = x
	{
		
		x_sel.assign(levels.size(),20);
		y_sel.assign(levels.size(),20);
		z_sel.assign(levels.size(),20);

		for ( unsigned int row = 0; row < height; row += downsample ) //row = y
		{			
			if ((ignore_region == true)&&(row >= ignored_rows_first)&&(row <= ignored_rows_last)&&(col >= ignored_cols_first)&&(col <= ignored_cols_last))
				continue;			
			
			//Obtain the depth of a pixel
			float x = depthImage.at<float>(row,col); 

			//Consider the point if it has valid depth information
			if(utils::isFinite(x)&&(x >= min_depth && x <= max_depth)) 
			{
				// Transform points to the absolute reference of coordinates
				point_local.x(x); 
				point_local.y(-(col - oy)*x*inv_f + 0.012);
				point_local.z(-(row - oz)*x*inv_f);
				point_transformed = kinect_pose + point_local;
			
				for (unsigned int i=0; i<levels.size(); i++)
				{
					if (i == 0)
					{
						if ((point_transformed[2] < levels[i])&&(point_transformed[2] >= floor_limit))
						{
							if (point_local[0] < x_sel[0])
							{
								x_sel[i] = point_transformed[0];
								y_sel[i] = point_transformed[1];
								z_sel[i] = point_transformed[2];
							}
						}
					}
					else
					{
						if ((point_transformed[2] < levels[i])&&(point_transformed[2] >= levels[i-1]))
						{
							if (point_local[0] < x_sel[i])
							{
								x_sel[i] = point_transformed[0];
								y_sel[i] = point_transformed[1];
								z_sel[i] = point_transformed[2];
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