/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2013  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Mariano Jaimez Tarifa  <marianojt@.uma.es>                    |
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

#include <iostream>
#include <stdio.h>
#include <string.h>

#include "CKinect_Odometry_App.h"


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------

CKinect_Odometry_App::CKinect_Odometry_App()
{
	num_iter = 0;
	e_icp.assign(6,0);
	e_dif.assign(6,0);
	tm_icp = 0;
	tm_dif = 0;	
	
	m_fovh = m_dif.fovh;
	m_fovv = m_dif.fovv;
	m_rows = m_dif.rows;
	m_cols = m_dif.cols;
	min_depth = 0.5;
	max_depth = 5;

	m_kinect_pose.setFromValues(-0.032,-0.545,0.23,-2.62,1.1,-2.13);
	m_kinect_pose_absincr.setFromValues(0,0,0,0,0,0);

	m_working = 0;
	m_use_icp = 0;
}

CKinect_Odometry_App::~CKinect_Odometry_App()
{
}

bool CKinect_Odometry_App::OnStartUp()
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
		cameraMatrix << 525.,0.,3.1950000000000000e+02,0.,525.,2.3950000000000000e+02,0.,0.,1.;
		m_visualization = m_ini.read_bool("","Opengl_scene", 0, false);
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

bool CKinect_Odometry_App::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("This module only accepts string command messages\n");

    std::string sCmd = Msg.GetString();
//  MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CKinect_Odometry_App::Iterate()
{
	try
	{
		if (m_working == 0)
		{
			;
		}
		else
		{
			cout << endl << "Time interval: " << m_clock_general.Tac();
			cout << endl << "Pose: " << m_kinect_pose << endl;
			m_dif.t_incr_inv = 1/m_clock_general.Tac();
			m_clock_general.Tic();
			m_kinect_oldpose = m_kinect_pose;
			CaptureFrame();
			OdometryCalculation();
			if (m_visualization)
				UpdateScene();
		}
		return 1;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CKinect_Odometry_App::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CKinect_Odometry_App::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
	AddMOOSVariable( "WORKING", "WORKING", "WORKING", 0 );
	AddMOOSVariable( "RESET_ODO", "RESET_ODO", "RESET_ODO", 0);
	AddMOOSVariable( "ARM_POSE", "ARM_POSE", "ARM_POSE", 0);
    RegisterMOOSVariables();

    return true;
}


bool CKinect_Odometry_App::OnNewMail(MOOSMSG_LIST &NewMail)
{
    UpdateMOOSVariables(NewMail);

    for (MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
    {
    	try
    	{
			if ((i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")))
			{
				// Disconnect comms:
				MOOSTrace("Closing Module \n");
				this->RequestQuit();
			}
			else if ((i->GetName()=="WORKING") && (MOOSStrCmp(i->GetString(),"true")))
			{
				m_clock_general.Tic();
				m_working = 1;
				CaptureFrame();

				char name[30];
				bool name_exist = 1;
				short cont = 1;
				while (name_exist)
				{
					sprintf(name,"Pose_kinect%d.txt",cont);
					name_exist = system::fileExists(name);
					cont++;
				}
				m_filek.open(name);
				sprintf(name,"Pose_arm%d.txt",cont-1);
				m_filea.open(name);
			}
			else if ((i->GetName()=="WORKING") && (MOOSStrCmp(i->GetString(),"false")))
			{
				m_working = 0;
				m_filek.close();
				m_filea.close();
			}
			else if (i->GetName()=="RESET_ODO")
			{
				ResetOdometry();
			}
			else if (i->GetName()=="ARM_POSE")
			{
				m_arm_pose.fromString(i->GetString());
				m_filek << m_kinect_pose.asString() << endl;
				m_filea << m_arm_pose.asString() << endl;
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

void CKinect_Odometry_App::InitializeScene()
{
	m_window = gui::CDisplayWindow3D::Create();
	global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
	m_window->resize(1000,900);
	m_window->setPos(900,0);
	m_window->setCameraZoom(16);
	m_window->setCameraAzimuthDeg(0);
	m_window->setCameraElevationDeg(90);
	m_window->setCameraPointingToPoint(0,0,0);
	m_window->setCameraPointingToPoint(0,0,1);

	m_scene = m_window->get3DSceneAndLock();

	//Grid (floor)
	m_floor = CGridPlaneXY::Create();
	m_scene->insert( m_floor );

	//Reference
	m_reference = stock_objects::CornerXYZ();
	m_scene->insert( m_reference );

	//Kinect
	m_kinect = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	m_kinect->setPose(m_kinect_pose);
	m_kinect->setColor(0,0,0);
	m_scene->insert( m_kinect );

	//Frustum
	m_FOV = CFrustum::Create(0.5, 6, 57.3*m_fovh, 57.3*m_fovv, 1.5f, true, false);
	m_FOV->setPose(m_kinect_pose);
	m_scene->insert( m_FOV );

	//Kinect points
	m_kin_points = CPointCloud::Create();
	m_kin_points->setColor(1,0,0);
	m_kin_points->setPointSize(2);
	m_kin_points->enablePointSmooth(1);
	m_kin_points->setPose(m_kinect_pose);
	m_scene->insert( m_kin_points );

	//Trajectory lines and points
	m_traj_lines = CSetOfLines::Create();
	m_traj_lines->setColor(0,0,0);
	m_traj_lines->setLineWidth(3);
	m_scene->insert( m_traj_lines );
	m_traj_points = CPointCloud::Create();
	m_traj_points->setColor(0,0.5,0);
	m_traj_points->setPointSize(4);
	m_traj_points->enablePointSmooth(1);
	m_scene->insert( m_traj_points );

	//Border points
	m_border_points = CPointCloud::Create();
	m_border_points->setColor(0,0,1);
	m_border_points->setPointSize(4);
	m_border_points->enablePointSmooth(1);
	m_border_points->setPose(m_kinect_pose);
	m_scene->insert( m_border_points );

	////Time derivative
	//m_dt_obj = CMesh::Create();
	//m_dt_obj->setLocation(-2,0,0);
	//m_dt_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_dt_obj->setZ(5*m_dif.dt);
	//m_dt_obj->enableWireFrame(true);
	//m_dt_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_dt_obj );

	//CTextPtr dt_txt = CText::Create("DT");
	//dt_txt->setLocation(-3,0,0);
	//dt_txt->setColor(0,0,0);
	//m_scene->insert(dt_txt);

	////Y derivative
	//m_dy_obj = CMesh::Create();
	//m_dy_obj->setLocation(-2,2,0);
	//m_dy_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_dy_obj->setZ(5*m_dif.dyp);
	//m_dy_obj->enableWireFrame(true);
	//m_dy_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_dy_obj );

	//CTextPtr dy_txt = CText::Create("DY");
	//dy_txt->setLocation(-3,2,0);
	//dy_txt->setColor(0,0,0);
	//m_scene->insert(dy_txt);

	////Z derivative
	//m_dz_obj = CMesh::Create();
	//m_dz_obj->setLocation(-2,-2,0);
	//m_dz_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_dz_obj->setZ(5*m_dif.dzp);
	//m_dz_obj->enableWireFrame(true);
	//m_dz_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_dz_obj );

	//CTextPtr dz_txt = CText::Create("DZ");
	//dz_txt->setLocation(-3,-2,0);
	//dz_txt->setColor(0,0,0);
	//m_scene->insert(dz_txt);

	////Points which are in a border
	//m_border_obj = CMesh::Create();
	//m_border_obj->setLocation(-4,-1,0);
	//m_border_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_border_obj->setZ(m_dif.borders);
	//m_border_obj->enableWireFrame(true);
	//m_border_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_border_obj );

	//CTextPtr border_txt = CText::Create("Border points");
	//border_txt->setLocation(-5,-1,0);
	//border_txt->setColor(0,0,0);
	//m_scene->insert(border_txt);

	////Points NAN detected
	//m_nan_obj = CMesh::Create();
	//m_nan_obj->setLocation(-4,1,0);
	//m_nan_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_nan_obj->setZ(m_dif.Nan);
	//m_nan_obj->enableWireFrame(true);
	//m_nan_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_nan_obj );

	//CTextPtr nan_txt = CText::Create("NAN points");
	//nan_txt->setLocation(-5,1,0);
	//nan_txt->setColor(0,0,0);
	//m_scene->insert(nan_txt);

	////Phi derivative
	//m_dphi_obj = CMesh::Create();
	//m_dphi_obj->setLocation(-6,2,0);
	//m_dphi_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_dphi_obj->setZ(10*m_dif.dphi);
	//m_dphi_obj->enableWireFrame(true);
	//m_dphi_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_dphi_obj );

	//CTextPtr dphi_txt = CText::Create("Dphi");
	//dphi_txt->setLocation(-7,2,0);
	//dphi_txt->setColor(0,0,0);
	//m_scene->insert(dphi_txt);

	////tita derivative
	//m_dtita_obj = CMesh::Create();
	//m_dtita_obj->setLocation(-6,-2,0);
	//m_dtita_obj->setGridLimits(-0.75,0.75,-0.75,0.75);
	//m_dtita_obj->setZ(10*m_dif.dtita);
	//m_dtita_obj->enableWireFrame(true);
	//m_dtita_obj->enableColorFromZ(true, utils::cmJET );
	//m_scene->insert( m_dtita_obj );

	//CTextPtr dtita_txt = CText::Create("Dtita");
	//dtita_txt->setLocation(-7,-2,0);
	//dtita_txt->setColor(0,0,0);
	//m_scene->insert(dtita_txt);

	////ICP old points
	//m_old_points = CPointCloud::Create();
	////for (unsigned int i=0; i<m_icp.cloud_old->size(); i++)
	////{
	////	m_old_points->insertPoint(odo.m_icp.cloud_old->points[i].x, odo.m_icp.cloud_old->points[i].y, odo.m_icp.cloud_old->points[i].z);
	////}
	//m_old_points->setColor(1,0,0);
	//m_old_points->setPointSize(4);
	//m_old_points->enablePointSmooth(1);
	//m_old_points->setLocation(0,-6,0);
	//m_scene->insert( m_old_points );

	////ICP new points
	//m_new_points = CPointCloud::Create();
	////for (unsigned int i=0; i<m_icp.cloud_new->size(); i++)
	////{
	////	m_new_points->insertPoint(odo.m_icp.cloud_new->points[i].x, odo.m_icp.cloud_new->points[i].y, odo.m_icp.cloud_new->points[i].z);
	////}
	//m_new_points->setColor(0,1,0);
	//m_new_points->setPointSize(4);
	//m_new_points->enablePointSmooth(1);
	//m_new_points->setLocation(0,-6,0);
	//m_scene->insert( m_new_points );	

	////ICP resulting points
	//m_icp_points = CPointCloud::Create();
	//m_icp_points->setColor(0,0,1);
	//m_icp_points->setPointSize(4);
	//m_icp_points->enablePointSmooth(1);
	//m_icp_points->setLocation(0,-6,0);
	//m_scene->insert( m_icp_points );

	m_window->unlockAccess3DScene();
	m_window->repaint();

}

void CKinect_Odometry_App::UpdateScene()
{
	m_scene = m_window->get3DSceneAndLock();

	//Kinect
	m_kinect->setPose(m_kinect_pose);

	//Frustum
	m_FOV->setPose(m_kinect_pose);

	//Kinect points
	m_kin_points->setPose(m_kinect_pose);
	m_scene->insert( m_kin_points );
	m_kin_points->clear();
	for (int y=0; y<m_cols; y++)
		for (int z=0; z<m_rows; z++)
		{
			m_kin_points->insertPoint(m_dif.depth(z,y), m_dif.yy(z,y), m_dif.zz(z,y));
			if ((m_dif.borders(z,y) == 1)||(m_dif.Nan(z,y) == 1))
			{
				m_dif.dyp(z,y) = 0;
				m_dif.dzp(z,y) = 0;
				m_dif.dt(z,y) = 0;
			}
		}

	//Trajectory lines and points
	m_traj_points->insertPoint(m_kinect_pose.x(), m_kinect_pose.y(), m_kinect_pose.z());
	m_traj_lines->appendLine(m_kinect_oldpose.x(),m_kinect_oldpose.y(), m_kinect_oldpose.z(), m_kinect_pose.x(), m_kinect_pose.y(), m_kinect_pose.z());

	//Border points
	m_border_points->clear();
	m_border_points->setPose(m_kinect_pose);
	m_border_points->loadFromPointsMap<CSimplePointsMap> (&m_dif.points_borders);
	for (int y=0; y<m_cols; y++)
		for (int z=0; z<m_rows; z++)
			if ((y==0) || (y==m_cols-1) || (z==0) || (z==m_rows-1))
				m_border_points->insertPoint(m_dif.depth(z,y), m_dif.yy(z,y), m_dif.zz(z,y));

	////Time derivative
	//m_dt_obj->setZ(5*m_dif.dt);

	////Y derivative
	//m_dy_obj->setZ(5*m_dif.dyp);

	////Z derivative
	//m_dz_obj->setZ(5*m_dif.dzp);

	////Points which are in a border
	//m_border_obj->setZ(0.5*m_dif.borders);

	////Points NAN detected
	//m_nan_obj->setZ(0.5*m_dif.Nan);

	////Phi derivative
	//m_dphi_obj->setZ(5*m_dif.dphi);

	////tita derivative
	//m_dtita_obj->setZ(5*m_dif.dtita);

	////ICP points
	//old_points->clear();
	//for (unsigned int i=0; i<odo.m_icp.cloud_old->size(); i++)
	//{
	//	old_points->insertPoint(odo.m_icp.cloud_old->points[i].x, odo.m_icp.cloud_old->points[i].y, odo.m_icp.cloud_old->points[i].z);
	//}
	//new_points->clear();
	//for (unsigned int i=0; i<odo.m_icp.cloud_new->size(); i++)
	//{
	//	new_points->insertPoint(odo.m_icp.cloud_new->points[i].x, odo.m_icp.cloud_new->points[i].y, odo.m_icp.cloud_new->points[i].z);
	//}
	//icp_points->clear();
	//for (unsigned int i=0; i<odo.m_icp.cloud_trans->size(); i++)
	//{
	//	icp_points->insertPoint(odo.m_icp.cloud_trans->points[i].x, odo.m_icp.cloud_trans->points[i].y, odo.m_icp.cloud_trans->points[i].z);
	//}

	m_window->unlockAccess3DScene();
	m_window->repaint();
}

void CKinect_Odometry_App::CaptureFrame()
{

	capture->grab();

	//cv::Mat rgbImage;
	cv::Mat depthImage;

	//Capture the current frame
    capture->retrieve( depthImage, CV_CAP_OPENNI_DEPTH_MAP );
    depthImage.convertTo( depthImage, CV_32FC1, 1./1000 );
	//capture->retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE );

	const float inv_fy = 1.f/cameraMatrix(0,0);
    const float inv_fz = 1.f/cameraMatrix(1,1);
    const float oy = cameraMatrix(0,2);
    const float oz = cameraMatrix(1,2);

    const int height = depthImage.rows;
    const int width = depthImage.cols;

	m_dif.entering_cloud->points.clear();
	m_dif.entering_cloud->height = 480;
	m_dif.entering_cloud->width = 640;


    for( int y = width-1; y >= 0; y--)
    {
		for( int z = height-1; z >= 0; z--)
        {
			float x = depthImage.at<float>(z,y);
			//If the point has valid depth information assign the 3D point to the point cloud
            if(pcl_isfinite(x) && x>=min_depth && x<=max_depth) 
            {
                m_dif.entering_cloud->points.push_back(PointXYZ(-(z-oz)*x*inv_fz, -(y-oy)*x*inv_fy, x));
				//m_process.depth_wf(height-z,width-y) = x;
				//m_process.yy_wf(height-z,width-y) = -(y - oy) * x * inv_fy;
				//m_process.zz_wf(height-z,width-y) = -(z - oz) * x * inv_fz;
                //cv::Vec3b& bgr = rgbImage.at<cv::Vec3b>(z,y);
                //m_pointCloudPtr->points[width*y+x].r = bgr[2];
                //m_pointCloudPtr->points[width*y+x].g = bgr[1];
                //m_pointCloudPtr->points[width*y+x].b = bgr[0];
            }
            else //else, assign a NAN value
            {
                m_dif.entering_cloud->points.push_back(PointXYZ(numeric_limits<float>::quiet_NaN(), numeric_limits<float>::quiet_NaN(), numeric_limits<float>::quiet_NaN()));
				//m_process.depth_wf(height-z,width-y) = numeric_limits<float>::quiet_NaN();
				//m_process.yy_wf(height-z,width-y) = numeric_limits<float>::quiet_NaN();
				//m_process.zz_wf(height-z,width-y) = numeric_limits<float>::quiet_NaN();
            }
        }
    }

	//Points to the icp motion estimator
	if (m_use_icp == 1)
	{
		m_icp.cloud_new->swap(*m_icp.cloud_old);
		m_icp.cloud_new->clear();
		m_icp.cloud_new = m_dif.entering_cloud;
		//for (int y=0; y<m_cols; y++)
		//{
		//	for (int z=0; z<m_rows; z++)
		//	{
		//		if (pcl_isfinite(m_process.depth_wf(z,y)))
		//			m_icp.cloud_new->push_back(PointXYZ(m_process.depth_wf(z,y), m_process.yy_wf(z,y), m_process.zz_wf(z,y)));
		//	}
		//}
		m_icp.gicp.setInputCloud(m_icp.cloud_old);
		m_icp.gicp.setInputTarget(m_icp.cloud_new);
	}
}

void CKinect_Odometry_App::OdometryCalculation()
{
	//Differential odometry
	m_clock_dif.Tic();
	m_dif.filterPointsAndUpdate();
	m_dif.calculateCoord();
	m_dif.calculateDyp2t();
	m_dif.calculateDzp2t();
	m_dif.calculateDt();
	m_dif.findBorders();
	m_dif.findNaNPoints();
	m_dif.countPointsInBordersAndNaN();
	m_dif.getPointsInBorders();
	if ((m_cols-2)*(m_rows-2) > m_dif.num_border_points + m_dif.num_nan_points - m_dif.num_border_and_nan + 6)
	{
		m_dif.solveSystemSparse();
	}
	else
	{
		cout << endl << "Not enough points to solve the system";
	}

	tm_dif+= 1000*m_clock_dif.Tac();
	cout << endl << "Differential odometry computational time(ms): " << 1000*m_clock_dif.Tac();

	//ICP Odometry
	if (m_use_icp == 1)
	{
		m_clock_icp.Tic();
		m_icp.solveSystemGeneralized();
		tm_icp+= 1000*m_clock_icp.Tac();
		cout << endl << "ICP odometry computational time(ms): " << 1000*m_clock_icp.Tac() << endl;
	}

	CMatrixDouble33 inv_trans;
	CMatrixDouble31 vel_abs_dif;
	CMatrixDouble31 des_abs_icp;
	CMatrixDouble31 vel_rel_dif;
	CMatrixDouble31	des_rel_icp;

	double yaw,pitch,roll,yawicp,pitchicp,rollicp;
	CMatrixDouble31 w_euler_dif;
	CMatrixDouble31 w_euler_icp;
	CMatrixDouble31 w_inter_icp;

	
	//Translations - Differential odometry
	vel_rel_dif(0,0) = m_dif.Var(0,0);
	vel_rel_dif(1,0) = m_dif.Var(1,0);
	vel_rel_dif(2,0) = m_dif.Var(2,0);
	m_kinect_pose.getRotationMatrix(inv_trans);
	vel_abs_dif = inv_trans*vel_rel_dif;

	//Rotations - Differential odometry
	m_kinect_pose.getYawPitchRoll(yaw,pitch,roll);
	w_euler_dif(0,0) = m_dif.Var(4,0)*sin(roll)/cos(pitch) + m_dif.Var(5,0)*cos(roll)/cos(pitch);
	w_euler_dif(1,0) = m_dif.Var(4,0)*cos(roll) - m_dif.Var(5,0)*sin(roll);
	w_euler_dif(2,0) = m_dif.Var(3,0) + m_dif.Var(4,0)*sin(roll)*tan(pitch) + m_dif.Var(5,0)*cos(roll)*tan(pitch);

	printf("\nRobot speed (relative coordinates): \n (%f, %f, %f, %f, %f, %f) \n \n", m_dif.Var(0,0), m_dif.Var(1,0), m_dif.Var(2,0), m_dif.Var(3,0), m_dif.Var(4,0), m_dif.Var(5,0));
	printf("Robot speed (absolute coordinates): \n (%f, %f, %f, %f, %f, %f) \n \n", vel_abs_dif(0,0), vel_abs_dif(1,0), vel_abs_dif(2,0), w_euler_dif(0,0), w_euler_dif(1,0), w_euler_dif(2,0));

	//Update the pose
	m_kinect_pose_absincr.setFromValues(vel_abs_dif(0,0), vel_abs_dif(1,0), vel_abs_dif(2,0), w_euler_dif(0,0), w_euler_dif(1,0), w_euler_dif(2,0));
	m_kinect_pose.x_incr(vel_abs_dif(0,0)/m_dif.t_incr_inv);
	m_kinect_pose.y_incr(vel_abs_dif(1,0)/m_dif.t_incr_inv);
	m_kinect_pose.z_incr(vel_abs_dif(2,0)/m_dif.t_incr_inv);
	m_kinect_pose.setYawPitchRoll(m_kinect_pose.yaw()+w_euler_dif(0,0)/m_dif.t_incr_inv, m_kinect_pose.pitch()+w_euler_dif(1,0)/m_dif.t_incr_inv, m_kinect_pose.roll()+w_euler_dif(2,0)/m_dif.t_incr_inv);

	if (m_use_icp == 1)
	{
		//Translations - ICP odometry
		des_rel_icp(0,0) = m_icp.estimated_pose[0];
		des_rel_icp(1,0) = m_icp.estimated_pose[1];
		des_rel_icp(2,0) = m_icp.estimated_pose[2];
		des_abs_icp = inv_trans*des_rel_icp;

		//Rotations - Differential odometry
		m_icp.estimated_pose.getYawPitchRoll(yawicp,pitchicp,rollicp);
		w_inter_icp(0,0) = rollicp - yawicp*sin(pitchicp);
		w_inter_icp(1,0) = pitchicp*cos(rollicp) + yawicp*cos(pitchicp)*sin(rollicp);
		w_inter_icp(2,0) = -pitchicp*sin(rollicp) + yawicp*cos(pitchicp)*cos(rollicp);
		w_euler_icp(0,0) = w_inter_icp(1,0)*sin(roll)/cos(pitch) + w_inter_icp(2,0)*cos(roll)/cos(pitch);
		w_euler_icp(1,0) = w_inter_icp(1,0)*cos(roll) - w_inter_icp(2,0)*sin(roll);
		w_euler_icp(2,0) = w_inter_icp(0,0) + w_inter_icp(1,0)*sin(roll)*tan(pitch) + w_inter_icp(2,0)*cos(roll)*tan(pitch);

		cout << endl << "ICP transformation (relative coordinates): " << endl << m_icp.estimated_pose << endl << endl;
		printf("ICP transformation (absolute coordinates): \n (%f, %f, %f, %f, %f, %f) \n \n", des_abs_icp(0,0), des_abs_icp(1,0), des_abs_icp(2,0), w_euler_icp(0,0), w_euler_icp(1,0), w_euler_icp(2,0));
	}
}

void CKinect_Odometry_App::ShowStatistics()
{
	printf("\n==============================================================================");
	printf("\n\n Average execution time (icp): %f", tm_icp/num_iter);
	printf("\n Average execution time (dif): %f", tm_dif/num_iter);
	printf("\n\n Average %% relative error (x). ICP y DIF: %f \t %f", e_icp[0]/num_iter, e_dif[0]/num_iter);
	printf("\n Average %% relative error (y). ICP y DIF: %f \t %f", e_icp[1]/num_iter, e_dif[1]/num_iter);
	printf("\n Average %% relative error (z). ICP y DIF: %f \t %f", e_icp[2]/num_iter, e_dif[2]/num_iter);
	printf("\n Average %% relative error ( yaw ). ICP y DIF: %f \t %f", e_icp[3]/num_iter, e_dif[3]/num_iter);
	printf("\n Average %% relative error (pitch). ICP y DIF: %f \t %f", e_icp[4]/num_iter, e_dif[4]/num_iter);
	printf("\n Average %% relative error (roll ). ICP y DIF: %f \t %f \n", e_icp[5]/num_iter, e_dif[5]/num_iter);
	printf("\n==============================================================================\n");
}



//				printf("\n================================================================================");
//				cout << endl << endl << " Initial pose: " << CPose3D(0,0,1.5,0,0,0);
//				cout << endl << " Final pose: " << odo.m_kinect_pose << endl;
//				printf("\n================================================================================");



void CKinect_Odometry_App::ResetOdometry()
{
	m_kinect_pose.setFromValues(-0.032,-0.545,0.23,-2.62,1.1,-2.13);

	//m_scene = m_window.get3DSceneAndLock();
	////m_kin_points->setPose(m_kinect_pose);
	////m_border_points->setPose(m_kinect_pose);
	//m_kinect->setPose(m_kinect_pose);
	//m_FOV->setPose(m_kinect_pose);
	//m_traj_lines->clear();
	//m_traj_points->clear();

	//m_window.unlockAccess3DScene();
	//m_window.repaint();
}

