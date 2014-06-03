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

#include <CMapirMOOSApp.h>
#include <fstream>
#include "Kinect_Odometry.h"

//using namespace mrpt;
//using namespace mrpt::gui;
//using namespace mrpt::opengl;
//using namespace mrpt::math;
//using namespace mrpt::slam;
//using namespace mrpt::opengl;
//using namespace std;
//using namespace pcl;
//using namespace Eigen;

class CKinect_Odometry_App : public CMapirMOOSApp
{
public:
    CKinect_Odometry_App();
	virtual ~CKinect_Odometry_App();

private:
	//Opengl objects
	CDisplayWindow3DPtr	m_window;
	COpenGLScenePtr		m_scene;
	CSetOfObjectsPtr	m_reference;
	CGridPlaneXYPtr		m_floor;
	CBoxPtr				m_kinect;
	CFrustumPtr			m_FOV;
	CPointCloudPtr		m_kin_points;
	CSetOfLinesPtr		m_traj_lines;
	CPointCloudPtr		m_traj_points;
	CPointCloudPtr		m_border_points;
	CMeshPtr			m_dt_obj;
	CMeshPtr			m_dy_obj;
	CMeshPtr			m_dz_obj;
	CMeshPtr			m_border_obj;
	CMeshPtr			m_nan_obj;
	CMeshPtr			m_dphi_obj;
	CMeshPtr			m_dtita_obj;
	CPointCloudPtr		m_old_points;
	CPointCloudPtr		m_new_points;
	CPointCloudPtr		m_icp_points;

	ofstream			m_filek;
	ofstream			m_filea;
	
	CDifOdo				m_dif;
	CIcpOdo				m_icp;

	CTicTac				m_clock_dif;
	CTicTac				m_clock_icp;
	CTicTac				m_clock_general;

	cv::VideoCapture	*capture;
	Eigen::Matrix3f		cameraMatrix;
	CSimplePointsMap	m_points;

	double	m_fovh;
	double	m_fovv;
	int		m_rows;
	int		m_cols;
	float	min_depth;
	float	max_depth;

	float	tm_icp;
	float	tm_dif;
	vector<float> e_icp;
	vector<float> e_dif;
	unsigned int num_iter;

	CPose3D m_kinect_pose;
	CPose3D m_kinect_oldpose;
	CPose3D m_kinect_pose_absincr;
	CPose3D m_arm_pose;

	bool m_working;
	bool m_use_icp;
	bool m_visualization;


	void CaptureFrame();
	void OdometryCalculation();
	void ShowStatistics();
	void ResetOdometry();


	/** called at startup */
    virtual bool OnStartUp();
    /** called when new mail arrives */
    virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
    /** called when work is to be done */
    virtual bool Iterate();
    /** called when app connects to DB */
    virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
    /** performs the registration for mail */
    bool DoRegistrations();

	void ReadConfiguration();

	void InitializeScene();

	void UpdateScene();
    
};



