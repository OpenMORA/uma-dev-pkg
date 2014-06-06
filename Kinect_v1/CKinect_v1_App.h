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


#include <CMapirMOOSApp.h>
#include <mrpt/system.h>
#include <mrpt/slam.h>
#include <mrpt/opengl.h>
#include <mrpt/gui.h>
#include "opencv2/highgui/highgui.hpp"


using namespace mrpt;

class CKinect_v1_App : public CMapirMOOSApp
{
public:
    CKinect_v1_App();
    virtual ~CKinect_v1_App();

protected:
	unsigned int downsample;
	std::vector <float> levels;
	float min_depth;
	float max_depth;
	float floor_limit;
	bool ignore_region;
	unsigned int ignored_rows_first;
	unsigned int ignored_rows_last;
	unsigned int ignored_cols_first;
	unsigned int ignored_cols_last;
	mrpt::slam::CSimplePointsMap	kinect_points;
	mrpt::poses::CPose3D			kinect_pose;

    cv::VideoCapture *capture;

	mrpt::gui::CDisplayWindow3DPtr	m_window;
	mrpt::opengl::COpenGLScenePtr	m_scene;
	bool							m_visualization;
	cv::Mat							depthImage;


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
    
	void ShowPoints();

	void getPointsMapHorizontal(); //Kinect only has yaw angle different from 0

	void getPointsMap90Roll(); //Kinect has a pure 90� roll angle and a customized yaw angle.

	void getPointsMap(); //Get the point map for any pose of the camera.
    
};

