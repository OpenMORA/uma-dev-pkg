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

#include <COpenMORAMOOSApp.h>
#include <mrpt/system.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl.h>
#include <mrpt/gui.h>
#include <mrpt/otherlibs/do_opencv_includes.h> // <opencv2\highgui.hpp> // To avoid errors with all versions of OpenCV
#include <OpenNI.h>  // Requires OpenNI2 (right?)


class CRGBD_App : public COpenMORAApp
{
public:
    CRGBD_App();
    virtual ~CRGBD_App();

protected:
	std::vector <float> levels;
	float min_depth;
	float max_depth;
	float floor_limit;
	float res_width;
	float res_height;
	mrpt::maps::CSimplePointsMap	kinect_points;
	mrpt::poses::CPose3D			kinect_pose;

    cv::VideoCapture *capture;

	mrpt::gui::CDisplayWindow3DPtr	m_window;
	mrpt::opengl::COpenGLScenePtr	m_scene;
	bool							m_visualization;
	bool							m_new_sensor;

	openni::Status		rc;
	openni::Device		device;
	openni::VideoMode	options;
	openni::VideoStream depth, ir;

	unsigned ir_threshold;
	unsigned count_threshold;
	bool discard_high_ir_points,detect_close_obstacles_with_ir;

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

	bool OpenPSdevice();

	bool OpenKinect();

	void GetClosestPointsPS();

	void GetClosestPointsKinect();

	void InitializeScene();

	void ShowPoints();

};


