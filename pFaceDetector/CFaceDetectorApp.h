/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CFaceDetectorApp_H
#define CFaceDetectorApp_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>

#include "opencv2/objdetect/objdetect.hpp"

class CFaceDetectorApp : public COpenMORAApp
{

public:
    CFaceDetectorApp();
    virtual ~CFaceDetectorApp();

protected:
	/** called at startup */
	virtual bool OnStartUp();

	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);

	/** called when work is to be done */
	virtual bool Iterate();

	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
	bool DoRegistrations();

	// main method
	bool detectFaces();
	bool getImage();

	// -----------------------------------------------
	// DATA. Your local variables here...
	// -----------------------------------------------
	mrpt::hwdrivers::CCameraSensor  m_camera;			//!< The camera object. See MRPT docs. Will be attached to a rtsp URL
	bool							m_grab_started;		//!< init = false, will be set to true when the sync signal is started and everything starts running.
	bool							m_initialized_ok;
	cv::CascadeClassifier			m_clasifier;		//!< The face cascade classifier

	int								m_verbose_level;
	bool							m_show_view;		//!< Whether or not to show the images 
	bool							m_start_detecting;	//!< Whether or not to start to find faces
	//mrpt::gui::CDisplayWindowPtr	m_display_window;				//!< The display window
	std::vector<mrpt::gui::CDisplayWindowPtr> m_display_windows;	//!< The display windows
	mrpt::utils::CImage				m_image;
	mrpt::utils::CTicTac			m_tictac;			//!< To measure time spent

	unsigned int					m_face_consolidation, m_face_consolidation_th;
	double							m_tout;

	mrpt::system::TTimeStamp		m_ini_time;

	struct TTrackingFaceInfo {
		int id;
		cv::Point p;
		int n_lost;

		TTrackingFaceInfo::TTrackingFaceInfo() : id(0), p(0,0), n_lost(0) {}
		TTrackingFaceInfo::TTrackingFaceInfo(const int _id, const cv::Point _p) : id(_id), p(_p), n_lost(0) {}
	};
	
	// std::vector<cv::Point>			m_faces;			//!< Detected faces
	std::vector<TTrackingFaceInfo>		m_faces;			//!< Detected faces
	// std::vector<bool>				m_faces_up;

};
#endif
