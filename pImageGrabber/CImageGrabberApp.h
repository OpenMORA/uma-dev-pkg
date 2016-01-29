/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CImageGrabberApp_H
#define CImageGrabberApp_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>

#include "opencv2/objdetect/objdetect.hpp"

class CImageGrabberApp : public COpenMORAApp
{

public:
    CImageGrabberApp();
    virtual ~CImageGrabberApp();

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

	// -----------------------------------------------
	// DATA. Your local variables here...
	// -----------------------------------------------
	mrpt::hwdrivers::CCameraSensor  m_camera;			//!< The camera object. See MRPT docs. Will be attached to a rtsp URL
	bool							m_start_grabbing;		//!< init = false, will be set to true when the sync signal is started and everything starts running.
	bool							m_initialized_ok;
	int								m_verbose_level;

	std::string						m_image_dir;
	mrpt::utils::CTicTac			m_tictac;			//!< To measure time spent
	int								m_counter;

	int								m_grabbed_images_counter;
	bool							m_delete_old_files;
	int								m_grabbed_images_th;
	std::deque<std::string>			m_grabbed_files;
	std::deque<std::pair<std::string,std::string> > m_grabbed_stereo_files;
};
#endif
