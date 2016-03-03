/**  @moos_module Module to detect faces in a stream of images grabbed from a webcam
  *  This module gets images from a webcam or an IP cam (or video stream) and looks for faces in the images
  */

#include "CImageGrabberApp.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "mrpt/utils/CTimeLogger.h"
#include "mrpt/utils/CConfigFileMemory.h"
#include "mrpt/utils/CConfigFile.h"
#include "mrpt/obs/CObservationImage.h"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace mrpt::utils;
using namespace mrpt::obs;

#define VERBOSE_LEVEL(_LEVEL) if(m_verbose_level>=_LEVEL) cout

enum CAM_TYPE {RTSP,WEBCAM};

/** Default Constructor */
CImageGrabberApp::CImageGrabberApp() : 
	m_start_grabbing(false),
	m_initialized_ok(false), 
	m_verbose_level(1),
	m_image_dir("."),
	m_counter(0), 
	m_grabbed_images_counter(0), 
	m_delete_old_files(false),
	m_grabbed_images_th(0)
{
	m_grabbed_files.clear();
	m_grabbed_stereo_files.clear();
} // end-constructor

/** Default Destructor */
CImageGrabberApp::~CImageGrabberApp() 
{
} // end-destructor

/** OnStartup */
bool CImageGrabberApp::OnStartUp()
{
	string camera_cfg_file;
	string camera_cfg_section;
	string m_image_dir;

	// load camera configuration
	if( !m_MissionReader.GetConfigurationParam( "camera_cfg_file", camera_cfg_file ) )
	{
		VERBOSE_LEVEL(0) << "[pImageGrabber -- ERROR]	Camera config " << camera_cfg_file << " file not found!" << endl;
		return false;
	}

	if( !m_MissionReader.GetConfigurationParam( "camera_cfg_section", camera_cfg_section ) )
	{
		VERBOSE_LEVEL(0) << "[pImageGrabber -- ERROR]	Camera config section not found!" << endl;
		return false;
	}

	// set/unset old files deletion
	m_MissionReader.GetConfigurationParam( "delete_old_files", m_delete_old_files );

	// sets the level of verbosity for this module: [0] Only errors, [1] Important info, [2] Everything!
	m_MissionReader.GetConfigurationParam( "verbose_level", m_verbose_level );

	if( m_delete_old_files )
	{
		// get maximum number of images recorded in disk (only applicable if 'm_delete_old_files' == true)
		m_MissionReader.GetConfigurationParam( "grabbed_images_th", m_grabbed_images_th );
	} // end-if

	// load camera configuration
	m_camera.loadConfig( CConfigFile(camera_cfg_file), camera_cfg_section );

	if( m_MissionReader.GetConfigurationParam( "image_dir", m_image_dir ) )
	{
		mrpt::utils::CImage::IMAGES_PATH_BASE = m_image_dir;
		m_camera.setPathForExternalImages(m_image_dir);
		VERBOSE_LEVEL(1) << "[pImageGrabber -- INFO]	Images will be recorded in folder " << endl << m_image_dir << endl << endl;
	}
	else
	{
		m_camera.setPathForExternalImages("./images");
		VERBOSE_LEVEL(1) << "[pImageGrabber -- INFO]	Images will be recorded in folder ./images" << endl;
	}

	//!  @moos_publish GRABBED_IMAGE_DIR
	m_Comms.Notify( "GRABBED_IMAGE_DIR", m_image_dir );

	// initialize camera
	m_camera.initialize();

	// autostart?
	m_MissionReader.GetConfigurationParam( "autostart", m_start_grabbing );
	if( m_start_grabbing )
		VERBOSE_LEVEL(1) << "[pImageGrabber -- INFO] Starting to grab ..." << endl;
	else 
		VERBOSE_LEVEL(1) << "[pImageGrabber -- INFO] Use 'IMAGE_GRABBER_CMD START' to start grabbing ..." << endl;

	m_initialized_ok = true;

	return true;
} // end-OnStartUp

/** DoRegistrations */
bool CImageGrabberApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe IMG_GRABBER_CMD
	AddMOOSVariable( "IMG_GRABBER_CMD", "IMG_GRABBER_CMD", "IMG_GRABBER_CMD", 0 );

	RegisterMOOSVariables();
	return true;
} // end-DoRegistrations

/** OnNewMail */
bool CImageGrabberApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i = NewMail.begin(); i != NewMail.end(); ++i)
	{
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
		if( (i->GetName()=="IMG_GRABBER_CMD") )	// process command to the grabber, typically: start or stop to grab
		{
			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Received Command: " << i->GetString().c_str() << endl;

			std::deque<std::string> lista;
			mrpt::system::tokenize( i->GetString(), " ", lista );

			cad = MOOSToLower( lista[0].c_str() );
			if( MOOSStrCmp(cad,"start") )
			{
				if( !m_start_grabbing )
				{
					m_grabbed_images_counter = 0;	// reset counter of grabbed images
					m_start_grabbing = true;
					VERBOSE_LEVEL(1) << "[pImageGrabber -- INFO] Image recording started." << endl;
				} // end-start-grabbing
			}
			else if( MOOSStrCmp(cad,"stop") )
			{
				if( m_start_grabbing )
				{
					m_start_grabbing = false;
					VERBOSE_LEVEL(1) << "[pImageGrabber -- INFO] Image recording stopped." << endl;
				} // end-stop-grabbing
			}
			else VERBOSE_LEVEL(0) << "[pImageGrabber -- INFO] Command not recognized: " << cad << endl;
		}
	}
    UpdateMOOSVariables(NewMail);
    return true;
} // end-OnNewMail

/** Iterate */
bool CImageGrabberApp::Iterate()
{
	if( !m_initialized_ok || !m_start_grabbing )
		return true;

	m_counter++;

	// grab image from video input
	CObservationPtr obs;
	obs = m_camera.getNextFrame();

	if(obs)
	{
		VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Object grabbed." << endl;

		// serialize observation and send message
		mrpt::vector_byte bObs;
		mrpt::utils::ObjectToOctetVector(obs.pointer(), bObs);

		//!  @moos_publish DETECTED_FACES
		m_Comms.Notify( "GRABBED_IMAGE", bObs );

		// update counter
		++m_grabbed_images_counter;

		// check if we are recording a mono or a stereo observation
		if( IS_CLASS(obs,CObservationImage) )
		{
			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Single image observation." << endl;

			CObservationImagePtr imgptr = static_cast<CObservationImagePtr>(obs);
		
			std::string filename = imgptr->image.getExternalStorageFileAbsolutePath();
			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Image file name: "  << filename << endl;
			m_grabbed_files.push_back(filename);

			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Files in list: " << m_grabbed_files.size() << endl;
			
			// save file path for later deletion if desired (clear buffer after N images)
			if( m_delete_old_files && (m_grabbed_images_th > 0) && (m_grabbed_files.size() > m_grabbed_images_th) )
			{
				mrpt::system::deleteFile( m_grabbed_files.front() );
				m_grabbed_files.pop_front();
			} // end-if

		} // end-if-mono 
		else if( IS_CLASS(obs,CObservationStereoImages) )
		{
			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Stereo image observation." << endl;

			CObservationStereoImagesPtr imgptr = static_cast<CObservationStereoImagesPtr>(obs);
			std::string filename1 = imgptr->imageLeft.getExternalStorageFileAbsolutePath();
			std::string filename2 = imgptr->imageRight.getExternalStorageFileAbsolutePath();

			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Image file names: "  << endl << filename1 << endl << filename2 << endl;
			m_grabbed_stereo_files.push_back( make_pair(filename1,filename2));

			VERBOSE_LEVEL(2) << "[pImageGrabber -- INFO] Stereo files in list: " << m_grabbed_stereo_files.size() << endl;

			// save file path for later deletion if desired (clear buffer after N images)
			if( m_delete_old_files && (m_grabbed_images_th > 0) && (m_grabbed_stereo_files.size() > m_grabbed_images_th) )
			{
				mrpt::system::deleteFile( m_grabbed_stereo_files.front().first );
				mrpt::system::deleteFile( m_grabbed_stereo_files.front().second );
				m_grabbed_stereo_files.pop_front();
			} // end-if

		} // end-if-stereo


		return true;
	}
	return false;
} // end-Iterate

bool CImageGrabberApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

bool CImageGrabberApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}