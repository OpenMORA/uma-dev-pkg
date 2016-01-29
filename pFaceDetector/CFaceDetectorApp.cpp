/**  @moos_module Module to detect faces in a stream of images grabbed from a webcam
  *  This module gets images from a webcam or an IP cam (or video stream) and looks for faces in the images
  */

#include "CFaceDetectorApp.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "mrpt/utils/CTimeLogger.h"
#include "mrpt/utils/CConfigFileMemory.h"
#include "mrpt/obs/CObservationImage.h"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace mrpt::utils;
using namespace mrpt::obs;

#define VERBOSE_LEVEL(_LEVEL) if(m_verbose_level>=_LEVEL) cout

/** Default Constructor */
CFaceDetectorApp::CFaceDetectorApp() : 
	m_initialized_ok(false), 
	m_start_detecting(false), 
	m_verbose_level(1),
	m_show_view(true), 
	//m_display_window(NULL),
	m_face_consolidation(0),
	m_face_consolidation_th(10),
	m_tout(0) // 0 = indefinitely
{
	m_image = mrpt::utils::CImage();	// empty image
} // end-constructor

/** Default Destructor */
CFaceDetectorApp::~CFaceDetectorApp() 
{
	// clear windows
	//m_display_window.clear();

	for(int i=0; i<m_display_windows.size(); ++i)
		if(m_display_windows[i])
			m_display_windows[i].clear();
} // end-destructor

/** OnStartup */
bool CFaceDetectorApp::OnStartUp()
{
	// load params from mission file
	// show view flag
	m_MissionReader.GetConfigurationParam( "show_view", m_show_view );

	//if( m_show_view )
	//{
	//	m_display_window = mrpt::gui::CDisplayWindow::Create("Detected face");
	//}
	
	// threshold for consolidating the face detection
	m_MissionReader.GetConfigurationParam( "face_consolidation_th", m_face_consolidation_th );

	// sets the level of verbosity for this module: [0] Only errors, [1] Important info, [2] Everything!
	m_MissionReader.GetConfigurationParam( "verbose_level", m_verbose_level );

	// sets the detector timeout (in seconds)
	m_MissionReader.GetConfigurationParam( "timeout", m_tout );

	// cascade classifier input data
	string face_cascade_name;
	if( !m_MissionReader.GetConfigurationParam( "classifier_model", face_cascade_name ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR] Classifier model not found in mission file!" << endl;
		return false;
	}

	VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Loading classifier ... " << endl;
	if( !m_clasifier.load( face_cascade_name ) ){
		VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR] Error loading classifier!" << endl; 
		return false; 
	}
	cout << "OK" << endl;

#if 0

	//const string camera_config_str = 
	//	"[CONFIG]\n"
	//	"grabber_type = ffmpeg\n"
	//	"ffmpeg_url="+video_url+"\n";
	
	const string camera_config_str = 
		"[CONFIG]\n"
		"grabber_type = opencv\n";

	mrpt::utils::CConfigFileMemory cfg(camera_config_str);
	m_camera.loadConfig(cfg,"CONFIG");
	m_camera.initialize();
#endif

	m_initialized_ok = true;
	return true;
} // end-OnStartUp

/** DoRegistrations */
bool CFaceDetectorApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe FACE_DETECT_CMD
	// FACE_DETECT_CMD {START,STOP}
	// FACE_DETECT_CMD {SET_TIMEOUT} TIMEOUT_IN_SECONDS
	AddMOOSVariable( "FACE_DETECT_CMD", "FACE_DETECT_CMD", "FACE_DETECT_CMD", 0 );

	//! @moos_subscribe GRABBED_IMAGE
	AddMOOSVariable( "GRABBED_IMAGE", "GRABBED_IMAGE", "GRABBED_IMAGE", 0 );

	//! @moos_subscribe GRABBED_IMAGE_DIR
	AddMOOSVariable( "GRABBED_IMAGE_DIR", "GRABBED_IMAGE_DIR", "GRABBED_IMAGE_DIR", 0 );

	RegisterMOOSVariables();
	return true;
} // end-DoRegistrations

/** OnNewMail */
bool CFaceDetectorApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		if( (i->GetName() == "SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}

		if( i->GetName() == "FACE_DETECT_CMD" )
		{
			VERBOSE_LEVEL(2) << "[pFaceDetector -- INFO] Received Command: " << i->GetString().c_str() << endl;
			
			std::deque<std::string> lista;
			mrpt::system::tokenize( i->GetString(), " ", lista );

			// Parse command
			cad = MOOSToLower( lista[0].c_str() );

			// START DETECTING
			if( MOOSStrCmp(cad,"start") )
			{
				m_ini_time = mrpt::system::now();	// set initial detecting time
				m_face_consolidation = 0;			// restart consolidation counter
				m_start_detecting = true;
				VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Starting to detect ..." << endl;
			}
			// STOP DETECTING
			else if( MOOSStrCmp(cad,"stop") )
			{
				m_start_detecting = false;
				VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Stopped" << endl;
			}
			// SETTING TIMEOUT
			else if( MOOSStrCmp(cad,"set_timeout") )
			{
				// timeout is defined
				if( lista.size() > 1 )
				{
					double tout = atof( lista[1].c_str() );
					if( tout >= 0 )			// to avoid negative timeouts
					{						
						m_tout = tout;
						VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Setting timeout to " << m_tout << endl;
					}
					else VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR] Timeout cannot be negative!: " << tout << endl;
				} // end-if
			}
			else VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR] Command not recognized: " << cad << endl;
			
		} // end

		if( i->GetName() == "GRABBED_IMAGE_DIR" )
		{
			mrpt::utils::CImage::IMAGES_PATH_BASE = i->GetString();
			VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Directory from where to take grabbed images: " << endl << mrpt::utils::CImage::IMAGES_PATH_BASE << endl;
		} // end-if
		
	} // end-for
    UpdateMOOSVariables(NewMail);
    return true;
} // end-OnNewMail

bool CFaceDetectorApp::getImage()
{
	CMOOSVariable * pVar = GetMOOSVar("GRABBED_IMAGE");
	if (pVar && pVar->IsFresh())
	{
		VERBOSE_LEVEL(2) << "[pFaceDetector -- INFO] 'GRABBED_IMAGE' found, deserialize observation." << endl;
		pVar->SetFresh(false);

		mrpt::utils::CSerializablePtr obj;
		mrpt::utils::RawStringToObject(pVar->GetStringVal(), obj);

		using mrpt::obs::CObservationImage;
		if( obj )
		{
			if( IS_CLASS(obj,CObservationImage) )
			{
				VERBOSE_LEVEL(2) << "[pFaceDetector -- INFO] Image found, trying to detect face ... " << endl;
				mrpt::obs::CObservationImagePtr obs = mrpt::obs::CObservationImagePtr(obj);
				const string sFilSrc = obs->image.getExternalStorageFileAbsolutePath();
				ASSERT_( mrpt::system::fileExists( sFilSrc ) )
				VERBOSE_LEVEL(2) << "[pFaceDetector -- INFO] File from: " << sFilSrc << endl;
				
				// copy image to internal variable
				m_image.loadFromFile(sFilSrc);
			
				return true;
			}
			else if( IS_CLASS(obj,CObservationStereoImages) )
			{
				VERBOSE_LEVEL(2) << "[pFaceDetector -- INFO] Stereo image found, trying to detect face in left image ... " << endl;
				mrpt::obs::CObservationStereoImagesPtr obs = mrpt::obs::CObservationStereoImagesPtr(obj);
				const string sFilSrc = obs->imageLeft.getExternalStorageFileAbsolutePath();
				ASSERT_( mrpt::system::fileExists( sFilSrc ) )
				VERBOSE_LEVEL(2) << "[pFaceDetector -- INFO] File from: " << sFilSrc << endl;
				
				// copy image to internal variable
				m_image.loadFromFile(sFilSrc);
			
				return true;
			}
			else
			{
				VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR]	Observation in 'GRABBED_IMAGE' is not CObservationImage or CObservationStereoImages" << endl;
				return false;
			}
		} // end-if
		else
		{
			VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR]	Variable 'GRABBED_IMAGE' does not contain a valid CObservationImage" << endl;
			return false;
		}
	}
	else
	{
		VERBOSE_LEVEL(0) << "[pFaceDetector -- ERROR] Variable 'GRABBED_IMAGE' does not exists" << endl;
		return false;
	} // end-GRABBED_IMAGE
} // end-getImage

bool CFaceDetectorApp::detectFaces()
{
	// detect faces
	Mat frame = cvarrToMat( m_image.getAs<IplImage>() );
	
	std::vector<Rect> faces;
	Mat frame_gray;

	if( frame.channels() > 1 ) cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
	else frame_gray = frame;

	// perform face detection
	m_tictac.Tic();
	m_clasifier.detectMultiScale( frame_gray, faces, 1.2, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
	double time = m_tictac.Tac();

	// publish messages if necessary
	const int nFaces = faces.size();
	// std::cout << "[CFaceDetectorApp -- INFO]	Detected faces: " << nFaces << " in " << time << " seconds" << endl;
	string message = mrpt::format("%d",nFaces);
	if( nFaces > 0 )
	{
		if( ++m_face_consolidation <= m_face_consolidation_th )
		{
			VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Face detected, consolidating (" << m_face_consolidation << ")" << endl;
		}
		else
		{
			VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Face detected (consolidation complete)!" << endl;
			
			// timeout management
			m_ini_time = mrpt::system::now();
			
			// display all faces
			m_display_windows.resize(nFaces);

			// FORMAT: N_FACES FACE_1_X FACE_1_Y FACE_1_WIDTH FACE_1_HEIGHT ... FACE_N_X FACE_N_Y FACE_N_WIDTH FACE_N_HEIGHT
			for( int i = 0; i < nFaces; ++i )
			{
				// std::cout << "[" << i << "]:" << faces[i].x << "," << faces[i].y << "," << faces[i].width << "," << faces[i].height << endl;
				message += mrpt::format(",%d,%d,%d,%d", faces[i].x, faces[i].y, faces[i].width, faces[i].height );

				// Serialize:
				const double Dw = 1.15;
				const double Dh = 1.35;
				const double Kx = 0.5;
				const double Ky = 0.55;

				Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
				const int nx = center.x-int(Kx*Dw*faces[i].width);
				const int ny = center.y-int(Ky*Dh*faces[i].height);

				int nw, nh;
				if( nx < 0 )									nw = Dw*faces[i].width+nx;
				else if( nx+Dw*faces[i].width > frame.cols-1 )	nw = frame.cols-center.x;
				else											nw = Dw*faces[i].width; // all the patch within the image
				
				if( ny < 0 )									nh = Dh*faces[i].height+ny;
				else if( ny+Dh*faces[i].height > frame.rows-1 )	nh = frame.rows-center.y;
				else											nh = Dh*faces[i].width; // all the patch within the image

				//cout << nx << "," << ny << "," << nw << "," << nh << endl;

				//// create a small image and send it
				//int h	= int( faces[i].height*1.22 );
				//int h_2 = int( h-faces[i].height );

				//
				//const int nw_2 = int(0.5*1.1*faces[i].width);
				//const int nh_2 = int(0.5*1.3*faces[i].height);
				//const int nx = int(std::max(0,center.x-nw_2));
				//const int ny = int(std::max(0,center.y-nh_2));

				//const int nw = (center.x+nw_2) > (frame.cols-1) ? frame.cols-center.x : 2*nw_2;
				//const int nh = (center.y+nh_2) > (frame.rows-1) ? frame.rows-center.y : 2*nh_2;

				CImage face_img;
				m_image.extract_patch( face_img, nx, ny, nw, nh );

				//m_image.extract_patch( face_img, 
				//	faces[i].x, 
				//	std::max(0,faces[i].y-h_2), 
				//	faces[i].width, 
				//	h );

				CObservationImagePtr new_img_obs = CObservationImage::Create();

				// look for the face label
				double min_d = std::numeric_limits<double>::max();
				int min_idx = 0;
				for( int k = 0; k < m_faces.size(); ++k )
				{
					const cv::Point & p = m_faces[k].p;
					const double d = sqrt(
						(m_faces[k].p.x-faces[i].x)*(m_faces[k].p.x-faces[i].x)+
						(m_faces[k].p.y-faces[i].y)*(m_faces[k].p.y-faces[i].y));

					if( d < min_d /*px*/ )
					{
						min_d = d;
						min_idx = k;
					} // end-if
				} // end-for

				// keep face tracking
				if( min_d < 50 /*px*/ )
				{
					new_img_obs->sensorLabel = format("%d",m_faces[min_idx].id);
					m_faces[min_idx].p = cv::Point(faces[i].x,faces[i].y);
					// m_faces_up[min_idx] = true;
				}
				else
				{
					int new_id = 0;
					vector<TTrackingFaceInfo>::iterator it = m_faces.begin();
					while( it != m_faces.end() )	// increase times not seen
					{
						if( new_id == it->id ) new_id++;
						if( ++it->n_lost > 10 ) it = m_faces.erase(it); // TO DO : variable threshold
						else					++it;
					}
					new_img_obs->sensorLabel = format("%d",new_id);	// new id
					m_faces.push_back(TTrackingFaceInfo(new_id,cv::Point(faces[i].x,faces[i].y)));
					// m_faces_up.push_back(true);
				}
				
				new_img_obs->image.copyFastFrom(face_img);
				mrpt::vector_byte bObs;
				mrpt::utils::ObjectToOctetVector(new_img_obs.pointer(), bObs);

				if( m_show_view )
				{
					if( !m_display_windows[i] )	// create if it doesn't exists
						m_display_windows[i] = mrpt::gui::CDisplayWindow::Create();
					m_display_windows[i]->setWindowTitle(new_img_obs->sensorLabel);
					m_display_windows[i]->showImage(new_img_obs->image);

					//m_display_window->setWindowTitle(new_img_obs->sensorLabel);
					//m_display_window->showImage(new_img_obs->image);
				}
				// std::cout << "[CFaceDetectorApp -- INFO]	Publishing image: " << endl << message << endl;
				
				//!  @moos_publish FACES_DETECT_IMGS
				m_Comms.Notify( "FACES_DETECT_IMGS", bObs );
				// std::cout << " ... OK" << endl;
			} // end-for

			/** /
			// remove those not updated
			vector<cv::Point>::iterator it;
			vector<bool>::iterator it_up;
			for( it = m_faces.begin(), it_up = m_faces_up.begin(); it != m_faces.end(); )
			{
				if( *it_up )
				{
					it = m_faces.erase(it);
					it_up = m_faces_up.erase(it_up);
				}
				else
				{
					it++;
					it_up++;
				}
			}
			/**/
			//std::cout << "[CFaceDetectorApp -- INFO]	Publishing message: " << endl << message << endl;
			
			//!  @moos_publish FACE_DETECT_FACES
			m_Comms.Notify( "FACE_DETECT_FACES", message );
			m_Comms.Notify( "FACE_DETECT_RESULT", "OK" );
			//std::cout << " ... OK" << endl;

			return true;
		} // 
	} // end-if
	else
	{
		m_face_consolidation = 0;
		m_faces.clear();
		m_Comms.Notify( "FACE_DETECT_RESULT", "NONE" );
	}

	return false;
} // end-detectFaces

/** Iterate */
bool CFaceDetectorApp::Iterate()
{
	if( !m_initialized_ok || !m_start_detecting )
		return true;

	// get image
	if( getImage() )
	{
		if( !detectFaces() )
		{
			if( m_tout > 0 ) // if m_tout == 0 -> no timeout has been set, so try to detect indefinitely
			{
				const double elapsed_time = mrpt::system::timeDifference( m_ini_time, mrpt::system::now() );
				VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] No faces found for " << elapsed_time << " seconds (timeout: " << m_tout << " sec.)" << endl;
				if( elapsed_time > m_tout )
				{
					//! @moos_publish FACE_DETECT_RESULT TIMEOUT
					m_Comms.Notify( "FACE_DETECT_RESULT", "TIMEOUT" );
					m_start_detecting = false;
					VERBOSE_LEVEL(1) << "[pFaceDetector -- INFO] Timeout! No face found in " << m_tout << " seconds, stop trying." << endl << "Signal 'FACE_DETECT_RESULT TIMEOUT' has been sent." << endl;
				} // end-if
			} // end-if
		} // end-if
	} // end-if

	return true;
} // end-Iterate

bool CFaceDetectorApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

bool CFaceDetectorApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}