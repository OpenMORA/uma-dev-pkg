/**  @moos_module Module to detect faces in a stream of images grabbed from a webcam
  *  This module gets images from a webcam or an IP cam (or video stream) and looks for faces in the images
  */

#include "CFaceRecognizerApp.h"

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
#define INVALID_FACE_CLASS -1

const double BASELINE_RATIO = 0.380;
inline string bool2string(const bool b) { return b ? "Yes" : "No"; }
inline string model2string(const MODEL_TYPE m) { return m == LBPH ? "LBPH" : m == EigenFaces ? "EigenFaces" : "FiserFaces"; }

/** Default Constructor */
CFaceRecognizerApp::CFaceRecognizerApp() : 
	m_verbose_level(1),
	m_show_view(false),
	m_debug(false),
	m_initialized_ok(false), 
	m_start_recognizing(false), 
	m_add_new_face(false),
	m_img_h(0), m_img_w(0),
	m_ellipse_roi(cv::Mat()),
	m_display_window(NULL),
	m_recognizer(NULL),
	m_recognizer2(NULL),
	m_recognizer3(NULL),
	m_model_type(LBPH),
	m_facerec_consolidation(0),
	m_facerec_consolidation_th(5),
	m_tout(0), // 0 = indefinitely
	m_temptative_class(INVALID_FACE_CLASS),
	m_face_labels_db(vector<string>()),
	m_num_classes(0),
	m_new_face_th(10)
{
	m_image = mrpt::utils::CImage();	// empty image
} // end-constructor

/** Default Destructor */
CFaceRecognizerApp::~CFaceRecognizerApp() 
{
	m_display_window.clear();
} // end-destructor

/** OnStartup */
bool CFaceRecognizerApp::OnStartUp()
{
	m_MissionReader.GetConfigurationParam( "debug", m_debug );

	// sets the detector timeout (in seconds)
	m_MissionReader.GetConfigurationParam( "timeout", m_tout );
	
	// show view flag
	m_MissionReader.GetConfigurationParam( "show_view", m_show_view );

	// create view
	if( m_show_view )
	{
		m_display_window = mrpt::gui::CDisplayWindow::Create("Recognition result");
	}

	// load params from mission file
	// sets the level of verbosity for this module: [0] Only errors, [1] Important info, [2] Everything!
	m_MissionReader.GetConfigurationParam( "verbose_level", m_verbose_level );

	// initialize recognizer
	int model_type;
	if( !m_MissionReader.GetConfigurationParam( "model_type", model_type ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Model type not found in mission file!" << endl;
		return false;
	}

	// initialize eyes detectors
	VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Initializing detectors ... ";
	string left_detector, right_detector;
	if( !m_MissionReader.GetConfigurationParam( "left_eye_detector", left_detector ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Left eye detector not found in mission file!" << endl;
		return false;
	}
	ASSERT_FILE_EXISTS_( left_detector )
	m_left_eye_detector.load( left_detector );
	
	if( !m_MissionReader.GetConfigurationParam( "right_eye_detector", right_detector ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Right eye detector not found in mission file!" << endl;
		return false;
	}
	ASSERT_FILE_EXISTS_( right_detector )
	m_right_eye_detector.load( right_detector );
	VERBOSE_LEVEL(1) << "OK" << endl;

	m_model_type = MODEL_TYPE(model_type);

	if(m_model_type == LBPH) m_recognizer = cv::createLBPHFaceRecognizer();
	else if(m_model_type == EigenFaces) m_recognizer = cv::createEigenFaceRecognizer(80);
	else if(m_model_type == FisherFaces) m_recognizer = cv::createFisherFaceRecognizer();
	else {
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Invalid model type specified in mission file!" << endl;
		return false;
	}

	m_recognizer2 = cv::createEigenFaceRecognizer(80);
	m_recognizer3 = cv::createFisherFaceRecognizer();

	if( !m_MissionReader.GetConfigurationParam( "img_h", m_img_h ) || !m_MissionReader.GetConfigurationParam( "img_w", m_img_w ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Training image size not found in mission file!" << endl;
		return false;
	}

	string recognizer_filename;
	if( !m_MissionReader.GetConfigurationParam( "recognizer_model", recognizer_filename ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Recognizer model not found in mission file!" << endl;
		return false;
	}
	ASSERT_FILE_EXISTS_( recognizer_filename )

	VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Loading recognizer ... ";
	m_recognizer->load( recognizer_filename );
	m_recognizer2->load( "EFaces_mapir.yml" );
	m_recognizer3->load( "FFaces_mapir.yml" );
	VERBOSE_LEVEL(1) << "OK" << endl;

	// fill the face database [int] -> [string] (label)
	VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Loading database ... ";
	if( !m_MissionReader.GetConfigurationParam( "n_faces_db", 	m_num_classes ) )
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Number of faces in db not defined!" << endl;
		return false;
	}

	// read label strings
	m_face_labels_db.resize(m_num_classes);
	for(int i=0;i<m_num_classes;++i)
	{
		string this_label;
		if( !m_MissionReader.GetConfigurationParam( mrpt::format("face_label_%d", i ).c_str(), this_label) )
		{
			VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Faces " << i << " label not defined!" << endl;
			return false;
		}
		m_face_labels_db[i] = this_label;
	} // end-for
	VERBOSE_LEVEL(1) << "OK (" << m_num_classes << " elements in DB)" << endl;

	// fill the face database [int] -> [string] (label)
	VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Loading new faces scheme ... ";
	m_MissionReader.GetConfigurationParam( "new_face_th", m_new_face_th );
	VERBOSE_LEVEL(1) << "OK" << endl;

	// next new face found will have this numeric label
	m_new_label = m_num_classes;

	// initialize class tracker
	m_class_track.initialize(m_num_classes,0.9); // TODO: 0.9 read from ini file

	// show summary
	VERBOSE_LEVEL(1)	<< "= SUMMARY ======================" << endl
						<< "	Verbose level		" << m_verbose_level << endl
						<< "	Show result?		" << bool2string(m_show_view) << endl
						<< "	Recognizer type		" << model2string(m_model_type) << endl
						<< "	Recognizer file		" << recognizer_filename << endl
						<< "	Eye detectors	" << endl
						<< "		L:	" << left_detector << endl
						<< "		R:	" << right_detector << endl
						<< "	Training image size	" << m_img_w << "x" << m_img_h << endl
						<< "	Database elements	" << m_num_classes << endl
						<< "	New face threshold	" << m_new_face_th << endl
						<< "===============================" << endl;

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

	// autostart?
	m_MissionReader.GetConfigurationParam( "autostart", m_start_recognizing );
	if( m_start_recognizing )
	{
		VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Starting to grab ..." << endl;
		m_ini_time = mrpt::system::now();
	}
	else 
		VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Use 'FACE_RECOGNIZE_CMD START' to start recognizing ..." << endl;

	m_initialized_ok = true;
	return true;
} // end-OnStartUp

/** DoRegistrations */
bool CFaceRecognizerApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe FACE_DETECT_CMD
	// FACE_DETECT_CMD {START,STOP}
	// FACE_DETECT_CMD {SET_TIMEOUT} TIMEOUT_IN_SECONDS
	AddMOOSVariable( "FACE_RECOGNIZE_CMD", "FACE_RECOGNIZE_CMD", "FACE_RECOGNIZE_CMD", 0 );

	//! @moos_subscribe FACES_DETECT_IMGS
	AddMOOSVariable( "FACES_DETECT_IMGS", "FACES_DETECT_IMGS", "FACES_DETECT_IMGS", 0 );

	//! @moos_subscribe FACE_DETECT_RESULT
	AddMOOSVariable( "FACE_DETECT_RESULT", "FACE_DETECT_RESULT", "FACE_DETECT_RESULT", 0 );

	RegisterMOOSVariables();
	return true;
} // end-DoRegistrations

/** OnNewMail */
bool CFaceRecognizerApp::OnNewMail(MOOSMSG_LIST &NewMail)
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

		if( i->GetName() == "FACE_RECOGNIZE_CMD" )
		{
			VERBOSE_LEVEL(2) << "[pFaceRecognizer -- INFO] Received Command: " << i->GetString().c_str() << endl;
			
			std::deque<std::string> lista;
			mrpt::system::tokenize( i->GetString(), " ", lista );

			// Parse command
			cad = MOOSToLower( lista[0].c_str() );

			// START DETECTING
			if( MOOSStrCmp(cad,"start") )
			{
				m_start_recognizing = true;
				m_ini_time = mrpt::system::now();		// set initial detecting time
				m_facerec_consolidation = 0;			// restart consolidation counter
				VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Starting to recognize ..." << endl;
			}
			// STOP DETECTING
			else if( MOOSStrCmp(cad,"stop") )
			{
				m_start_recognizing = false;
				VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Stopped" << endl;
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
						VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Setting timeout to " << m_tout << endl;
					}
					else VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Timeout cannot be negative!: " << tout << endl;
				} // end-if
			}
			else VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Command not recognized: " << cad << endl;
			
		} // end

		if( i->GetName() == "FACE_DETECT_RESULT" )
		{
			if( MOOSStrCmp(i->GetAsString(),"NONE") )
			{
				// clear
			}
		} // end-if
		
	} // end-for
    UpdateMOOSVariables(NewMail);
    return true;
} // end-OnNewMail

/** m_copy_image */
bool CFaceRecognizerApp::m_copy_image()
{
	CMOOSVariable * pVar = GetMOOSVar("FACES_DETECT_IMGS");
	if (pVar && pVar->IsFresh())
	{
		VERBOSE_LEVEL(2) << "[pFaceRecognizer -- INFO] 'FACES_DETECT_IMGS' found, deserialize observation." << endl;
		pVar->SetFresh(false);

		mrpt::utils::CSerializablePtr obj;
		mrpt::utils::RawStringToObject(pVar->GetStringVal(), obj);

		// convert to CObservationImage
		using mrpt::obs::CObservationImage;
		if( obj )
		{
			VERBOSE_LEVEL(2) << "[pFaceRecognizer -- INFO] Image found, trying to detect face ... " << endl;
			mrpt::obs::CObservationImagePtr obs = mrpt::obs::CObservationImagePtr(obj);

			// copy image to internal variable
			m_image.copyFastFrom(obs->image);
			VERBOSE_LEVEL(2) << "OK" << endl;

			// copy label from observation (will lead the consolidation scheme)
			m_obs_label = atoi(obs->sensorLabel.c_str());
			
			return true;
		} // end-if
		else
		{
			VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR]	Variable 'FACES_DETECT_IMGS' does not contain a valid CObservationImage" << endl;
			return false;
		}
	}
	else
	{
		VERBOSE_LEVEL(0) << "[pFaceRecognizer -- ERROR] Variable 'FACES_DETECT_IMGS' does not exist" << endl;
		return false;
	} // end-FACES_DETECT_IMGS
} // end-getImage

/** m_get_correct_eye_position */
bool CFaceRecognizerApp::m_get_correct_eye_position( 
	const std::vector<cv::Rect> & eyesL,					
	const std::vector<cv::Rect> & eyesR, 
	const int					w, 
	cv::Point					& left_eye_pos,			// output
	cv::Point					& right_eye_pos,		// output
	cv::Rect					& left_eye,
	cv::Rect					& right_eye ) const		
{
	VERBOSE_LEVEL(2) << " -- [L]: " << eyesL.size() << " and [R]: " << eyesR.size() << " -- ";
	if( (eyesL.size() + eyesR.size())<2 ) return false; // not enough eyes :-P

	int max_x = 0, min_x = w;
	
	// user left eye: closest to the right border
	for( size_t j = 0; j < eyesL.size(); ++j )
	{
		if( eyesL[j].x > max_x )
		{
			max_x = eyesL[j].x;
			left_eye_pos = Point(eyesL[j].x+0.5*eyesL[j].width,eyesL[j].y+0.5*eyesL[j].height);
			left_eye = eyesL[j];
		}
		if( eyesL[j].x < min_x )
		{
			min_x = eyesL[j].x;
			right_eye_pos = Point(eyesL[j].x+0.5*eyesL[j].width,eyesL[j].y+0.5*eyesL[j].height);
			right_eye = eyesL[j];
		} 
	}

	// user right eye: closest to the left border
	for( size_t j = 0; j < eyesR.size(); ++j )
	{
		if( eyesR[j].x > max_x )
		{
			max_x = eyesR[j].x;
			left_eye_pos = Point(eyesR[j].x+0.5*eyesR[j].width,eyesR[j].y+0.5*eyesR[j].height);
			left_eye = eyesR[j];
		}
		if( eyesR[j].x < min_x )
		{
			min_x = eyesR[j].x;
			right_eye_pos = Point(eyesR[j].x+0.5*eyesR[j].width,eyesR[j].y+0.5*eyesR[j].height);
			right_eye = eyesR[j];
		} 
	}

	// return 'true' if we have found one eye at each part of the image
	// return left_eye_pos.x >= w/2 && right_eye_pos.x <= w/2;
	
	// return 'true' if the baseline is long enough
	const int baseline_2 =  sqrt((left_eye_pos.x-right_eye_pos.x)*(left_eye_pos.x-right_eye_pos.x) +
								 (left_eye_pos.y-right_eye_pos.y)*(left_eye_pos.y-right_eye_pos.y));
	return baseline_2 > (0.11*m_img_w); // 11% of image width
}

/** processImage */
void CFaceRecognizerApp::m_transform_image(
	const cv::Mat	& input, 
	const cv::Point & left_eye, 
	const cv::Point & right_eye,
	cv::Mat			& output ) const
{
	// 1) Rotate and scale 
	// baseline direction and distance
	int ed_x = left_eye.x-right_eye.x, ed_y = left_eye.y-right_eye.y;
	double angle_rad = atan2f(ed_y,ed_x);
	double angle = 180.0/CV_PI*angle_rad;				// deg
	double baseline = sqrt(ed_x*ed_x+ed_y*ed_y);		// current baseline in pixels
	double d_baseline = input.cols*BASELINE_RATIO;		// desired baseline
	
	// rotate, translate and scale!
	double scale = d_baseline/baseline;					// get the scale according to baseline
	
	// rotation and scale
	//cv::Mat rot_mat = cv::getRotationMatrix2D(center,angle,scale);
	cv::Mat rot_mat = cv::getRotationMatrix2D(right_eye,angle,1.0/*scale*/);
	cv::Rect bbox = cv::RotatedRect(right_eye/*center*/,input.size(),angle).boundingRect();
    rot_mat.at<double>(0,2) += bbox.width/2.0 - right_eye.x;
    rot_mat.at<double>(1,2) += bbox.height/2.0 - right_eye.y;	

	// apply transform
	cv::Mat temp;
	cv::warpAffine(input,temp,rot_mat,input.size());
	cv::warpAffine(input,temp,rot_mat,bbox.size(),INTER_CUBIC);

	// 2) Translate to final position
	// Transform eye position:
	Point2d new_right_eye, new_left_eye;
	new_right_eye.x = right_eye.x*rot_mat.at<double>(0,0)+right_eye.y*rot_mat.at<double>(0,1)+rot_mat.at<double>(0,2);
	new_right_eye.y = right_eye.x*rot_mat.at<double>(1,0)+right_eye.y*rot_mat.at<double>(1,1)+rot_mat.at<double>(1,2);
	
	new_left_eye.x = left_eye.x*rot_mat.at<double>(0,0)+left_eye.y*rot_mat.at<double>(0,1)+rot_mat.at<double>(0,2);
	new_left_eye.y = left_eye.x*rot_mat.at<double>(1,0)+left_eye.y*rot_mat.at<double>(1,1)+rot_mat.at<double>(1,2);

	Point mid_point( (new_right_eye.x+new_left_eye.x)/2,(new_right_eye.y+new_left_eye.y)/2 ); // mid-point between eyes

	// take the mid-point to the position (0.50*output.cols,0.40*output.rows) of the output image
	double tr_x = mid_point.x-0.50*input.cols;
	double tr_y = mid_point.y-0.50*input.rows;
	
	Mat trans_mat = Mat::zeros(2,3,CV_64F);
	trans_mat.at<double>(0,0) = trans_mat.at<double>(1,1) = 1;
	trans_mat.at<double>(0,2) = -tr_x;
	trans_mat.at<double>(1,2) = -tr_y;

	cv::warpAffine(temp,output,trans_mat,input.size(),INTER_CUBIC);

} // end-m_transform_image

/** m_preprocess_imageAndDetect */
bool CFaceRecognizerApp::m_preprocess_image( cv::Mat & out_image )
{
	// detect faces
	VERBOSE_LEVEL(2) << "	From Ipl to MAT";
	Mat frame = cv::cvarrToMat( m_image.getAs<IplImage>() );
	VERBOSE_LEVEL(2) << " ... OK" << endl;

	// DEBUG
	if( m_debug )
	{
		cv::namedWindow( "Input image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Input image", frame );
		cv::waitKey(1);
	}

	// normalize image and convert to grayscale
	VERBOSE_LEVEL(2) << "	Convert to gray";
	Mat frame_gray;
	if( frame.channels() > 1 ) cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
	else frame_gray = frame;
	VERBOSE_LEVEL(2) << " ... OK" << endl;

	if (m_debug)
	{
		cv::namedWindow( "Gray image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Gray image", frame_gray );
		cv::waitKey(1); 
	}
	
	// resize image to that from training set
	Mat frame_gray_res;	
	VERBOSE_LEVEL(2) << "	Resize query image";
	cv::resize(frame_gray, frame_gray_res, cv::Size(m_img_w,m_img_h));
	VERBOSE_LEVEL(2) << "... OK" << endl;

	if (m_debug)
	{
		cv::namedWindow( "Resized image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Resized image", frame_gray_res );
		cv::waitKey(1);
	}

	// Image process in 3 steps:
	// 1) Equalize histogram
	VERBOSE_LEVEL(2) << "	Equalize hist";
	Mat frame_gray_res_eq;
	cv::equalizeHist( frame_gray_res, frame_gray_res_eq );
	VERBOSE_LEVEL(2) << "... OK" << endl;
		
	if (m_debug)
	{
		cv::namedWindow( "Eq image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Eq image", frame_gray_res_eq );
		cv::waitKey(1);
	}

	// 2) Find eyes position
	int h = frame_gray_res_eq.rows, w = frame_gray_res_eq.cols;
	const double offset_y_0 = 0.25*h;
	const double offset_y_1 = 0.45*h;

	// Create image ROI for searching eyes ...
	cv::Rect eyes_roi(0,offset_y_0,w,offset_y_1);
	Mat faceROI = frame_gray_res_eq(eyes_roi);
	if (m_debug)
	{
		imshow("roi eyes",faceROI);
		waitKey(1);
	}

	// ... and ellipse face ROI
	Mat roi(frame_gray_res_eq.size(),frame_gray_res_eq.type(),Scalar(0,0,0)); // black ROI
	cv::ellipse(roi,Point(roi.cols/2,roi.rows/2),Size(roi.cols/2,roi.rows/2),0,0,360,Scalar(255,255,255),-1);
		
	// search for eyes
	VERBOSE_LEVEL(2) << "	Detect eyes";
	vector<Rect> eyesL,eyesR;
	m_left_eye_detector.detectMultiScale(faceROI, eyesL);
	m_right_eye_detector.detectMultiScale(faceROI, eyesR);
	VERBOSE_LEVEL(2) << "... OK" << endl;

	// 3) Affine transformation to match training set eye position
	VERBOSE_LEVEL(2) << "	Correct eye position";
	cv::Point left_eye_pos, right_eye_pos;
	cv::Mat frame_gray_res_eq_proc;
	
	cv::Rect left_eye, right_eye;

	// check 'eyesL' and 'eyesR' for unreliable results (multiple and/or wrongly located)
	const bool correct_ok = m_get_correct_eye_position( 
		eyesL, 
		eyesR, 
		w, 
		left_eye_pos, 
		right_eye_pos, 
		left_eye, 
		right_eye );

	if( correct_ok )
	{
		// restore actual 'y' coordinate due to the ROI offset
		left_eye_pos.y += offset_y_0;
		right_eye_pos.y += offset_y_0;

		left_eye.y += offset_y_0;
		right_eye.y += offset_y_0;

		// DEBUG: show
		if (m_debug)
		{
			cv::Mat aux;
			frame_gray_res_eq.copyTo(aux);

			cv::rectangle(aux,left_eye,Scalar(0,255,0),2);
			cv::rectangle(aux,right_eye,Scalar(0,255,0),2);

			cv::imshow("eyes found",aux);
			cv::waitKey(1);
		}

		VERBOSE_LEVEL(2) << " -- DONE" << endl;
		m_transform_image( 
			frame_gray_res_eq,				// input image
			left_eye_pos,					// left image position
			right_eye_pos,					// right image position
			frame_gray_res_eq_proc);		// output transformed image
	}
	else
	{
		VERBOSE_LEVEL(2) << " --  NOT ACHIEVED" << endl;
		frame_gray_res_eq_proc = frame_gray_res_eq;
	}
	VERBOSE_LEVEL(2) << "... OK" << endl;

	if (m_debug)
	{
		cv::namedWindow( "Rotated image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Rotated image", frame_gray_res_eq_proc );
		cv::waitKey(1);
	}

	// 4) Apply ellipsoidal ROI to restrict face area
	VERBOSE_LEVEL(2) << "	Apply ellipsoidal ROI: ";
	cv::bitwise_and( frame_gray_res_eq_proc, roi, out_image );
	VERBOSE_LEVEL(2) << "... OK" << endl;

	if (m_debug)
	{
		cv::namedWindow( "ROI image", WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "ROI image", out_image );
		cv::waitKey(); 
	}

	return correct_ok;
}

/** learnNewFace (only with LBPH) */
bool CFaceRecognizerApp::m_add_face()
{
	// preprocess image
	cv::Mat frame_pr;
	m_preprocess_image(frame_pr);

	// add
	m_new_face_images.push_back( frame_pr );
	m_new_face_labels.push_back( m_new_label );

	// update model
	if( m_new_face_images.size() >= m_new_face_th )
	{
		m_recognizer->update(m_new_face_images,m_new_face_labels);
		
		// reset		
		m_new_label++;
		m_new_face_images.clear();
		m_new_face_labels.clear();

		m_add_new_face = false;
		m_start_recognizing = true;
	} // end-if
	return true;
}

/** m_recognize_face */
bool CFaceRecognizerApp::m_recognize_face()
{
	// Pre-process current image:
	// 1. RGB -> grayscale
	// 2. Resize
	// 3. Equalize histogram
	// 4. Look for eye positions
	// 5. Rotate, scale and translate image according to eye positions
	// 6. Apply ellipsoidal ROI
	cv::Mat frame_gray_res_eq_roi;
	const bool correct = m_preprocess_image(frame_gray_res_eq_roi);

	// Query image to the recognizer
	VERBOSE_LEVEL(2) << "Query the db";
	int queryLabel,qL2,qL3;
	double conf,conf2,conf3;
	cv::Mat distances,d2,d3;
	m_recognizer->predict( frame_gray_res_eq_roi, queryLabel, conf, distances ); // distances: Mx2 with (col0) the label of each image in DB and (col1) its distance to the query
	m_recognizer2->predict( frame_gray_res_eq_roi, qL2, conf2, d2 ); // distances: Mx2 with (col0) the label of each image in DB and (col1) its distance to the query
	m_recognizer3->predict( frame_gray_res_eq_roi, qL3, conf3, d3 ); // distances: Mx2 with (col0) the label of each image in DB and (col1) its distance to the query
	VERBOSE_LEVEL(1) << "	-- [Naive] MOST PROBABLE CLASS: " << m_face_labels_db[queryLabel] << " [" << queryLabel << "] -- (" << conf << ")" << endl;

	FILE *f = mrpt::system::os::fopen("product.txt","at");
	cout << endl << " ====================== " << conf*conf2*conf3 << " ====================== " << endl << endl;
	mrpt::system::os::fprintf(f,"%.0f\n",conf*conf2*conf3);
	mrpt::system::os::fclose(f);

	//if( queryLabel != qL2 || queryLabel != qL3 || qL3 != qL2 )
	//{
	//	cout << " ****************** UNKNOWN DIFF ************************" << endl;
	//	cout << queryLabel << " -> " << conf << endl;
	//	cout << qL2 << " -> " << conf2 << endl;
	//	cout << qL3 << " -> " << conf3 << endl;
	//	cout << " ********************************************************" << endl;
	//}

	// Get most likely class
	double bestProb = 0.0;
	size_t bestClass = 0;
	m_class_track.getMostLikelyClass( distances, bestClass, bestProb );

	VERBOSE_LEVEL(1) << "	-- [SBF] MOST PROBABLE CLASS: " << m_face_labels_db[bestClass] << " [" << bestClass << "] -- ## -- PROB: " << bestProb << endl;

	if( m_show_view )
	{
		CImage plotImage = m_image;
		plotImage.colorImageInPlace();
		plotImage.textOut(0,0,m_face_labels_db[bestClass],TColor::red);
		m_display_window->showImage(plotImage);
	}

	// consolidation stage (at least 5 consecutive times)
	if( m_temptative_class == INVALID_FACE_CLASS || bestClass == m_temptative_class )
	{
		if( ++m_facerec_consolidation < m_facerec_consolidation_th )
		{
			VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Face recognized, consolidating (" << m_facerec_consolidation << ")" << endl;
		}
		else
		{
			m_ini_time = mrpt::system::now();
			VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Face recognized (consolidation complete)!: " << m_face_labels_db[bestClass] << endl;
			m_Comms.Notify( "FACE_RECOGNIZE_RESULT", m_face_labels_db[bestClass] );
			VERBOSE_LEVEL(2) << " ... OK (" << m_face_labels_db[bestClass] << ")" << endl;
			return true;
		}
	}
	else
		m_facerec_consolidation = 0;

#if 0
	// debug
	/** /
	FILE *f = mrpt::system::os::fopen("dist.txt","wt");
	for( int i = 0; i < distances.size().height; ++i)
		mrpt::system::os::fprintf(f,"%d,%.3f\n",int(distances.at<double>(i,0)),distances.at<double>(i,1));
	mrpt::system::os::fclose(f);
	f = mrpt::system::os::fopen("prob.txt","wt");
	for( std::map<int,double>::iterator it = classProb.begin(); it!=classProb.end(); ++it)
		mrpt::system::os::fprintf(f,"%d,%.3f\n",it->first,it->second);
	mrpt::system::os::fclose(f);
	/**/
	// end-debug

	VERBOSE_LEVEL(2) << " -- RESULT: ## " << queryLabel << " ## -- conf: " << conf << endl;

	// select the votation information for this person
	TVotationInfo *vot;
	std::map<int,TVotationInfo>::iterator it = m_votes.find(m_obs_label);
	if( it != m_votes.end() )
	{
		// cout << "Found votation info " << m_obs_label << endl;
		vot = &(it->second);
	}
	else
	{
		// cout << "Create new votation info " << m_obs_label << endl;
		// create a new one
		m_votes[m_obs_label] = TVotationInfo( this->m_number_of_faces(), queryLabel );
		vot = &(m_votes[m_obs_label]);
	}

	// consolidation stage (either known or unknown face)
	// check if the face has changed
	if( queryLabel == vot->m_last_label )	
	{
		vot->m_cons_votes++;
		// cout << "	increasing cons votes: " << vot->m_cons_votes << endl;
	}
	else 							
	{
		vot->m_last_label = queryLabel;
		vot->m_cons_votes = 1;
		// cout << "	restarting cons votes count: " << vot->m_cons_votes << endl;
	} // end-else

	if( vot->m_cons_votes >= 3 )	// clear the votation result since the face has changed
	{
		// face has been consolidated
		if( vot->m_last_label == -1 /*unknown*/ )
		{
			m_add_new_face = true;	// start to learn a new face (using the following 'm_add_face_th' images)
		}
		else
		{
			vot->m_face_votation[queryLabel]++;
			const double av_sofar = vot->m_face_confidence[queryLabel];
			vot->m_face_confidence[queryLabel] = av_sofar+(conf-av_sofar)/vot->m_face_votation[queryLabel];

			int aux_v = vot->m_face_votation[queryLabel];
			double aux_c = vot->m_face_confidence[queryLabel];

			// clear vectors
			std::fill(vot->m_face_votation.begin(), vot->m_face_votation.end(), 0);
			std::fill(vot->m_face_confidence.begin(), vot->m_face_confidence.end(), 0.0);

			// restore current information
			vot->m_face_votation[queryLabel] = aux_v;
			vot->m_face_confidence[queryLabel] = aux_c;
		}

		// show votation info
		if( m_verbose_level >= 1 )
			vot->dumpToConsole( m_face_labels_db /* for model names */);

		vector<int>::iterator it = std::max_element(vot->m_face_votation.begin(),vot->m_face_votation.end());
		int idx = std::distance(vot->m_face_votation.begin(),it);

		// Manage results for robustness and response agility (change of person)
		// map label -> votes
		// get the label with maximum votes (with a minimum of them)
		// show votes
		// cout << "Votes: " << *it << endl;
		if( *it > 5 )
		{
			// query the 'database' for the recognized name and publish the result
			m_Comms.Notify( "FACE_RECOGNIZE_RESULT", m_face_labels_db[queryLabel] );
			VERBOSE_LEVEL(2) << " ... OK (" << m_face_labels_db[idx] << ")" << endl;

			if( m_show_view )
			{
#if 0
				CImage plotImage = m_image;
				plotImage.colorImageInPlace();
				plotImage.textOut(0,0,m_face_labels_db[idx],TColor::red);
				m_display_window->showImage(plotImage);
#else
				cv::Mat img2Plot = cvarrToMat( m_image.getAs<IplImage>() );
				putText( img2Plot, m_face_labels_db[idx], cv::Point(5,10), FONT_HERSHEY_SIMPLEX, 1, Scalar(1,0,0) );
				cv::imshow( format("%d",m_obs_label), img2Plot );
#endif
			}
		}
		else
		{
			VERBOSE_LEVEL(2) << " ... OK (" << m_face_labels_db[queryLabel] << " -- conf: " << conf << "). Consolidating (Max: " << m_face_labels_db[idx] << " --  " << *it << ") ..." << endl;
		}

		return true;
	} // end-if
#endif
	// return true;

#if 0

	if( queryLabel != -1 ) // recognition was successfull
	{
		ASSERT_(queryLabel<m_face_labels_db.size())

		// votation 
		m_face_votation[queryLabel]++;
		const double av_sofar = m_face_confidence[queryLabel];
		m_face_confidence[queryLabel] = av_sofar+(conf-av_sofar)/m_face_votation[queryLabel];

		// check if the face has changed
		if( queryLabel == m_last_label )	
			m_cons_votes++;
		else 							
		{
			m_last_label = queryLabel;
			m_cons_votes = 1;
		} // end-else
		
		if( m_cons_votes >= 3 )	// clear the votation result since the face has changed
		{
			int aux_v = m_face_votation[queryLabel];
			double aux_c = m_face_confidence[queryLabel];

			// clear vectors
			std::fill(m_face_votation.begin(), m_face_votation.end(), 0);
			std::fill(m_face_confidence.begin(), m_face_confidence.end(), 0.0);

			// restore current information
			m_face_votation[queryLabel] = aux_v;
			m_face_confidence[queryLabel] = aux_c;

		} // end-if

		// show votation info
		if( m_verbose_level >= 2 )
			dumpVotation();

		vector<int>::iterator it = std::max_element(m_face_votation.begin(),m_face_votation.end());
		int idx = std::distance(m_face_votation.begin(),it);

		// Manage results for robustness and response agility (change of person)
		// map label -> votes
		// get the label with maximum votes (with a minimum of them)
		// show votes
		if( *it > 5 )
		{
			// query the 'database' for the recognized name and publish the result
			m_Comms.Notify( "FACE_RECOGNIZE_RESULT", m_face_labels_db[queryLabel] );
			VERBOSE_LEVEL(2) << " ... OK (" << m_face_labels_db[idx] << ")" << endl;

			if( m_show_view )
			{
				CImage plotImage = m_image;
				plotImage.colorImageInPlace();
				plotImage.textOut(0,0,m_face_labels_db[idx],TColor::red);
				m_display_window->showImage(plotImage);
			}
		}
		else
		{
			VERBOSE_LEVEL(2) << " ... OK (" << m_face_labels_db[queryLabel] << " -- conf: " << conf << "). Consolidating (Max: " << m_face_labels_db[idx] << " --  " << *it << ") ..." << endl;
		}

		return true;
	}
#endif
} // end-m_recognize_faces

/** Iterate */
bool CFaceRecognizerApp::Iterate()
{
	if( !m_initialized_ok || !m_start_recognizing )
		return true;

	// get image if available
	if( m_copy_image() )
	{
		if( m_add_new_face )	m_add_face();
		else					
		{
			if( !m_recognize_face() ) 
			{
				if( m_tout > 0 ) // if m_tout == 0 -> no timeout has been set, so try to detect indefinitely
				{
					const double elapsed_time = mrpt::system::timeDifference( m_ini_time, mrpt::system::now() );
					VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] No faces recognized for " << elapsed_time << " seconds (timeout: " << m_tout << " sec.)" << endl;
					if( elapsed_time > m_tout )
					{
						//! @moos_publish FACE_DETECT_RESULT TIMEOUT
						m_Comms.Notify( "FACE_RECOGNIZE_RESULT", "NONE" );
						m_start_recognizing = false;
						VERBOSE_LEVEL(1) << "[pFaceRecognizer -- INFO] Timeout! No face recognized in " << m_tout << " seconds, stop trying." << endl << "Message 'FACE_RECOGNIZE_RESULT NONE' has been sent." << endl;
					} // end-if
				} // end-if
			}
		}
	} // end-if

	return true;
} // end-Iterate

bool CFaceRecognizerApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

bool CFaceRecognizerApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}