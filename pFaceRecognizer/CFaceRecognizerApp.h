/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CFaceRecognizerApp_H
#define CFaceRecognizerrApp_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

enum MODEL_TYPE {LBPH = 0, EigenFaces, FisherFaces};

using namespace std;

class CFaceRecognizerApp : public COpenMORAApp
{

public:
    CFaceRecognizerApp();
    virtual ~CFaceRecognizerApp();

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

	// main methods
	bool m_copy_image();
	bool m_recognize_face();

	bool m_get_correct_eye_position( 
		const std::vector<cv::Rect> & eyesL,							// input left eye positions (may contain multiple positions)
		const std::vector<cv::Rect> & eyesR,							// input right eye positions (may contain multiple positions)
		const int					w,									// image width
		cv::Point					& left_eye_pos,						// output: actual position of the left eye
		cv::Point					& right_eye_pos,					// output: actual position of the right eye
		cv::Rect					& left_eye = cv::Rect(),			// optional output: actual left eye rectangle
		cv::Rect					& right_eye = cv::Rect() ) const;	// optional output: actual right eye rectangle	

	void m_transform_image(
		const cv::Mat	& input,			// input image
		const cv::Point & left_eye,			// left eye position
		const cv::Point & right_eye,		// right eye position
		cv::Mat			& output ) const;	// scaled, rotated and translated image to 

	bool m_preprocess_image( 
		cv::Mat & out_image );

	bool m_add_face();

	inline int m_number_of_faces() const { return m_face_labels_db.size(); }
	void dumpVotation( const int i ) const;

	// -----------------------------------------------
	// DATA. Your local variables here...
	// -----------------------------------------------
	int								m_verbose_level;
	bool							m_show_view;
	bool							m_debug;
	bool							m_initialized_ok;				
	bool							m_start_recognizing;	//!< Whether or not to start to find faces
	bool							m_add_new_face;
	mrpt::utils::CTicTac			m_tictac;				//!< To measure the spent time 

	mrpt::utils::CImage				m_image;
	int								m_img_w, m_img_h;
	cv::Mat							m_ellipse_roi;			//!< Ellipsoidal ROI for face recognition
	mrpt::gui::CDisplayWindowPtr	m_display_window;		//!< The display window

	cv::Ptr<cv::FaceRecognizer>		m_recognizer;								//!< The face recognizer
	cv::Ptr<cv::FaceRecognizer>		m_recognizer2;								//!< The face recognizer
	cv::Ptr<cv::FaceRecognizer>		m_recognizer3;								//!< The face recognizer
	cv::CascadeClassifier			m_left_eye_detector, m_right_eye_detector;	//!< The eye cascade classifiers
	MODEL_TYPE						m_model_type;

	// consolidation and timeout stage
	unsigned int					m_facerec_consolidation, m_facerec_consolidation_th;
	double							m_tout;
	mrpt::system::TTimeStamp		m_ini_time;
	int								m_temptative_class;

	// face database and votation scheme
	vector<string>					m_face_labels_db;
	int								m_num_classes;

	// observation label
	int								m_obs_label;
	
	// adding new face scheme
	vector<cv::Mat>					m_new_face_images;
	vector<int>						m_new_face_labels;
	int								m_new_face_th;
	int								m_new_label;
	
	struct TClassFiltProb {
		int m_n_classes;				//!< Number of classes in the recognizer
		cv::Mat m_bel_1;				//!< Will be 'm_n_classes' x 1
		cv::Mat	m_prior_prob;			//!< Will be 'm_n_classes' x 1 (maybe variable)
		cv::Mat m_transition_model;		//!< Will be 'm_n_classes' x 'm_n_classes' and constant
		double m_prob_repeat;			//!< Probability of repeating the result
		bool m_initialized;				//!< Initialized flag

		/** default constructor */
		TClassFiltProb::TClassFiltProb() : m_initialized(false) {}
		
		/** initialization constructor */
		TClassFiltProb::TClassFiltProb( 
			const int		& n_classes,	// number of classes
			const double	& prob_rep )	// probability of repeating classes in consecutive time-steps
		{
			initialize(n_classes,prob_rep);
		} 

		/** get most likely class */
		void TClassFiltProb::getMostLikelyClass( const cv::Mat & distances, size_t & best_class, double & best_class_belief )
		{
			// get minimum distance for each class
			map<size_t,double> min_distances_class;
			m_get_min_distance_class(distances,min_distances_class);

			// DEBUG: save to file
			FILE *f = mrpt::system::os::fopen("face_distances.txt","at");
			mrpt::system::os::fprintf(f,"-1,-1.0\n");
			for( map<size_t,double>::const_iterator it = min_distances_class.begin(); it != min_distances_class.end(); ++it )
				mrpt::system::os::fprintf(f,"%d,%.5f\n",it->first,it->second);
			mrpt::system::os::fclose(f);
	
			// compute posterior for each class
			std::vector<double> class_posterior;
			m_get_class_posterior( min_distances_class, class_posterior );

			// DEBUG: save to file
			f = mrpt::system::os::fopen("face_posterior.txt","at");
			mrpt::system::os::fprintf(f,"-1,-1.0\n");
			for( size_t i = 0; i < class_posterior.size(); ++i )
				mrpt::system::os::fprintf(f,"%d,%.5f\n",i,class_posterior[i]);
			mrpt::system::os::fclose(f);

			// get new belief and best class
			m_get_best_class( class_posterior, best_class, best_class_belief );
		}

		/** initialization */
		void TClassFiltProb::initialize( const int & n_classes, const double & prob_rep )
		{
			m_n_classes = n_classes;
			m_prob_repeat = prob_rep;

			// uniform probability for the classes
			m_prior_prob = cv::Mat::ones(m_n_classes,1,CV_64FC1);
			m_prior_prob *= 1.0/m_n_classes;

			// transition model:
			//				[p	m	.	m]
			// P(Ct|Ct-1) = [m	p	.	m] where m = (1-p)/m_n_classes and p = prob_rep
			//				[.	.	.	m]
			//				[m	.	m	p]
			m_transition_model = cv::Mat::ones(m_n_classes,m_n_classes,CV_64FC1);
			const double m = (1-m_prob_repeat)/m_n_classes;
			m_transition_model *= m;
			for(int i = 0; i < m_transition_model.rows; ++i)
				m_transition_model.at<double>(i,i) = m_prob_repeat;

			// initial belief
			m_bel_1 = cv::Mat::ones(m_n_classes,1,CV_64FC1);
			m_bel_1 *= 1.0/m_n_classes;

			m_initialized = true;
		}

	private:

		/** get minimun distance for each class */
		void TClassFiltProb::m_get_min_distance_class( 
			const cv::Mat			& distances, 
			std::map<size_t,double> & min_distances ) const
		{
			for( int i = 0; i < distances.rows; ++i )
			{
				const size_t this_idx = size_t(distances.at<double>(i,0)); 
				const double this_dis = distances.at<double>(i,1);
				if( min_distances.find(this_idx) == min_distances.end() )
					min_distances[this_idx] = this_dis;		// create
				else
				{
					if( this_dis < min_distances[this_idx] )
					min_distances[this_idx] = this_dis;		// update
				}
					
			} // end-for
		} // end-getMinDistanceClass

		/** getClassProbabilities */
		void TClassFiltProb::m_get_class_posterior( 
			const map<size_t,double>	& distances, 
			std::vector<double>			& posterior )
		{
			posterior.resize(m_n_classes);
			double total = 0.0;
			size_t i = 0;
			for( map<size_t,double>::const_iterator it = distances.begin(); it != distances.end(); ++it, ++i )
			{
				posterior[i] = 1.0/it->second;
				total += posterior[i];
			}
			
			// normalize to sum 1
			for( int i = 0; i < posterior.size(); ++i )
				posterior[i] /= total; 
		} // end-getClassProbabilities
		
		/** get best class */
		void TClassFiltProb::m_get_best_class( 
			const vector<double>	& posterior, 
			size_t					& best_class, 
			double					& best_class_belief)
		{
			// check initialization
			if( !m_initialized )
				THROW_EXCEPTION("FaceRecognizer -- Probabilistic filter for recognizing not initialized. Call to 'initialize' with the desired parameters.")

			// compute this belief
			vector<double> bel(m_n_classes,0.0);
			double total = 0.0;
			for( size_t i = 0; i < m_n_classes; ++i )
			{
				const double k = posterior[i]/m_prior_prob.at<double>(i);
				//cout << k << "*" << endl << "[";
				for( size_t k = 0; k < m_n_classes; ++k )
					bel[i] += m_transition_model.at<double>(k,i)*m_bel_1.at<double>(k);
				
				bel[i] *= k;
				total += bel[i];

				/** /
				for( size_t k = 0; k < m_n_classes; ++k )
					cout << setprecision(4) << m_transition_model.at<double>(k,i) << ",";
				cout << "]*" << endl;
				
				for( size_t k = 0; k < m_n_classes; ++k )
					cout << setprecision(4) << m_bel_1.at<double>(k) << ",";
				cout << "]" << endl;
				/**/
			}

			// normalize belief to sum 1
			for( size_t i = 0; i < m_n_classes; ++i )
				bel[i] /= total;

			// DEBUG
			/** /
			cout << endl << "Posterior and belief: " << endl;
			for( size_t i = 0; i < m_n_classes; ++i )
				cout << "[" << i << "]: "<< posterior[i] << "," << bel[i] << endl;
			/**/
			
			// look for the max
			best_class = 0;
			best_class_belief = 0.0;
			for( int i = 0; i < bel.size(); ++i )
			{
				if( bel[i] > best_class_belief )
				{
					best_class_belief = bel[i];
					best_class = i;
				}
			} // end-for

			// update belief
			for( size_t i = 0; i < m_n_classes; ++i )
				m_bel_1.at<double>(i) = bel[i];

		} // end-method
	}; // end-struct
	
	// for controlling the face recognition consolidation
	struct TVotationInfo {
		vector<int>				m_face_votation;
		vector<double>				m_face_confidence;
		int								m_cons_votes;
		int								m_last_label;

		// def constructor
		TVotationInfo::TVotationInfo() : 
				m_face_votation( vector<int>() ),
				m_face_confidence( vector<double>() ),
				m_cons_votes(0),
				m_last_label(-1) {}
		
		// initializer constructor
		TVotationInfo::TVotationInfo( const int s, const int queryLabel ) : 
				m_cons_votes(0),
				m_last_label( queryLabel ) 
		{
			m_face_votation.resize(s);
			m_face_confidence.resize(s);
		}

		TVotationInfo & operator=( const TVotationInfo & other )
		{
			this->m_cons_votes = other.m_cons_votes;
			this->m_last_label = other.m_last_label;
			this->m_face_votation.resize(other.m_face_votation.size());
			this->m_face_confidence.resize(other.m_face_confidence.size());
			std::copy(other.m_face_votation.begin(), other.m_face_votation.end(), this->m_face_votation.begin());
			std::copy(other.m_face_confidence.begin(), other.m_face_confidence.end(), this->m_face_confidence.begin());
			return *this;
		}

		/** dumpToConsole */
		void TVotationInfo::dumpToConsole( const vector<string> & face_db ) const
		{
			cout << endl << "=== VOTATION LIST =================" << endl;
			for(int j = 0; j < m_face_votation.size(); ++j)
			{
				if( m_face_votation[j] > 0 )
					cout << "[" << face_db[j] << "] --> " << m_face_votation[j] << " : " << m_face_confidence[j] << endl;
			}
			cout << " Last label: " << m_last_label << endl;
			cout << " Consecutive votes: " << m_cons_votes << endl;
			cout << "====================================" << endl;
		}
	};

	map<int,TVotationInfo>			m_votes;

	TClassFiltProb					m_class_track;

};
#endif
