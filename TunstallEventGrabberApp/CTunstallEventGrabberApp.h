/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CTunstallEventGrabberApp_H
#define CTunstallEventGrabberApp_H

#include "TunstallSoprano.h"

#include <COpenMORAMOOSApp.h>

#include <mrpt/utils/CImage.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>

enum TUNSTALL_EVENT_TYPE {
	TET_UNKNOWN = 0, TET_HANDSHAKE, TET_NONE,
	TET_DOOR_OPEN, TET_DOOR_CLOSED, 
	TET_CHAIR_ON,  TET_CHAIR_OFF
};

typedef struct 
{
	mrpt::system::TTimeStamp	tstamp;
	std::string					plain_event;
	TUNSTALL_EVENT_TYPE			tunstall_event;

} TTunstallEvent;

class CTunstallEventGrabberApp : public COpenMORAApp
{

public:
    CTunstallEventGrabberApp();
    virtual ~CTunstallEventGrabberApp();

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

	/* custom methods */
	int wrapConnectAthena(const int & port) const;
	int wrapConnectAthena(const char* in);
	void wrapConnectAthena(const char* in, char* out);
	int wrapDisconnectAthena() const;
	TUNSTALL_EVENT_TYPE string2tet(const std::string & in_event) const;

	bool getTunstallEvent( TTunstallEvent & out_event ) const; // true: valid event, false: invalid event
	
	// -----------------------------------------------
	// DATA. Your local variables here...
	// -----------------------------------------------
	bool							m_initialized_ok;
	bool							m_show_view;		//!< Whether or not to show the images 
	bool							m_save_to_file;		//!< Whether or not to save the events to a text file
	bool							m_start_grabbing;	//!< Whether or not to start to grab data

	int								m_verbose_level;
	int								m_port;				//!< COM port number

	mrpt::gui::CDisplayWindowPtr	m_display_window;	//!< The display window (if desired)
	mrpt::utils::CImage				m_image;
	mrpt::utils::CTicTac			m_tictac;			//!< To measure spent time

	double							m_tout;

	mrpt::system::TTimeStamp		m_ini_time;
	std::string						m_log_filename;
	FILE							*m_log_file;

};
#endif
