/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CManPublisherApp_H
#define CManPublisherApp_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/utils/TStereoCamera.h>

class CManPublisherApp : public COpenMORAApp
{

public:
    CManPublisherApp();
    virtual ~CManPublisherApp();

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
	bool							m_initialized_ok;

};
#endif
