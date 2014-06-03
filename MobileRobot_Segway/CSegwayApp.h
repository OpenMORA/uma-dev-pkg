/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Emil Jatib Khatib  <emilkhatib@uma.es>						   |
   |             Juan Antonio Infantes Diaz  <ersame@gmail.com>				   |
   |             Gregorio Navidad Vidal <gregorio.navidad@isa.uma.es>		   |                                                                           
   |																		   |
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

#ifndef CSegwayApp_H
#define CSegwayApp_H

#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils.h>
#include <mrpt/system.h>
#include <CMapirMOOSApp.h>
#include <mrpt/hwdrivers.h>
#include <mrpt/base.h>


class CSegwayApp : public CMapirMOOSApp
{
public:
    CSegwayApp();
    virtual ~CSegwayApp();

protected:
int counter,counter2;
std::string serName;
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
	/** state our interest in variables from other modules (registration for mail)*/
	bool DoRegistrations();

	/** start up the controller */
	bool startController();

	/** stop the controller */
	bool stopController();

	/** start up the base */
	bool startSegway();


	/** turn the base on */
    bool turnOn();
	/** stop the base */
	bool stopSegway();

	int numDigits(int position);

	/** Set the position of the motor (in steps) */
	bool setMotorPosition(int position);

	/** Set the speed of the motor (in rpm) */
	bool setMotorSpeed(int speed);

	/** Set maximum speed */
	bool setMaxSpeed();

	/** Configure serial port to connect to the base */
	void configurePort();

	mrpt::hwdrivers::CSerialPort serPort;


	/*
	void SendMessage(std::string);
    bool CheckNewMessage();
    std::string GetReceivedMessage();
    void PublishReceivedMessage();
    void ProcessReceivedMessage();
    PyObject* PopMessage();
	void AddVars();

    std::string openHelp(const char* filename);


	PyObject *skype,*msgList;
	long msgcount;
	std::string remoteUser;
	std::map<std::string,std::string> MOOSVars;*/

    	// Segway direction control section

	long value,k;

    float joystick;

    int steps;

	bool checkerror;

    bool startDirectionBoard();

	/**select the configuration of the board*/
	bool configure();

	/** get the rigth values in both analog output channels*/
	void move(long value);

	/** close the board */
	void close();

};

#endif
/** @moos_TODO 
  * Add documentation related to the modifications and improvements performed to the segway.
  * Provide reference to articles where the modified segway is used.
  * Rename the functions that manage the forward/backward movement just like the direction management functions: configurePort()
  * Modify the OnStartUp() method to read parameters from .ini file.
  * Develop the Linux driver
*/