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
   |             Gregorio Navidad Vidal <gregorio.navidad@isa.uma.es>          |
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

/**  @moos_module The interface to a modified Segway mobile robotic base.
  *  This module enables the communications and control of a Segway based mobile robotic base.
  *  There is implemented access to the segway base, motors, controllers, odometry, ticks counts, velocities and battery charge status, as well as rudimentary velocity control.
  *  Notice that this module is only applicable for the modified segway version from the MAPIR group.
*/

#include "CSegwayApp.h"

#define MAX_MOTION_CMD_V 1.0;
const float MAX_STEPS= 31000.0;


#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
	#include <stdlib.h>
	#include <stdio.h>
	#include <olmem.h>
	#include <olerrors.h>
	#include <oldaapi.h>

	typedef struct tag_board {
	HDEV  hdrvr;        /* driver handle            */
	HDASS hdass;        /* sub system handle        */
	ECODE status;       /* board error status       */
	HBUF  hbuf;         /* sub system buffer handle */
	PWORD lpbuf;        /* buffer pointer           */
	char name[MAX_BOARD_NAME_LENGTH];  /* string for board name    */
	char entry[MAX_BOARD_NAME_LENGTH]; /* string for board name    */
	}BOARD;

	typedef BOARD* LPBOARD;
	static BOARD board;

	BOOL CALLBACK  GetDriver( PTSTR lpszName, PTSTR lpszEntry, LPARAM lParam);
#endif


void CSegwayApp::configurePort(){
	//Serial port setup
	serPort.open(serName.c_str());
    serPort.setTimeouts(10000,1,10000,1,5000);
    serPort.setConfig( 19200, 0,8,1 );
}

bool CSegwayApp::startController()
{
	// Send an "EN" command to the FTDI controller to enable it
	const char buf1[] = "EN\n";
	serPort.Write(buf1,sizeof(buf1));
	MOOSTrace("EN\n");
	// Wait for the response
	char buf2[1];
	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Sleep for a while and return the status
	mrpt::system::sleep(3);
	return nRead == 1;
}

bool CSegwayApp::stopController()
{
	// Send an "DI" command to the FTDI controller to disable it
	const char buf1[] = "DI\n";
	serPort.Write(buf1,sizeof(buf1));
	MOOSTrace("DI\n");
	// Wait for the response
	char buf2[1];
	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return the status
	return nRead == 1;
}



bool CSegwayApp::startSegway()
{
    // Send an "SCM" command to the FTDI controller to find the initial position
	const char buf0[] = "SCM-\n";
	serPort.Write(buf0,sizeof(buf0));
	MOOSTrace("SCM-\n");
	// Wait for the response
	char buf3[1];
	size_t nRead1 = serPort.Read(buf3,sizeof(buf3));

    // Wait 15 seconds for the device to prepare
    MOOSTrace("Please wait ...");
    mrpt::system::sleep(15000);
    MOOSTrace(" Initialized!\n");

	// Send an "ON" command to the FTDI controller to turn on the modified Segway
	//const char buf1[] = "ON\n";
	//serPort.Write(buf1,sizeof(buf1));
	//MOOSTrace("ON\n");
	// Wait for the response
	//char buf2[1];
	//size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return the status
	return nRead1 == 1;
}

bool CSegwayApp::turnOn()
{
    // Send an "SCM" command to the FTDI controller to find the initial position
	//const char buf0[] = "SCM-\n";
	//serPort.Write(buf0,sizeof(buf0));
	//MOOSTrace("SCM-\n");
	// Wait for the response
	//char buf3[1];
	//size_t nRead1 = serPort.Read(buf3,sizeof(buf3));

    // Wait 15 seconds for the device to prepare
    //MOOSTrace("Please wait ...");
    //mrpt::system::sleep(15000);
    //MOOSTrace(" Initialized!\n");

	// Send an "ON" command to the FTDI controller to turn on the modified Segway
	const char buf1[] = "ON\n";
	serPort.Write(buf1,sizeof(buf1));
	MOOSTrace("ON\n");
	// Wait for the response
	char buf2[1];
	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return the status
	return nRead == 1;
}




bool CSegwayApp::stopSegway()
{
	// Send an "OFF" command to the FTDI controller to turn off the modified Segway
	const char buf1[] = "OFF\n";
	serPort.Write(buf1,sizeof(buf1));
	MOOSTrace("OFF\n");
	// Wait for the response
	char buf2[1];
	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return the status
	return nRead == 1;
}

/** Auxiliary function: get the number of digits of a number */
int CSegwayApp::numDigits(int position){
  int num_digits = 0;
  while(position > 0) {
      num_digits++;
      position/=10;
  }
  return num_digits;
}


/** Set the position of the motor (in steps) */
bool CSegwayApp::setMotorPosition(int position)
{

	char buf1[11];
	int num_digits = numDigits(position);

	// Decide if movement is positive (forward) or negative (backward)
	if (position >= 0){
		sprintf(buf1,"SP+");
	}else{
		sprintf(buf1,"SP-");
	}

	// Fill with zeroes to get a SP+0000XY format (where XY are the digits of the position, supposing it had two digits)
	for (int i = 0; i < 6-num_digits; i ++){
		sprintf(buf1,"%s0",buf1);
	}

	// Put together the command and send it
	sprintf(buf1,"%s%d\n",buf1,position);
	serPort.Write(buf1,sizeof(buf1));
	MOOSTrace("SPX\n");
	// Read the response
	//char buf2[6];
//	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return status
//	return nRead == 1;
    return true;
}

/** Set the speed of the motor (in rpm) */
bool CSegwayApp::setMotorSpeed(int speed)
{

	char buf1[11];
	int num_digits = numDigits(speed);

	// Decide if speed is positive (forward) or negative (backward)
	if (speed >= 0){
		sprintf(buf1,"SV+");
	}else{
		sprintf(buf1,"SV-");
	}

	// Fill with zeroes to get a SV+0000XY format (where XY are the digits of the position, supposing it had two digits)
	for (int i = 0; i < 6-num_digits; i ++){
		sprintf(buf1,"%s0",buf1);
	}

	// Put together the command and send it
	sprintf(buf1,"%s%d\n",buf1,speed);
	serPort.Write(buf1,sizeof(buf1));
	MOOSTrace("SVX\n");
	// Read the response
	char buf2[6];
	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return status
	return nRead == 1;
}

/** Set maximum speed */
bool CSegwayApp::setMaxSpeed()
{
	// prepare and send the command
	const char buf1[] = "SCP019999\n";
	serPort.Write(buf1,sizeof(buf1));
    MOOSTrace("SCP019999\n");
	// Wait for response
	char buf2[6];
	size_t nRead = serPort.Read(buf2,sizeof(buf2));

	// Return status
	return nRead == 1;
}




CSegwayApp::CSegwayApp()
{
MOOSTrace("CSegwayApp::CSegwayApp()\n");
    counter2 = 0;
	counter =0;
}

CSegwayApp::~CSegwayApp()
{
	// stop the base (in case it is not stopped)
	stopSegway();

	// stop the controller

	stopController();
}

bool CSegwayApp::OnStartUp()
{

MOOSTrace("CSegwayApp::OnStartUp()\n");
    DoRegistrations();
    joystick = 0;
    startDirectionBoard();
	m_MissionReader.GetConfigurationParam( "com_port", serName);


    // configure the port (lineal speed)
    /// Todo: rename the functions that manage the forward/backward movement just like the direction management functions
	configurePort();

	// start the controller
	startController();

    // start the Segway
	startSegway();
return true;

}

bool CSegwayApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pMobileRobot_Segway only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CSegwayApp::Iterate()
{

//MOOSTrace("CSegwayApp::Iterate()\n");
// Move in the commanded direction
//move(value); /// Identify as a direction management function, may look confusing
    if (counter%6 == 0){
        counter = 1;
        printf("%i\n",steps);
        setMotorPosition(steps);
        if (joystick == 1){
            turnOn();
            //MOOSTrace("ON\n");
            joystick = 0;
        }
        if (joystick == 4){
            stopSegway();
            //MOOSTrace("OFF\n");
            joystick = 0;
        }
    }
    counter++;


return true;
}


bool CSegwayApp::OnConnectToServer()
{

MOOSTrace("CSegwayApp::OnConnectToServer()\n");
    DoRegistrations();
    return true;
}


bool CSegwayApp::DoRegistrations()
{

MOOSTrace("CSegwayApp::DoRegistrations()\n");
	//! @moos_subscribe MOTION_CMD_V
    this->m_Comms.Register("MOTION_CMD_V",0);
	//! @moos_subscribe MOTION_CMD_W
    this->m_Comms.Register("MOTION_CMD_W",0);
	//! @moos_subscribe BUTTONS_CMD
    this->m_Comms.Register("BUTTONS_CMD",0);

	//! @moos_subscribe SHUTDOWN
	this->m_Comms.Register("SHUTDOWN",0);

    RegisterMOOSVariables();
    return true;
}

bool CSegwayApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
//MOOSTrace("CSegwayApp::OnNewMail(MOOSMSG_LIST &NewMail)\n");
	//int steps;
	float v2;
    //bool b;	
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
	    const CMOOSMsg &m = *it;

		if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_V")){

			    v2 = m.GetDouble(); // / MAX_MOTION_CMD_V;

			    steps = floor((-v2*32000) + MAX_STEPS);
          //if (abs(counter-steps) > 5000){
            //    std::cout << "Steps: " << steps << " double: " << m.GetDouble() << " proportion: " << v2 << "/n";
			 //   setMotorPosition(steps);
              //  counter = steps;
            //}
		}
		if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_W")){
            value= m.GetDouble();   /// Todo: adapt the value to that given by the joystick
            int moveqty = floor((value/1.1)*(-12));
            if (abs(counter2-moveqty) > 5){
                move(moveqty);
                printf("%d \n",moveqty);
                counter2 = moveqty;
            }
		}

        if (MOOSStrCmp(m.GetKey(),"BUTTONS_CMD")){
                //unsigned* rb;
            if (joystick == 0)
                joystick = m.GetDouble();
                //rb = (unsigned*)&rvalue;
                /*if (rvalue == 1){
                    b = turnOn();
                    MOOSTrace("ON\n");
                }
                if (rvalue== 4){
                    b = stopSegway();
                    MOOSTrace("OFF\n");
                }
               printf("%f \n",rvalue);*/

		}

/*		if( (i->GetName()=="TURNSEGWAY") )
		{
			value= i->GetDouble();
			printf("%d \n",value);
		}
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			Segway::close();
			this->RequestQuit();
		}*/

		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
		{
			// Disconnect comms:
			/*MOOSTrace("Disconnecting and Disabling Motors \n");
			m_robot.disconnectAndDisableMotors();
			mrpt::system::sleep(1000);
			MOOSTrace("Closing Module \n");*/
            MOOSTrace("Closing Module \n");
			CSegwayApp::close();
			this->RequestQuit();

		}
	}

    UpdateMOOSVariables(NewMail);
    return true;
}


// Direction Management functions



//-------------------------------------
// GetDriver
//-------------------------------------
#ifdef MRPT_OS_WINDOWS

BOOL CALLBACK GetDriver( PTSTR lpszName, PTSTR lpszEntry, LPARAM lParam)
/*
this is a callback function of olDaEnumBoards, it gets the
strings of the Open Layers board and attempts to initialize
the board.  If successful, enumeration is halted.
*/
{
   LPBOARD lpboard = (LPBOARD)(LPVOID)lParam;

   /* fill in board strings */

  lstrcpyn(lpboard->name,lpszName,MAX_BOARD_NAME_LENGTH-1);
   lstrcpyn(lpboard->entry,lpszEntry,MAX_BOARD_NAME_LENGTH-1);

   /* try to open board */

   lpboard->status = olDaInitialize(lpszName,&lpboard->hdrvr);

   if   (lpboard->hdrvr != NULL)
     return FALSE;          /* false to stop enumerating */
	else
      return TRUE;           /* true to continue          */
}
#endif

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CSegwayApp::startDirectionBoard()
{
#ifdef MRPT_OS_WINDOWS
	board.hdrvr = NULL;
	olDaEnumBoards(&GetDriver,(LPARAM)(LPBOARD)&board);

	checkerror= this->configure();

	if (checkerror){
		printf("CONFIGURATION OK \n");

	}else{
		printf("ERROR¡¡¡¡¡ \n");
	}
	value=0.0;
	return DoRegistrations();
#else
	MRPT_TODO("Try to implement a Linux variant of this module if possible")
	THROW_EXCEPTION("Only implemented for Windows")
#endif
}

//-------------------------------------
// Doregistrarions()
//-------------------------------------
/*
bool Segway::DoRegistrations()
{
	
	//AddMOOSVariable("TURNSEGWAY","TURNSEGWAY","TURNSEGWAY",0);
	
	//AddMOOSVariable("TASK_PLANNED","TASK_PLANNED","TASK_PLANNED",0);
	
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0.1 );
	
	AddMOOSVariable( "TURNSEGWAY","TURNSEGWAY","TURNSEGWAY",0.1);
	RegisterMOOSVariables();
	return true;
}
*/



//-------------------------------------
// configure()
//-------------------------------------
bool CSegwayApp::configure(){
#ifdef MRPT_OS_WINDOWS
	if (board.hdrvr == NULL){
		return FALSE;
	}else{

		olDaGetDASS(board.hdrvr,OLSS_DA,0,&board.hdass);

		/* set subsystem for single value operation */

		olDaSetDataFlow(board.hdass,OL_DF_SINGLEVALUE);
		olDaConfig(board.hdass);
		return TRUE;
	}
#else
	THROW_EXCEPTION("Only implemented for Windows")
#endif
}
//-------------------------------------
// move(value)
//-------------------------------------
void CSegwayApp::move(long value){
#ifdef MRPT_OS_WINDOWS
	//for(k=0; k<value; k++){

		olDaPutSingleValue(board.hdass,(617+value),0,1.0);
		olDaPutSingleValue(board.hdass,(573-value),1,1.0);
		//Sleep(30);
	//}

	 //   olDaPutSingleValue(board.hdass,(617),0,1.0);
	 //   olDaPutSingleValue(board.hdass,(573),1,1.0);
#else
	THROW_EXCEPTION("Only implemented for Windows")
#endif
}
//-------------------------------------
// close(board)
//-------------------------------------
void CSegwayApp::close()
{
#ifdef MRPT_OS_WINDOWS
	olDaReleaseDASS(board.hdass);
	olDaTerminate(board.hdrvr);
#else
	THROW_EXCEPTION("Only implemented for Windows")
#endif
}
