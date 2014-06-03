	
	/*---------------------------------------------------------------
	|																|
	|			J.R. Ruiz-Sarmiento(jotaraul@uma.es)				|
	|		Department of Computer Engineering and Automatics.		|
	|			   MAPIR Group. University of Málaga				|
	|																|
	|							License:							|
	|	Creative Commons Attribution-NonCommercial-ShareAlike		|
	|	2.0 Generic (CC BY-NC-SA 2.0). Further information about	|
	|	this license here:											|
	|	http://creativecommons.org/licenses/by-nc-sa/2.0/			|
	|																|
	---------------------------------------------------------------*/


#include "CSocketCom.h"
#include <iostream>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace std;


//-----------------------------------------------------------
//					 CSocketCom
//-----------------------------------------------------------
CSocketCom::CSocketCom()
{
	m_ip	= "127.0.0.1";
	m_port	= 25557;
	init();
}

CSocketCom::CSocketCom(std::string ip="127.0.0.1", unsigned short port=25557)
{
	m_ip	= ip;
	m_port	= port;
	init();
}


//-----------------------------------------------------------
//					  ~CSocketCom
//-----------------------------------------------------------

CSocketCom::~CSocketCom()
{
	cout << "[INFO] Destroying Communication object..." << endl;

	if ( srvSocket )
		delete srvSocket;

	if ( socket )
		delete socket;

	cout << "[INFO] Communication object destroyed" << endl;
}


//-----------------------------------------------------------
//						    init
//-----------------------------------------------------------

bool CSocketCom::init()
{	

	srvSocket = new mrpt::utils::CServerTCPSocket(m_port,m_ip,10,true);
	run();
	return true;
}


//-----------------------------------------------------------
//						    run
//-----------------------------------------------------------

bool CSocketCom::run()
{
	if (srvSocket->isListening())
	{
		printf("[SERVER]: Waiting external connection to ip:%s port:%u ...\n", m_ip.c_str(), m_port);
		socket = srvSocket->accept();

		if( socket==NULL)
			printf("Connection error. socket = NULL");
		else
		{
			printf("[SERVER]: Connection established ...\n");
			//socket->setTCPNoDelay(1);
			
			if ( socket->isConnected() )
			{
				printf("[Client]: Connection established...");
				return true;
			}
			else
				return false;
		}
	}
	else
	{
		printf("ERROR: Server socket not listening\n");
		return false;
	}
}


//-----------------------------------------------------------
//						   write String
//-----------------------------------------------------------

bool CSocketCom::write(const std::string &str)
{
	//NAV_TRY

	char buf[128];
	mrpt::system::os::sprintf( buf, 128, str.c_str() );
	unsigned short toSend = str.length();

	socket->WriteBuffer( buf, toSend ); 

	//writeDebugLine( format("Writed: %s",string(buf).c_str()), COMMUNICATION );

	//socket.ReadBuffer( buf2, 5 );
	
	return true;
}

//-----------------------------------------------------------
//						   write Buffer
//-----------------------------------------------------------

bool CSocketCom::write(const void* buffer, size_t count)
{
	if ( socket->isConnected() )
	{
		socket->WriteBuffer( buffer, count );
		return true;
	}
	else
	{
		printf("Socket disconnected \n");
		return false;
	}
}

//-----------------------------------------------------------
//						    read Buffer
//-----------------------------------------------------------

bool CSocketCom::read(void* buffer, size_t count)
{
	try
	{
		if ( socket->isConnected() )
		{
			//printf("Resquested %u Bytes - ",(int) count);
			
			
			/*
				// Test Strings
				char bufchar[100000];
				size_t bytes = socket->readAsync(bufchar,count,-1,-1);
				bufchar[count+1] = '\0';		//end of strign
				printf("[client]: received '%s'\n",bufchar);
			*/

			size_t bytes = socket->readAsync(buffer,count,-1,-1);

			//printf("Got: %i \n",(int) bytes);
			
			//size_t bytes = socket->ReadBuffer(buffer,count);
			if (bytes == count)
				return true;			
			else
			{
				//printf("\nEOF or any other read error while reading\n");
				//delete socket;
				//delete srvSocket;
				//init();			//restart communication
				//printf("\nSocket restarted OK\n");
				return false;
			}
		}
		else
		{
			printf("[server]: Disconnected!\n ");
			socket->close();
			//Restart socket
			delete socket;
			run();
		}
	}
	catch (std::exception &e)
	{
		printf( "Exception reading buffer." );
		cerr << "**ERROR** " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

//-----------------------------------------------------------
//						    read
//-----------------------------------------------------------

bool CSocketCom::read(std::string &read)
{
	if (socket->getline( read ))
	{
		//writeDebugLine(format("Readed: %s", read.c_str()),COMMUNICATION); 
		return true;
	}
	else
	{
		//writeDebugLine("EOF or any other read error while reading",COMMUNICATION);
		printf("\nEOF or any other read error while reading\n");
		delete socket;
		delete srvSocket;
		init();			//restart communication
		printf("\nSocket restarted OK\n");
		return false;
	}
}


//-----------------------------------------------------------
//					 readVariousLines
//-----------------------------------------------------------

bool CSocketCom::readVariousLines(vector<std::string> &read, const size_t &n_lines)
{
	//NAV_TRY

	read.clear();
	read.resize(n_lines);

	for ( size_t i_line = 0; i_line < n_lines; i_line++ )
	{
		if (socket->getline( read[i_line] ))
		{
			//writeDebugLine(format("Readed: %s", read[i_line].c_str()),COMMUNICATION); 
			return true;
		}
		else
		{
			//writeDebugLine("EOF or any other read error while reading",COMMUNICATION);
			return false;
		}
	}
	return true;
}