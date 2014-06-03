/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2012  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |    Robotics (MAPIR) Lab, University of Malaga (Spain).                    |
   |    Contact: Carlos Sánchez  <carlossanchez@uma.es>                        |
   |                                                                           |
   |   This file is part of the MORA project.                                  |
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

/**  @moos_module  */

#include "CCalibration.h"
#include <mrpt/base.h>
#include <mrpt/scanmatching.h>
#include "opencv2/calib3d/calib3d.hpp"

using namespace System;
using namespace System::Collections::Generic;
using namespace System::Threading;
using namespace std;
using namespace mrpt;
using namespace mrpt::poses;

#include <mrpt/opengl.h>
#include <mrpt/gui.h>
#include <mrpt/slam.h>
#include <Eigen/Dense>
#include <fstream>

using namespace::mrpt;
using namespace::mrpt::opengl;
using namespace::mrpt::slam;
using namespace::Eigen;

// Constructor
CCalibration::CCalibration()
{

}

// Destructor
CCalibration::~CCalibration()
{
	
}

void CCalibration:: CalculateMatrix ()
{
	vector_double inPoints;
	vector_double outPointsQuad;
	inPoints.reserve(600);
	CPose3DQuat outQuad;

	float camera_x = 0, camera_y = 0, camera_z = 0, robot_x = 0, robot_y = 0, robot_z = 0;
	CMatrixDouble44 out_HM;  

	ifstream fp("ArmCamPoints.txt");
	ofstream fp2("TransformationMatrix.txt");
		
	while (!fp.eof())
	{
		fp>>robot_x;
		fp>>robot_y;
		fp>>robot_z;
		fp>>camera_x;
		fp>>camera_y;
		fp>>camera_z;

		inPoints.push_back(robot_x);
		inPoints.push_back(robot_y);
		inPoints.push_back(robot_z);
		inPoints.push_back(camera_x);
		inPoints.push_back(camera_y);
		inPoints.push_back(camera_z);		
	}

	cout<<endl;

	try
	{ 
		mrpt::scanmatching::HornMethod(inPoints,outQuad,true);		
		
		CPose3D pose(outQuad);
		
		pose.getHomogeneousMatrix(out_HM);  
		
		cout<< "\n\nThe transformation matrix is\n\n" << out_HM << endl; 
		fp2<<out_HM;
		
		std::system("PAUSE");

	} 
	
	catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		std::system("PAUSE");
	}
	catch (...)
	{
		printf("Untyped exception!!");
		std::system("PAUSE");
	}

	fp.close();
	fp2.close();


}

void CCalibration:: MarianoMethod()
{
	ifstream file("ArmCamPoints.txt");
	ofstream fp2("TransformationMatrix.txt");
	MatrixXf A;
	MatrixXf B;
	MatrixXf Var;
	MatrixXf At;
	MatrixXf aux;
	MatrixXf aux_inv;
	

	CSimplePointsMap parm;
	CSimplePointsMap pcam;
	TPoint3D arm, cam;
	CMatrixDouble44 global_trans;

	//This isn't very efficient because I store then and load then later...
	while (!file.eof())
	{
		file >> arm.x;
		file >> arm.y;
		file >> arm.z;
		parm.insertPoint(arm);
		file >> cam.x;
		file >> cam.y;
		file >> cam.z;
		pcam.insertPoint(cam.x, cam.y, cam.z);
		//pcam.insertPoint(-cam.x, -cam.z, -cam.y);
	}
	
	//Build the matrices and solve the least-square problem
	A.setSize(3*parm.size(), 12);
	B.setSize(3*parm.size(), 1);
	Var.setSize(12,1);

	for (unsigned int i=0; i<parm.size(); i++)
	{
		pcam.getPoint(i,cam);
		parm.getPoint(i,arm);
		A(i*3,0) = cam.x;
		A(i*3,1) = cam.y;
		A(i*3,2) = cam.z;
		A(i*3,9) = 1;
		A(i*3+1,3) = cam.x;
		A(i*3+1,4) = cam.y;
		A(i*3+1,5) = cam.z;
		A(i*3+1,10) = 1;
		A(i*3+2,6) = cam.x;
		A(i*3+2,7) = cam.y;
		A(i*3+2,8) = cam.z;
		A(i*3+2,11) = 1;
		B(i*3) = arm.x;
		B(i*3+1) = arm.y;
		B(i*3+2) = arm.z;
	}

	At = A.transpose();
	aux = At*A;
	aux_inv = aux.inverse();
	//aux.inv(aux_inv);
	Var = aux_inv*At*B;

	//Fill the transformation matrix with the solution
	global_trans(0,0) = Var(0,0);
	global_trans(0,1) = Var(1,0);
	global_trans(0,2) = Var(2,0);	
	global_trans(0,3) = Var(9,0);
	global_trans(1,0) = Var(3,0);
	global_trans(1,1) = Var(4,0);
	global_trans(1,2) = Var(5,0);	
	global_trans(1,3) = Var(10,0);
	global_trans(2,0) = Var(6,0);
	global_trans(2,1) = Var(7,0);
	global_trans(2,2) = Var(8,0);	
	global_trans(2,3) = Var(11,0);
	global_trans(3,0) = 0;
	global_trans(3,1) = 0;
	global_trans(3,2) = 0;	
	global_trans(3,3) = 1;

	cout << endl << "Transformation:  " << endl << global_trans;
	CPose3D trans(global_trans);
	
	//pose.getHomogeneousMatrix(global_trans); ////Devuelve la correspondiente matriz 4x4 de transformación homogénea para el punto (traslación) o pose (traslación + orientación). 
		
	//cout<< "\n\nThe transformation matrix is\n\n" << global_trans << endl; //Matriz
	fp2<<global_trans;

	//Show the result
	gui::CDisplayWindow3D	window;
	COpenGLScenePtr	scene;
	global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;
	window.setWindowTitle("Point transformation");
	window.resize(1000,780);
	window.setPos(800,50);
	window.setCameraZoom(4);
	scene = window.get3DSceneAndLock();

	CPointCloudPtr points1 = opengl::CPointCloud::Create();
	CPointCloudPtr points2 = opengl::CPointCloud::Create();
	CPointCloudPtr points3 = opengl::CPointCloud::Create();

	//Points in the arm coordinate system
	points1->loadFromPointsMap<CSimplePointsMap> (&parm);
	points1->enablePointSmooth(true);
	points1->setPointSize(5);
	points1->setColor(1,0,0);
	points1->setLocation(0,0,0);
	scene->insert( points1 );

	//Points in the camera coordinate system
	points2->loadFromPointsMap<CSimplePointsMap> (&pcam);
	points2->enablePointSmooth(true);
	points2->setPointSize(5);
	points2->setColor(0,1,0);
	points2->setLocation(0,0,0);
	scene->insert( points2 );

	//Points from the camera transformed into the arm coordinate system
	points3->loadFromPointsMap<CSimplePointsMap> (&pcam);
	points3->enablePointSmooth(true);
	points3->setPointSize(5);
	points3->setColor(0,0,1);
	points3->setPose(trans);
	scene->insert( points3 );

	//Grid (floor)
	opengl::CGridPlaneXYPtr grid = opengl::CGridPlaneXY::Create(-4,4,-4,4,0);
	scene->insert( grid );


	system::sleep(10);
	window.unlockAccess3DScene();
	window.repaint();

	system::os::getch();
}

void CCalibration::CalculateMatrix2 ()
{
	
}