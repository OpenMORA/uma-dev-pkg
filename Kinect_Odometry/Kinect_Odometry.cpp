
#include "Kinect_Odometry.h"
#include <iostream>

CDifOdo::CDifOdo()
{
	rows = 91;
	cols = 121;
	fovh = M_PI*58.0/180.0;
	fovv = M_PI*45.0/180.0;
	
	A.resize(3*(rows-2)*(cols-2),6+2*(rows-2)*(cols-2));
	Var.setSize(6+2*(rows-2)*(cols-2),1);
	B.setSize(3*(rows-2)*(cols-2),1);
	dyp.setSize(rows,cols);
	dyp.assign(0);
	dzp.setSize(rows,cols);
	dzp.assign(0);
	dt.setSize(rows,cols);
	dt.assign(0);
	depth.setSize(rows,cols);
	depth_old.setSize(rows,cols);
	depth_inter.setSize(rows,cols);
	depth_ft.setSize(480,640);
	yy.setSize(rows,cols);
	yy_inter.setSize(rows,cols);
	yy_ft.setSize(480,640);
	zz.setSize(rows,cols);
	zz_inter.setSize(rows,cols);
	zz_ft.setSize(480,640);
	tita.setSize(rows,cols);
	phi.setSize(rows,cols);
	borders.setSize(rows,cols);
	borders.assign(0);
	Nan.setSize(rows,cols);
	Nan.assign(0);
	dtita.setSize(rows,cols);
	dtita.assign(0);
	dphi.setSize(rows,cols);
	dphi.assign(0);
	
	num_border_points = 0;
	num_nan_points = 0;
	num_border_and_nan = 0;

	f_dist = 1.0/525.0;																	//In meters
	y_incr = 2.0*f_dist*(floor(640.0/float(cols))*cols/640.0)*tan(0.5*fovh)/(cols-1);	//In meters
	z_incr = 2.0*f_dist*(floor(480.0/float(rows))*rows/480.0)*tan(0.5*fovv)/(rows-1);	//In meters
	t_incr_inv = 2;																		//In Hz
	dt_threshold = 0.30;
	dyz_threshold = 0.12;

	entering_cloud = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);

}

void CDifOdo::calculateCoord()
{
	for (int y = 0; y < cols; y++)
	{
		for (int z = 0; z < rows; z++)
		{
			depth_inter(z,y) = 0.5*(depth(z,y) + depth_old(z,y));
			yy_inter(z,y) = 0.5*(yy(z,y) + yy_old(z,y));
			zz_inter(z,y) = 0.5*(zz(z,y) + zz_old(z,y));
			phi(z,y) = atan2(yy_inter(z,y),depth_inter(z,y));
			tita(z,y) = atan2(zz_inter(z,y), depth_inter(z,y)/cos(phi(z,y)));		
		}
	}
}


void CDifOdo::calculateDyp()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dyp(z,y) = 0.5*(depth_inter(z,y+1) - depth_inter(z,y-1));
		}
	}
}


void CDifOdo::calculateDyp2t()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dyp(z,y) = 0.25*(depth(z,y+1) - depth(z,y-1) + depth_old(z,y+1) - depth_old(z,y-1));	//Instantes inicial y final
		}
	}
}

void CDifOdo::calculateDyp3t()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dyp(z,y) = 0.08333*(4*(depth_inter(z,y+1) - depth_inter(z,y-1)) + depth(z,y+1) - depth(z,y-1) + depth_old(z,y+1) - depth_old(z,y-1));	//Parabólica con 3 instantes
		}
	}
}

void CDifOdo::calculateDzp()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dzp(z,y) = 0.5*(depth_inter(z+1,y) - depth_inter(z-1,y));
		}
	}
}

void CDifOdo::calculateDzp2t()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dzp(z,y) = 0.25*(depth(z+1,y) - depth(z-1,y) + depth_old(z+1,y) - depth_old(z-1,y));	//Instantes inicial y final
		}
	}
}

void CDifOdo::calculateDzp3t()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dzp(z,y) = 0.08333*(4*(depth_inter(z+1,y) - depth_inter(z-1,y)) + depth(z+1,y) - depth(z-1,y) + depth_old(z+1,y) - depth_old(z-1,y));	//Parabólica con 3 instantes
		}
	}
}

void CDifOdo::calculateDt()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dt(z,y) = t_incr_inv*(depth(z,y) - depth_old(z,y));
		}
	}
}

void CDifOdo::calculateDt9p()
{
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			dt(z,y) = 10.0*(depth(z,y) - depth_old(z,y));		
			dt(z,y) += depth(z-1,y-1) - depth_old(z-1,y-1);
			dt(z,y) += depth(z-1,y) - depth_old(z-1,y);
			dt(z,y) += depth(z-1,y+1) - depth_old(z-1,y+1);
			dt(z,y) += depth(z,y-1) - depth_old(z,y-1);
			dt(z,y) += depth(z,y+1) - depth_old(z,y+1);
			dt(z,y) += depth(z+1,y-1) - depth_old(z+1,y-1);
			dt(z,y) += depth(z+1,y) - depth_old(z+1,y);
			dt(z,y) += depth(z+1,y+1) - depth_old(z+1,y+1);
			dt(z,y) *= t_incr_inv/18.0;
		}
	}
}

void CDifOdo::filterPointsAndUpdate()
{
	//Push the frames back
	depth_old = depth;
	yy_old = yy;
	zz_old = zz;

	//Filter the new PointCloud
	filter.setInputCloud(entering_cloud);
	filter.setEarlyDivision(0);
	//filter.setSigmaS(10);
	//filter.setSigmaR(0.05);
	filter.filter(filtered_cloud);


	//Downsample the pointcloud
	const float iniz = 0.5*(480-floor(float(480)/float(rows))*rows);
	const float iniy = 0.5*(640-floor(float(640)/float(cols))*cols);

	for (unsigned int y = 0; y < 640; y++)
	{
		for (unsigned int z = 0; z < 480; z++)
		{
			depth_ft(z,y) = filtered_cloud.points[y*480+z].z;
			yy_ft(z,y) = filtered_cloud.points[y*480+z].y;
			zz_ft(z,y) = filtered_cloud.points[y*480+z].x;
		}
	}
	for (int y = 0; y < cols; y++)
	{
		for (int z = 0; z < rows; z++)
		{
			depth(z,y) = depth_ft(iniz+z*floor(float(480)/float(rows)),iniy+y*floor(float(640)/float(cols)));
			yy(z,y) = yy_ft(iniz+z*floor(float(480)/float(rows)),iniy+y*floor(float(640)/float(cols)));
			zz(z,y) = zz_ft(iniz+z*floor(float(480)/float(rows)),iniy+y*floor(float(640)/float(cols)));
		}
	}

}

void CDifOdo::findBorders()
{
	const float maxdifdy = dyp.maximum() - dyp.minimum() + 0.005;
	const float maxdifdz = dzp.maximum() - dzp.minimum() + 0.005;
	const float maxdifdt = dt.maximum() - dt.minimum() + 0.005;
	num_border_points = 0;
	borders.assign(0);
		
	//Detect borders without jump
	//for (unsigned int y = 1; y < cols-1; y++)
	//{
	//	for (unsigned int z = 1; z < rows-1; z++)
	//	{
	//		if ((abs(dyp(z,y+1)-dyp(z,y)) > (1.2-4*pose_incr)*maxdifdy/sqrt(float(cols)))&&(y <cols-2))
	//		{
	//			borders(z,y) = 1;
	//			borders(z,y+1) = 1;
	//		}
	//		if ((abs(dzp(z+1,y)-dzp(z,y)) > (1.2-4*pose_incr)*maxdifdz/sqrt(float(rows)))&&(z <rows-2))
	//		{
	//			borders(z,y) = 1;
	//			borders(z+1,y) = 1;
	//		}
	//		if ((abs(dyp(z+1,y)-dyp(z,y)) > (1.2-4*pose_incr)*maxdifdy/sqrt(float(cols)))&&(z <rows-2))
	//		{
	//			borders(z,y) = 1;
	//			borders(z+1,y) = 1;
	//		}
	//		if ((abs(dzp(z,y+1)-dzp(z,y)) > (1.2-4*pose_incr)*maxdifdz/sqrt(float(rows)))&&(y <cols-2))
	//		{
	//			borders(z,y) = 1;
	//			borders(z,y+1) = 1;
	//		}
	//		if (abs(depth(z,y+1)-depth(z,y-1)-(depth_old(z,y+1)-depth_old(z,y-1))) > 4*maxdifdy/sqrt(float(cols)))
	//		{
	//			borders(z,y) = 1;
	//		}
	//		if (abs(depth(z+1,y)-depth(z-1,y)-(depth_old(z+1,y)-depth_old(z-1,y))) > 4*maxdifdz/sqrt(float(rows)))
	//		{
	//			borders(z,y) = 1;
	//		}

	//	}
	//}
	//Detect borders with jump
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			if (abs(dt(z,y)) > dt_threshold)
			{
				borders(z,y) = 1;
			}
			if (abs(dzp(z,y)) > dyz_threshold)
			{
				borders(z,y) = 1;
			}
			if (abs(dyp(z,y)) > dyz_threshold)
			{
				borders(z,y) = 1;
			}
			if (abs(depth(z,y+1)-depth(z,y-1)-(depth_old(z,y+1)-depth_old(z,y-1))) > 0.05)
			{
				borders(z,y) = 1;
			}
			if (abs(depth(z+1,y)-depth(z-1,y)-(depth_old(z+1,y)-depth_old(z-1,y))) > 0.05)
			{
				borders(z,y) = 1;
			}
			//printf("\n (Dt, Dz, Dy) = (%f, %f, %f)", dt(z,y), dzp(z,y), dyp(z,y));
		}
	}

	num_border_points = borders.sumAll(); 
	cout << "Numero de puntos en bordes: " << num_border_points;
}

void CDifOdo::findNaNPoints()
{
	num_nan_points = 0;
	Nan.assign(0);
	points_nan.clear();
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			//if ((!pcl_isfinite(yy_inter(z,y)))||(!pcl_isfinite(dzp(z,y)))||(!pcl_isfinite(dyp(z,y)))||(!pcl_isfinite(dt(z,y))))
			if (!pcl_isfinite(yy_inter(z,y)))
			{
				Nan(z,y) = 1;
				points_nan.insertPoint(depth_inter(z,y), yy_inter(z,y), zz_inter(z,y));
				//printf("\nPunto(%d,%d) = (%f,%f,%f)",z,y,depth(z,y),yy(z,y),zz(z,y));
			}
		}
	}
	num_nan_points = Nan.sumAll();
	cout << endl << "Cantidad de puntos NAN encontrados: " << num_nan_points;
}

void CDifOdo::countPointsInBordersAndNaN()
{
	num_border_and_nan = 0;
	for (int y = 1; y < cols-1; y++)
	{
		for (int z = 1; z < rows-1; z++)
		{
			if ((borders(z,y) == 1) && (Nan(z,y) == 1))
				num_border_and_nan++;
		}
	}
}

void CDifOdo::getPointsInBorders()
{
	points_borders.clear();
	for (int z = 0; z < rows; z++)
	{
		for (int y = 0; y < cols; y++)
		{
			if (borders(z,y) == 1)
			{
				points_borders.insertPoint(depth(z,y), yy(z,y), zz(z,y));
			}
		}
	}
}



void CDifOdo::solveSystemSparse()
{
	//CTicTac	clock;
	unsigned int cont = 0;
	vector <Triplet<float>> coord;
	A.resize(3*((rows-2)*(cols-2)-num_border_points-num_nan_points+num_border_and_nan),6+2*((rows-2)*(cols-2)-num_border_points-num_nan_points+num_border_and_nan));
	Var.setSize(6+2*((rows-2)*(cols-2)-num_border_points-num_nan_points+num_border_and_nan),1);
	B.setSize(3*((rows-2)*(cols-2)-num_border_points-num_nan_points+num_border_and_nan),1);

	//clock.Tic();

	//Fill the matrix A and the vector B
	//The order of the variables will be (vx, vy, vz, wx, wy, wz, wtita1, wphi1, wtita2, wphi2.... wtitanN, wphiN)
	//The points order will be (1,1), (1,2)...(1,cols-1), (2,1), (2,2)...(row-1,cols-1). Points at the borders are not included

	const float f_inv_y = f_dist/y_incr;
	const float f_inv_z = f_dist/z_incr;

	for (int z = 1; z < rows-1; z++)
	{
		for (int y = 1; y < cols-1; y++)
		{
			if ((borders(z,y) == 0)&&(Nan(z,y) == 0))
			{
				float inv_cosphi = 1.0f/cos(phi(z,y));
				float inv_costita = 1.0f/cos(tita(z,y));
				float tanphi = tan(phi(z,y));
				float tantita = tan(tita(z,y));
				float elem1 = dzp(z,y)*f_inv_z*inv_costita*inv_costita*inv_cosphi;
				float elem2 = dyp(z,y)*f_inv_y*inv_cosphi*inv_cosphi + dzp(z,y)*f_inv_z*tanphi*tantita*inv_cosphi;
				
				//1.First equation
				coord.push_back(Triplet<float>(3*cont,0,1.0f));
				//A(3*cont,1) = 0;
				//A(3*cont,2) = 0;
				//A(3*cont,3) = 0;
				coord.push_back(Triplet<float>(3*cont,4, zz_inter(z,y)));
				coord.push_back(Triplet<float>(3*cont,5, -yy_inter(z,y)));
				coord.push_back(Triplet<float>(3*cont,6+2*cont, elem1));
				coord.push_back(Triplet<float>(3*cont,7+2*cont, elem2));

				//2.Second equation
				//A(3*cont+1,0) = 0;
				coord.push_back(Triplet<float>(3*cont+1,1, 1.0));
				//A(3*cont+1,2) = 0;
				coord.push_back(Triplet<float>(3*cont+1,3, -zz_inter(z,y)));
				//A(3*cont+1,4) = 0;
				coord.push_back(Triplet<float>(3*cont+1,5, depth_inter(z,y)));
				coord.push_back(Triplet<float>(3*cont+1,6+2*cont, elem1*tanphi));
				coord.push_back(Triplet<float>(3*cont+1,7+2*cont, elem2*tanphi + depth_inter(z,y)*inv_cosphi*inv_cosphi));

				//3.Third equation
				//A(3*cont+2,0) = 0;
				//A(3*cont+2,1) = 0;
				coord.push_back(Triplet<float>(3*cont+2,2, 1.0));
				coord.push_back(Triplet<float>(3*cont+2,3, yy_inter(z,y)));
				coord.push_back(Triplet<float>(3*cont+2,4, -depth_inter(z,y)));
				//A(3*cont+2,5) = 0;
				coord.push_back(Triplet<float>(3*cont+2,6+2*cont, elem1*tantita*inv_cosphi + depth_inter(z,y)*inv_costita*inv_costita*inv_cosphi));
				coord.push_back(Triplet<float>(3*cont+2,7+2*cont, elem2*tantita*inv_cosphi + depth_inter(z,y)*tantita*tanphi*inv_cosphi));

				B(3*cont,0) = -dt(z,y);
				B(3*cont+1,0) = -dt(z,y)*tanphi;
				B(3*cont+2,0) = -dt(z,y)*tantita*inv_cosphi;

				cont++;
			}
		}
	}

	//cout << endl << "Tiempo para crear todos los triplets: " << 1000*clock.Tac() << "ms";
	//clock.Tic();

	A.setFromTriplets(coord.begin(), coord.end());

	//cout << endl << "Tiempo para construir la matriz con los triplets: " << 1000*clock.Tac() << "ms";
	//clock.Tic();
	
	//Solve the linear system of equations using a minimum least squares method

	//clock.Tic();
	SparseMatrix<float> aux = A.transpose()*A;
	//cout << endl << "Tiempo para transponer y multiplicar: " << 1000*clock.Tac() << "ms";

	//SimplicialCholesky<SparseMatrix<float>> SparseCholesky;
	SimplicialLDLT<SparseMatrix<float>>	SparseCholesky;
	//SimplicialLLT<SparseMatrix<float>>	SparseCholesky;

	//clock.Tic();

	SparseCholesky.compute(aux);

	////cout << endl << "Tiempo para hacer la factorización: " << 1000*clock.Tac() << "ms";

	////clock.Tic();

	if(SparseCholesky.info()!= 1 )
	{
		cout << endl << "Fallo en la descomposicion";
	}

	Var = SparseCholesky.solve(A.transpose()*B);
	if(SparseCholesky.info()!= 1)
	{
		cout << endl << "Fallo en la resolucion";
	}

	//cout << endl << "Tiempo para resolver el sistema: " << 1000*clock.Tac() << "ms";


	cont = 0;
	for (int z = 1; z < rows-1; z++)
	{
		for (int y = 1; y < cols-1; y++)
		{
			if ((borders(z,y) == 0)&&(Nan(z,y) == 0))
			{
				dtita(z,y) = Var(6+2*cont,0);
				dphi(z,y) = Var(7+2*cont,0);
				cont++;
			}
			else
			{
				dtita(z,y) = 0.0;
				dphi(z,y) = 0.0;
			}
		}
	}
}


CIcpOdo::CIcpOdo()
{
	cloud_old = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
	cloud_new = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
	cloud_trans = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
	
	//ICP options
	//icp.setRANSACIterations(5);
	//icp.setRANSACOutlierRejectionThreshold();
	gicp.setMaxCorrespondenceDistance (0.5);
    // Set the maximum number of iterations (criterion 1)
    gicp.setMaximumIterations (300);
    // Set the transformation epsilon (criterion 2)
    gicp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    gicp.setEuclideanFitnessEpsilon (0.01);
}


void CIcpOdo::solveSystemGeneralized()
{
	
    // Perform the alignment
    gicp.align (*cloud_trans);
     
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = gicp.getFinalTransformation();

	CMatrixDouble33 rot_matrix;
	for (unsigned int i=0; i<3; i++)
		for (unsigned int j=0; j<3; j++)
			rot_matrix(i,j) = transformation(i,j);

	estimated_pose.setRotationMatrix(rot_matrix);
	estimated_pose.x(transformation(0,3));
	estimated_pose.y(transformation(1,3));
	estimated_pose.z(transformation(2,3));

	cout << endl << "ICP has converged: " << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << endl;
}



