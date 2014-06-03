
#include <mrpt/base.h>
#include <mrpt/math.h>
#include <mrpt/utils.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/slam.h>
#include <mrpt/maps.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/fast_bilateral.h>
#include <opencv/highgui.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace std;
using namespace pcl;
using namespace Eigen;

class CDifOdo {
public:

	SparseMatrix<float> A;
	MatrixXf Var;
	MatrixXf B;
	MatrixXf dyp;
	MatrixXf dzp;
	MatrixXf dt;
	MatrixXf depth;
	MatrixXf depth_old;
	MatrixXf depth_inter;
	MatrixXf depth_ft;
	MatrixXf yy;
	MatrixXf yy_ft;
	MatrixXf yy_inter;
	MatrixXf yy_old;
	MatrixXf zz;
	MatrixXf zz_ft;
	MatrixXf zz_inter;
	MatrixXf zz_old;
	MatrixXf tita;
	MatrixXf phi;
	MatrixXf borders;
	MatrixXf Nan;
	MatrixXf dtita;
	MatrixXf dphi;

	float f_dist;		//In meters
	float y_incr;		//In meters
	float z_incr;		//In meters
	float t_incr_inv;	//In seconds
	float dt_threshold;
	float dyz_threshold;

	float fovh;
	float fovv;
	int rows;
	int cols;
	int num_border_points;
	int num_nan_points;
	int num_border_and_nan;

	CSimplePointsMap	points_borders;
	CSimplePointsMap	points_nan;

	PointCloud<PointXYZ>::Ptr	entering_cloud;
	PointCloud<PointXYZ>		filtered_cloud;
	pcl::FastBilateralFilter<PointXYZ>	filter;

	void calculateCoord();
	void calculateDyp();
	void calculateDyp2t();
	void calculateDyp3t();
	void calculateDzp();
	void calculateDzp2t();
	void calculateDzp3t();
	void calculateDt();
	void calculateDt9p();
	void filterPointsAndUpdate();
	void findBorders();
	void findNaNPoints();
	void solveSystemSparse();
	void getPointsInBorders();
	void countPointsInBordersAndNaN();
	CDifOdo();
};

class CIcpOdo {
public:
	
	PointCloud<PointXYZ>::Ptr	cloud_old;
	PointCloud<PointXYZ>::Ptr	cloud_new;
	PointCloud<PointXYZ>::Ptr	cloud_trans;
	CPose3D						estimated_pose;

	GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;

	void solveSystemGeneralized();
	CIcpOdo();
};


