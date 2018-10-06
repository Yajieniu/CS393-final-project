#include <tuple>
#include <Eigen>
#include "KalmanFilter.h"

using namespace Eigen;

class KalmanFilter {

	int n;  // number of states
	int m;  // number of controls
	int k;  // number of measurements

	MatrixXf At;
	MatrixXf Bt;
	MatrixXf Ct;
	MatrixXf Rt;
	MatrixXf Qt;

	VectorXf predictMean(VectorXf&, VectorXf&);
	MatrixXf predictCov(MatrixXf&);
	MatrixXf calKalmanFilter(MatrixXf&);
	VectorXf updateMean(VectorXf&, VectorXf&, MatrixXf&);
	MatrixXf updateCovt(MatrixXf&, MatrixXf&);

public:
	KalmanFilter(int, int, int, MatrixXf&, MatrixXf&, MatrixXf&, MatrixXf&, MatrixXf&); 
	tuple<VectorXf, MatrixXf> algorithm(MatrixXf&, VectorXf&, VectorXf&, VectorXf&, int);	
};

