#include <tuple>
#include <Eigen/Core>
#include <Eigen/LU>

class KalmanFilter {

	int n;  // number of states
	int m;  // number of controls
	int k;  // number of measurements

	Eigen::MatrixXf At;
	Eigen::MatrixXf Bt;
	Eigen::MatrixXf Ct;
	Eigen::MatrixXf Rt;
	Eigen::MatrixXf Qt;

	Eigen::VectorXf predictMean(Eigen::VectorXf&, Eigen::VectorXf&);
	Eigen::MatrixXf predictCov(Eigen::MatrixXf&);
	Eigen::MatrixXf calKalmanFilter(Eigen::MatrixXf&);
	Eigen::VectorXf updateMean(Eigen::VectorXf&, Eigen::VectorXf&, Eigen::MatrixXf&);
	Eigen::MatrixXf updateCovt(Eigen::MatrixXf&, Eigen::MatrixXf&);

public:
	KalmanFilter(int, int, int, Eigen::MatrixXf&, Eigen::MatrixXf&, Eigen::MatrixXf&, Eigen::MatrixXf&, Eigen::MatrixXf&); 
	std::tuple<Eigen::VectorXf, Eigen::MatrixXf> 
		algorithm(Eigen::MatrixXf&, Eigen::VectorXf&, Eigen::VectorXf&, Eigen::VectorXf&, int);	
};

