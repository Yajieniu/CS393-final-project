#include <tuple>
#include <Eigen/Core>
#include <Eigen/LU>

#define kalman_n 5 // number of states
#define kalman_m 5 // number of controls
#define kalman_k 5 // number of measurements

class KalmanFilter {

	int n;
	int m;
	int k;

	typedef Eigen::Matrix<float, kalman_n, kalman_n> Matrixnnf;
	typedef Eigen::Matrix<float, kalman_n, kalman_m> Matrixnmf;
	typedef Eigen::Matrix<float, kalman_k, kalman_n> Matrixknf;
	typedef Eigen::Matrix<float, kalman_k, kalman_k> Matrixkkf;

	typedef Eigen::Vector<kalman_n, float> Vectornf;
	typedef Eigen::Vector<kalman_m, float> Vectormf;
	typedef Eigen::Vector<kalman_k, float> Vectorkf;
	
	Matrixnnf At;
	Matrixnmf Bt;
	Matrixknf Ct;
	Matrixnnf Rt;
	Matrixkkf Qt;


	Matrixnnf lastCov;
	Vectornf lastw;
	Vectormf ut;
	Vectorkf zt;


	// Vectornf& predictMean(Eigen::VectorXf&, Eigen::VectorXf&);
	// Matrixnnf& predictCov(Eigen::MatrixXf&);
	// Matrixnnf& calKalmanFilter(Eigen::MatrixXf&);
	// Vectornf& updateMean(Eigen::VectorXf&, Eigen::VectorXf&, Eigen::MatrixXf&);
	// Matrixnnf& updateCovt(Eigen::MatrixXf&, Eigen::MatrixXf&);

public:
	KalmanFilter(); 
	std::tuple<Eigen::Vectornf&, Eigen::Matrixnnf&> 
		algorithm(Eigen::Matrixnnf&, Eigen::Vectornf&, Eigen::Vectormf&, Eigen::Vectorkf&, int);	
};

