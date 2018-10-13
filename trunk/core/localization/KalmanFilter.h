#include <tuple>
#include <Eigen/Core>
#include <Eigen/LU>
// #include <Eigne/FullPivLU>

#define kalman_n 4 // number of states
#define kalman_m 1 // number of controls // 0 may not work
#define kalman_k 4 // number of measurements

class KalmanFilter {

public:
	typedef Eigen::Matrix<float, kalman_n, kalman_n> Matrixnnf;
	typedef Eigen::Matrix<float, kalman_n, kalman_m> Matrixnmf;
	typedef Eigen::Matrix<float, kalman_k, kalman_n> Matrixknf;
	typedef Eigen::Matrix<float, kalman_k, kalman_k> Matrixkkf;

	typedef Eigen::Matrix<float, kalman_n, 1>  Vectornf;
	typedef Eigen::Matrix<float, kalman_m, 1> Vectormf;
	typedef Eigen::Matrix<float, kalman_k, 1> Vectorkf;

private:
	int n;
	int m;
	int k;
	
	Matrixnnf At;
	Matrixnmf Bt;
	Matrixknf Ct;
	Matrixnnf Rt;
	Matrixkkf Qt;

	Vectornf wt;
	Matrixnnf Covt;

	bool initialized;
	// Matrixnnf lastCov;
	// Vectornf lastw;
	// Vectormf ut;
	// Vectorkf zt;


	// Vectornf& predictMean(Eigen::VectorXf&, Eigen::VectorXf&);
	// Matrixnnf& predictCov(Eigen::MatrixXf&);
	// Matrixnnf& calKalmanFilter(Eigen::MatrixXf&);
	// Vectornf& updateMean(Eigen::VectorXf&, Eigen::VectorXf&, Eigen::MatrixXf&);
	// Matrixnnf& updateCovt(Eigen::MatrixXf&, Eigen::MatrixXf&);

public:
	// KalmanFilter(KalmanFilter::Matrixnnf&, KalmanFilter::Matrixnmf&, 
	// 	KalmanFilter::Matrixknf&, KalmanFilter::Matrixnnf&, KalmanFilter::Matrixnnf&);
	KalmanFilter();
	int get_n();
	int get_m();
	int get_k();
	bool isInitialized();
	void reset();
	void setwt(Vectornf&);
	void setCovt(Matrixnnf&);
	void setConstants(Matrixnnf&, Matrixnmf&, Matrixknf&, Matrixnnf&, Matrixnnf&);
	void updateCovt(float);
	std::tuple<KalmanFilter::Vectornf, KalmanFilter::Matrixnnf> algorithm(Vectormf&, Vectorkf&);	
};

