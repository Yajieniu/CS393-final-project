#include <tuple>
#include <Eigen/Core>
#include <Eigen/LU>

#define extended_kalman_n 4 // number of states
#define extended_kalman_m 1 // number of controls // 0 may not work
#define extended_kalman_k 4 // number of measurements

class ExtendedKalmanFilter {

public:
	typedef Eigen::Matrix<float, extended_kalman_n, extended_kalman_n> Matrixnnf;
	typedef Eigen::Matrix<float, extended_kalman_n, extended_kalman_m> Matrixnmf;
	typedef Eigen::Matrix<float, extended_kalman_k, extended_kalman_n> Matrixknf;
	typedef Eigen::Matrix<float, extended_kalman_k, extended_kalman_k> Matrixkkf;

	typedef Eigen::Matrix<float, extended_kalman_n, 1>  Vectornf;
	typedef Eigen::Matrix<float, extended_kalman_m, 1> Vectormf;
	typedef Eigen::Matrix<float, extended_kalman_k, 1> Vectorkf;

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

public:
	ExtendedKalmanFilter();
	int get_n();
	int get_m();
	int get_k();

	ExtendedKalmanFilter::Vectornf g (Vectornf&, Vectormf&);
	ExtendedKalmanFilter::Matrixnnf dg (Vectornf&, Vectormf&);
	ExtendedKalmanFilter::Vectorkf h (Vectornf&);
	ExtendedKalmanFilter::Matrixknf dh (Vectornf&);

	bool isInitialized();
	void reset();
	void setwt(Vectornf&);
	void setCovt(Matrixnnf&);
	void setConstants(Matrixnnf&, Matrixnmf&, Matrixknf&, Matrixnnf&, Matrixnnf&);
	
	std::tuple<ExtendedKalmanFilter::Vectornf, ExtendedKalmanFilter::Matrixnnf> algorithm(Vectormf&, Vectorkf&);	
};

