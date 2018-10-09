#include "KalmanFilter.h"

using namespace Eigen;

/* 
* Constructor
* n: dimension of state vector
* m: dimension of control vector
* k: dimension of meansurement vector
*/
KalmanFilter::KalmanFilter() {
	
  n = kalman_n;
  m = kalman_m;
  k = kalman_k;

  Matrixnnf At = MatrixXf::Identity(n, n);
  Matrixnmf Bt = MatrixXf::Zero(n, m);
  Matrixknf Ct = MatrixXf::Zero(k, n);
  Matrixnnf Rt = MatrixXf::Identity(n, n);
  Matrixkkf Qt = MatrixXf::Identity(k, k);

}

int KalmanFilter::get_n() {return n;}
int KalmanFilter::get_m() {return m;}
int KalmanFilter::get_k() {return k;}

std::tuple<KalmanFilter::Vectornf, KalmanFilter::Matrixnnf> KalmanFilter::algorithm(Matrixnnf& lastCov, 
		Vectornf& lastw, Vectormf& ut, Vectorkf& zt) {

	// predict state
	Vectornf predictedWt = At * lastw + Bt * ut; 

	// predict covariance
	Matrixnnf predictedCovt = At * lastCov * At.transpose() + Rt;

	// calcualte Kalman gain
	Matrixnnf Kt = predictedCovt * Ct.transpose() * 
				(Ct * predictedCovt * Ct.transpose() + Qt).inverse();

	// update state
	Vectornf wt = predictedWt + Kt * (zt - Ct * predictedWt);
	
	// update covariance
	MatrixXf I = MatrixXf::Identity(n,n);
	Matrixnnf Covt = (I - Kt * Ct) * lastCov;

	return std::make_tuple(wt, Covt);

}


// std::tuple<VectorXf&, MatrixXf&> KalmanFilter::algorithm1(MatrixXf& lastCov, 
// 	VectorXf& lastw, VectorXf& ut, VectorXf& zt) {

// 	Vectornf& predictedWt = predictMean(lastw, ut);
// 	Matrixnnf& predictedCovt = predictCov(lastCov);

// 	Matrixnnf& Kt = calKalmanFilter(predictedCovt);
// 	Matrixnnf& Covt = updateCovt(Kt, lastCov);
// 	Vectornf& wt = updateMean(predictedWt, zt, Kt);

// 	return std::make_tuple(wt, Covt);
// }


// Vectornf& KalmanFilter::predictMean(VectorXf& lastw, VectorXf& ut) {

// 	return At * lastw + Bt * ut;
// }


// Matrixnnf& KalmanFilter::predictCov(MatrixXf& lastCov) {

// 	return At * lastCov * At.transpose() + Rt;
// }


// Matrixnnf& KalmanFilter::calKalmanFilter(MatrixXf& predictedCovt) {

// 	return predictedCovt * Ct.transpose() * 
// 		(Ct * predictedCovt * Ct.transpose() + Qt).inverse();
// }


// Vectornf& KalmanFilter::updateMean(VectorXf& predictedWt, VectorXf& zt,
// 						   MatrixXf& Kt) {

// 	return predictedWt + Kt * (zt - Ct * predictedWt);
// }


// Matrixnnf& KalmanFilter::updateCovt(MatrixXf& Kt, MatrixXf& lastCov) {
	
// 	MatrixXf I = MatrixXf::Identity(n,n);

// 	return (I - Kt * Ct) * lastCov;
// }


