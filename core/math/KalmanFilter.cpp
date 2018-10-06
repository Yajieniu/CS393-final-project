#include "KalmanFilter.h"

/* 
* Constructor
* n: dimension of state vector
* m: dimension of control vector
* k: dimension of meansurement vector
*/
KalmanFilter::KalmanFilter(int nn, int mm, int kk, Eigen::MatrixXf& A, Eigen::MatrixXf& B,
			 Eigen::MatrixXf& C, Eigen::MatrixXf& R, Eigen::MatrixXf& Q) {
	n = nn;
	m = mm;
	k = kk;

	At.resize(n, n);
	Bt.resize(n, m);
	Ct.resize(k, n);
	Rt.resize(n, n);
	Qt.resize(k, k);

	At = A;
	Bt = B;
	Ct = C;
	Rt = R;
	Qt = Q;

}

Eigen::VectorXf KalmanFilter::predictMean(Eigen::VectorXf& lastw, Eigen::VectorXf& ut) {

	return At * lastw + Bt * ut;
}


Eigen::MatrixXf KalmanFilter::predictCov(Eigen::MatrixXf& lastCov) {

	return At * lastCov * At.transpose() + Rt;
}


Eigen::MatrixXf KalmanFilter::calKalmanFilter(Eigen::MatrixXf& predictedCovt) {

	return predictedCovt * Ct.transpose() * (Ct * predictedCovt * Ct.transpose() + Qt).inverse();
}


Eigen::VectorXf KalmanFilter::updateMean(Eigen::VectorXf& predictedWt, Eigen::VectorXf& zt,
						   Eigen::MatrixXf& Kt) {

	return predictedWt + Kt * (zt - Ct * predictedWt);
}


Eigen::MatrixXf KalmanFilter::updateCovt(Eigen::MatrixXf& Kt, Eigen::MatrixXf& lastCov) {
	
	Eigen::MatrixXf I = Eigen::MatrixXf::Identity(n,n);

	return (I - Kt * Ct) * lastCov;
}
 
std::tuple<Eigen::VectorXf, Eigen::MatrixXf> KalmanFilter::algorithm(Eigen::MatrixXf& lastCov, 
	Eigen::VectorXf& lastw, Eigen::VectorXf& ut, Eigen::VectorXf& zt, int t) {

	Eigen::VectorXf predictedWt = predictMean(lastw, ut);
	Eigen::MatrixXf predictedCovt = predictCov(lastCov);

	Eigen::MatrixXf Kt = calKalmanFilter(predictedCovt);
	Eigen::MatrixXf Covt = updateCovt(Kt, lastCov);
	Eigen::VectorXf wt = updateMean(predictedWt, zt, Kt);

	return std::make_tuple(wt, Covt);
}