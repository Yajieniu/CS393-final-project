#include "KalmanFilter.h"

using namespace Eigen;

/* 
* Constructor
* n: dimension of state vector
* m: dimension of control vector
* k: dimension of meansurement vector
*/
KalmanFilter::KalmanFilter(int nn, int mm, int kk, MatrixXf& A, MatrixXf& B,
			 MatrixXf& C, MatrixXf& R, MatrixXf& Q) {
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

VectorXf KalmanFilter::predictMean(VectorXf& lastw, VectorXf& ut) {

	return At * lastw + Bt * ut;
}


MatrixXf KalmanFilter::predictCov(MatrixXf& lastCov) {

	return At * lastCov * At.transpose() + Rt;
}


MatrixXf KalmanFilter::calKalmanFilter(MatrixXf& predictedCovt) {

	return predictedCovt * Ct.transpose() * 
		(Ct * predictedCovt * Ct.transpose() + Qt).inverse();
}


VectorXf KalmanFilter::updateMean(VectorXf& predictedWt, VectorXf& zt,
						   MatrixXf& Kt) {

	return predictedWt + Kt * (zt - Ct * predictedWt);
}


MatrixXf KalmanFilter::updateCovt(MatrixXf& Kt, MatrixXf& lastCov) {
	
	MatrixXf I = MatrixXf::Identity(n,n);

	return (I - Kt * Ct) * lastCov;
}
 
std::tuple<VectorXf, MatrixXf> KalmanFilter::algorithm(MatrixXf& lastCov, 
	VectorXf& lastw, VectorXf& ut, VectorXf& zt) {

	VectorXf predictedWt = predictMean(lastw, ut);
	MatrixXf predictedCovt = predictCov(lastCov);

	MatrixXf Kt = calKalmanFilter(predictedCovt);
	MatrixXf Covt = updateCovt(Kt, lastCov);
	VectorXf wt = updateMean(predictedWt, zt, Kt);

	return std::make_tuple(wt, Covt);
}
