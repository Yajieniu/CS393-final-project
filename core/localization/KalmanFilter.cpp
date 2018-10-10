#include "KalmanFilter.h"
#include <iostream> 
#include <assert.h>

using namespace std;
using namespace Eigen;

/* 
* Constructor
* n: dimension of state vector
* m: dimension of control vector
* k: dimension of meansurement vector
*/
// KalmanFilter::KalmanFilter(Matrixnnf &At, Matrixnmf &Bt, Matrixknf &Ct, 
// 	Matrixnnf &Rt, Matrixnnf &Qt) {

KalmanFilter::KalmanFilter() {

  n = kalman_n;
  m = kalman_m;
  k = kalman_k;

  // this->At = At;
  // this->Bt = Bt;
  // this->Ct = Ct;
  // this->Rt = Rt;
  // this->Qt = Qt;

  At = MatrixXf::Identity(n, n);
  Bt = MatrixXf::Zero(n, m);
  Ct = MatrixXf::Zero(k, n);
  Rt = MatrixXf::Identity(n, n);
  Qt = MatrixXf::Identity(k, k);

  // Might need to change initial belief
  wt = Vectornf::Zero(n);
  Covt = MatrixXf::Identity(n, n);

  initialized = false;

}

int KalmanFilter::get_n() {return n;}
int KalmanFilter::get_m() {return m;}
int KalmanFilter::get_k() {return k;}
bool KalmanFilter::isInitialized() {return initialized;}
void KalmanFilter::setwt(Vectornf &wt) {
	initialized = true;
	this->wt = wt;
}

void KalmanFilter::setCovt(Matrixnnf& Covt) {
	initialized = true;
	this->Covt = Covt;
}

void KalmanFilter::setConstants(Matrixnnf &At, Matrixnmf &Bt, Matrixknf &Ct, 
 	Matrixnnf &Rt, Matrixnnf &Qt) {
	
	this->At = Matrixnnf(At);
	this->Bt = Matrixnmf(Bt);
	this->Ct = Matrixknf(Ct);
	this->Rt = Matrixnnf(Rt);
	this->Qt = Matrixnnf(Qt);
}

void KalmanFilter::updateCovt(float ratio) {
	this->Covt = this->Covt * ratio;
}

void KalmanFilter::reset() {
	this->initialized = false;
}

std::tuple<KalmanFilter::Vectornf, KalmanFilter::Matrixnnf> KalmanFilter::algorithm
	(Vectormf& ut, Vectorkf& zt) {

	static string sep = "\n----------------------------------------\n";

	// std::cout << "At: " << At << sep;

	// std::cout << "old wt: \n" << wt << sep;

	// predict state
	Vectornf predictedWt = At * wt + Bt * ut; 

	// cout << "predictedWt: \n" << predictedWt << sep;
	// wt = predictedWt;

	// predict covariance
	Matrixnnf predictedCovt = At * Covt * At.transpose() + Rt;

	// cout << "predictedCovt: \n" << predictedCovt << sep;

	// calcualte Kalman gain
	Matrixnnf Kt = predictedCovt * Ct.transpose() * 
				(Ct * predictedCovt * Ct.transpose() + Qt).inverse();

	// assert(predictedCovt * Ct.transpose() * 
	// 		(Ct * predictedCovt * Ct.transpose() + Qt).isInvertible());

	// cout << "Kt: \n" << Kt << sep;
	// update state
	this->wt = predictedWt + Kt * (zt - Ct * predictedWt);

	// cout << "new wt: \n" << wt << sep;
	
	// update covariance
	MatrixXf I = MatrixXf::Identity(n,n);
	this->Covt = (I - Kt * Ct) * Covt; //
	this->Covt = (this->Covt + this->Covt.transpose()) / 2;
	// * (I - Kt * Ct) + Kt * Rt * Kt.transpose();

	// cout << "new Covt: \n" << this->Covt << endl;

	// LLT<Matrixnnf> lltOfCovt(Covt);

	// if (lltOfCovt.info() == Eigen::NumericalIssue) {
	// 	cout << "Covt not PSD" << endl;
	// };

	return std::make_tuple(predictedWt, Covt);

}

// void KalmanFilter::increaseCovt()

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


