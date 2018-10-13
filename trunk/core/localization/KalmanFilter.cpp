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

KalmanFilter::KalmanFilter() {

  n = kalman_n;
  m = kalman_m;
  k = kalman_k;

  // Might need to change initial belief
  // wt = Vectornf::Zero(n);
  // Covt = MatrixXf::Identity(n, n);

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

	// predict state
	Vectornf predictedWt = At * wt + Bt * ut; 

	// predict covariance
	Matrixnnf predictedCovt = At * Covt * At.transpose() + Rt;

	// calcualte Kalman gain
	Matrixnnf Kt = predictedCovt * Ct.transpose() * 
				(Ct * predictedCovt * Ct.transpose() + Qt).inverse();

	// assert(predictedCovt * Ct.transpose() * 
	// 		(Ct * predictedCovt * Ct.transpose() + Qt).isInvertible());

	// update state
	this->wt = predictedWt + Kt * (zt - Ct * predictedWt);

	// update covariance
	MatrixXf I = MatrixXf::Identity(n,n);
	this->Covt = (I - Kt * Ct) * Covt; 
	
	// LLT<Matrixnnf> lltOfCovt(Covt);

	// if (lltOfCovt.info() == Eigen::NumericalIssue) {
	// 	cout << "Covt not PSD" << endl;
	// };

	return std::make_tuple(predictedWt, Covt);

}


