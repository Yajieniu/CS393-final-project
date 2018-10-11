#include "ExtendedKalmanFilter.h"
#include <iostream> 

using namespace std;
using namespace Eigen;

/* 
* Constructor
* n: dimension of state vector
* m: dimension of control vector
* k: dimension of meansurement vector
*/

ExtendedKalmanFilter::ExtendedKalmanFilter() {

  n = extended_kalman_n;
  m = extended_kalman_m;
  k = extended_kalman_k;

  initialized = false;

}

int ExtendedKalmanFilter::get_n() {return n;}
int ExtendedKalmanFilter::get_m() {return m;}
int ExtendedKalmanFilter::get_k() {return k;}
bool ExtendedKalmanFilter::isInitialized() {return initialized;}

void ExtendedKalmanFilter::reset() {
	this->initialized = false;
}

void ExtendedKalmanFilter::setwt(Vectornf &wt) {
	this->wt = wt;
}

void ExtendedKalmanFilter::setCovt(Matrixnnf& Covt) {
	initialized = true;
	this->Covt = Covt;
}

void ExtendedKalmanFilter::setConstants(Matrixnnf &At, Matrixnmf &Bt, Matrixknf &Ct, 
 	Matrixnnf &Rt, Matrixnnf &Qt) {
	
	this->At = Matrixnnf(At);
	this->Bt = Matrixnmf(Bt);
	this->Ct = Matrixknf(Ct);
	this->Rt = Matrixnnf(Rt);
	this->Qt = Matrixnnf(Qt);
}

std::tuple<ExtendedKalmanFilter::Vectornf, ExtendedKalmanFilter::Matrixnnf> ExtendedKalmanFilter::algorithm
	(Vectormf& ut, Vectorkf& zt) {

	// predict state
	auto predictedWt = g(wt, ut);

	// predict covariance
	auto Gt = dg(wt, ut);
	auto predictedCovt = Gt * Covt * Gt.transpose() + Rt;

	// calcualte Kalman gain
	auto Ht = dh(predictedWt);

	auto Kt = predictedCovt * Ht.transpose() * 
				(Ht * predictedCovt * Ht.transpose() + Qt).inverse();

	// update state
	this->wt = predictedWt + Kt * (zt - h(predictedWt));

	// update covariance
	auto I = MatrixXf::Identity(n,n);
	this->Covt = (I - Kt * Ht) * Covt;

	return std::make_tuple(predictedWt, Covt);

}

ExtendedKalmanFilter::Vectornf ExtendedKalmanFilter::g ( Vectornf& wt, Vectormf& ut ) {
	return (At * wt + Bt * ut);
}

ExtendedKalmanFilter::Matrixnnf ExtendedKalmanFilter::dg ( Vectornf& wt, Vectormf& ut ) {
	return At;
}

ExtendedKalmanFilter::Vectorkf ExtendedKalmanFilter::h ( Vectornf& wt ) {
	return (Ct * wt);
}

ExtendedKalmanFilter::Matrixknf ExtendedKalmanFilter::dh ( Vectornf& wt ) {
	return Ct;
}
