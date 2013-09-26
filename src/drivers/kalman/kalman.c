#include "kalman.h"

void kalman_state_predict(KF *kf){
	kf->x = kf->X;
}

void kalman_covariance_predict(KF *kf){
	kf->p = kf->P + kf->Q;	
}

void kalman_state_update(KF *kf, double Z){
	kf->X = kf->x + (kf->K * (Z - kf->x));
}

void kalman_covariance_update(KF *kf){
	kf->P = (1 - kf->K) * kf->p;
}

void kalman_gain(KF *kf){
	kf->K = kf->p / (kf->p + kf->R); 
}
