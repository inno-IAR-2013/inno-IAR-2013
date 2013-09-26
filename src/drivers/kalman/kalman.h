#ifndef __KALMAN_H
#define __KALMAN_H
//#include "vector.h"
//#include "uart.h"

struct kalman_filter{
	double Q, R, K, X, x, P, p;
};
typedef struct kalman_filter KF;
								  
void kalman_state_predict(KF *kf);
void kalman_covariance_predict(KF *kf);
void kalman_state_update(KF *kf, double Z);
void kalman_covariance_update(KF *kf);
void kalman_gain(KF *kf);

#endif
