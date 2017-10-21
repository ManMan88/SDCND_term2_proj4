#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	//initialize error values
	p_error = 0;
	i_error = 0;
	d_error = 0;
	//initialize PID hyper parameters values
	Kp = 0;
	Ki = 0;
	Kd = 0;

	return;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	//set the PID hyper parameters
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	//initialize error values
	p_error = 0;
	i_error = 0;
	d_error = 0;

	return;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}


