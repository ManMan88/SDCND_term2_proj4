#include <iostream>
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

void PID::Init(double Kp_, double Kd_, double Ki_) {
	//set the PID hyper parameters
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	//initialize error values
	p_error = 0;
	i_error = 0;
	d_error = 0;

	return;
}

void PID::UpdateError(const double cte, const double dt) {
	// compute updated errors
	d_error = (cte - p_error)/dt;
	i_error += cte*dt;
	p_error = cte;

	//cout << "p_error\t" << p_error << "\td_error\t" << d_error << "\ti_error" << i_error << endl;
}

double PID::TotalError(const double saturation_max, const double saturation_min) {
	// compute control signal

	//cout << "p_signal\t" << -Kp*p_error << "\td_signal\t" << - Kd*d_error << "\ti_signal" << - Ki* i_error << endl;

	double control_signal = -Kp*p_error - Kd*d_error - Ki* i_error;
	// avoid exceeding saturation
	if (control_signal > saturation_max)
		control_signal = saturation_max;
	else if (control_signal < saturation_min)
		control_signal = saturation_min;

	return control_signal;
}


