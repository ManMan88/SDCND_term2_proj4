#include <iostream>
#include "twiddle.h"

using namespace std;

Twiddle::Twiddle() {
  best_error = 0;
}

Twiddle::~Twiddle() {}


void Twiddle::Init(double Kp_, double Kp_mod, double Kd_, double Kd_mod, double Ki_, double Ki_mod) {
  // set initial parameters
  K_params.Kp = Kp_;
  K_params.Kd = Kd_;
  K_params.Ki = Ki_;

  K_modify.Kp = Kp_mod;
  K_modify.Kd = Kd_mod;
  K_modify.Ki = Ki_mod;

  K_test = K_params;

  state = init_t;
  return;
}


void Twiddle::StateMachine() {

}


