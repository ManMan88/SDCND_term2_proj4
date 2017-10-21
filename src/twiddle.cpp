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


void Twiddle::StateMachine(const double error) {

  switch(state) {
    case init_t:
      state = start_t;
      break;

    case start_t:
      best_error = error;
      K_test.Kp = K_params.Kp + K_modify.Kp;
      state = increased_Kp;
      break;

    case increased_Kp:
      if (error < best_error) {
        best_error = error;
        K_modify.Kp *= 1.1;
        K_params.Kp = K_test.Kp;

        K_test.Kd = K_params.Kd + K_modify.Kd;
        state = increased_Kd;
      }
      else {
        K_test.Kp = K_params.Kp - K_modify.Kp;
        state = decreased_Kp;
      }
      break;

    case decreased_Kp:
      if (error < best_error) {
        best_error = error;
        K_modify.Kp *= 1.1;
        K_params.Kp = K_test.Kp;
      }
      else {
        K_modify.Kp *= 0.9;
      }

      K_test.Kd = K_params.Kd + K_modify.Kd;
      state = increased_Kd;
      break;
  }

}


