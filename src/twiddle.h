#ifndef TWIDDLE_H
#define TWIDDLE_H

class Twiddle {
public:

  // Best found error
  double best_error;

  // control parameters
  struct control_parameters {
	  double Kp;
	  double Kd;
	  double Ki;
  };

  // best found control parameters
  control_parameters K_params;

  // tested control parameters
  control_parameters K_test;

  // twiddle modification values
  control_parameters K_modify;


  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize twiddle.
  */
  void Init(double Kp_, double Kp_mod, double Kd_, double Kd_mod, double Ki_, double Ki_mod);

  /*
  * The state machine determines what the twiddle will do next.
  */
  void StateMachine();


private:
  //State ENUM
  enum State_Options {init, start, increased_Kp, decreased_Kp, increased_Kd, decreased_Kd, increased_Ki, decreased_Ki};
  State_Options state;
};

#endif /* TWIDDLE_H */
