#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double sum_of_squared_errors;  // will be a moving average to prevent wind-up
  double prev_cte;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Unfinished, "hide" the implementation and lots of global variable
  * of the online "Twiddle" from the main. see writeup
  */
  void Twiddlestep(double *p, double *dp, double &best_err, double err);

};

#endif /* PID_H */
