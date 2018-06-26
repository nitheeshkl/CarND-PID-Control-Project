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

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  // number of hyper params used
  const static unsigned short num_params = 3;
  double *p; // array of params
  double *dp; // twiddle params
  double best_cte; // to store best cte for twiddle
  double prev_cte; // stores previous cte used to calculate diff error
  unsigned short index; // points to the current hyper param in twiddle
  constexpr static double tol = 0.1; // tol used to check sum of dp in twiddle
  unsigned short twiddle_step = 1; // stores the next step to process in twiddle
  double total_cte = 0;
  long step = 0;

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
};

#endif /* PID_H */
