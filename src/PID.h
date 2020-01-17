#ifndef PID_H
#define PID_H

#include <list>

class PID {
public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void control_value(double &steer);

private:
  /**
   * PID Errors
   */
  std::list<double> cte_;
  double diff_cte_;
  double total_cte_;

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;
};

#endif // PID_H