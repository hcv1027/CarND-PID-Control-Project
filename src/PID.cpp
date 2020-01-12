#include "PID.h"
#include <cmath>
#include <limits>
#include <stdio.h>
#include <vector>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  Kp_ = 0.0;
  Ki_ = 0.0;
  Kd_ = 0.0;
  Kp2_ = 0.2;
  Ki2_ = 0.006;
  Kd2_ = 2.5;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  // cte_ = 0.0;
  diff_cte_ = 0.0;
  total_cte_ = 0.0;
  best_error_ = std::numeric_limits<double>::infinity();
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  diff_cte_ = cte - *(cte_.rbegin());
  cte_.push_back(cte);
  total_cte_ += cte;
  if (cte_.size() > 100) {
    cte_.pop_front();
    // twiddle();
  }
}

void PID::twiddle() {
  const double tolerance = 0.2;
  std::vector<double> params = {Kp_, Ki_, Kd_};
  std::vector<double> d_params = {0.2, 0.2, 0.2};
  double error = TotalError();
  if (error < best_error_) {
    best_error_ = error;
    for (int i = 0; i < d_params.size(); ++i) {
      d_params[i] *= 1.1;
    }
  } else {
    for (int i = 0; i < params.size(); ++i) {
      params[i] -= 2 * d_params[i];
    }
    // robot = make_robot()
    // x_trajectory, y_trajectory, err = run(robot, p)

    if (error < best_error_) {
      best_error_ = error;
      for (int i = 0; i < d_params.size(); ++i) {
        d_params[i] *= 1.1;
      }
    } else {
      for (int i = 0; i < params.size(); ++i) {
        params[i] += d_params[i];
        d_params[i] *= 0.9;
      }
    }
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // TODO: Add your total error calc here!
  while (cte_.size() > 100) {
    cte_.pop_front();
  }
  double total_error = 0.0;
  for (double cte : cte_) {
    total_error += cte * cte;
  }

  return total_error;
}

void PID::control_value(double &throttle, double &steer) {
  Kp_ = 0.2;
  Ki_ = 0.004;
  Kd_ = 3.0;
  double cte = *(cte_.rbegin());
  double p_term = -Kp_ * cte;
  double d_term = -Kd_ * diff_cte_;
  double i_term = -Ki_ * total_cte_;
  steer = p_term + d_term + i_term;
  /* printf("steer: %10.5f, cte: %10.5f, diff_cte_: %10.5f, total_cte: %10.5f, "
         "pid : % 10.5f, % 10.5f, % 10.5f\n ",
         steer, cte, diff_cte_, total_cte_, p_term, d_term, i_term); */

  Kp2_ = 0.3;
  Ki2_ = 0.01;
  Kd2_ = 3.0;
  // When the absolute of cte is large, we should decrease the throttle.
  double p_term2 = -Kp2_ * fabs(cte);
  // When the differential of cte is increasing, we should decrease the
  // throttle. Otherwise, we can increase the throttle.
  double d_term2 = -Kd2_ * diff_cte_;
  double i_term2 = -Ki2_ * fabs(total_cte_);
  double error_term = p_term2 + d_term2 + i_term2;
  throttle = std::min(0.8, std::max(0.1, 0.8 + p_term2 + d_term2 + i_term2));
  // throttle = error_term;
  printf("steer: %8.5f, throttle: %8.5f, cte: %8.5f, %8.5f, %8.5f,  "
         "pid : % 10.5f, % 10.5f, % 10.5f\n ",
         steer, throttle, cte, diff_cte_, total_cte_, p_term2, d_term2,
         i_term2);
}