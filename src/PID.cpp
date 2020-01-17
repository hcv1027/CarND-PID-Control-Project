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
  diff_cte_ = 0.0;
  total_cte_ = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  printf("Set Kp: %f, Ki: %f, Kd: %f\n", Kp, Ki, Kd);
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  cte_.clear();
  diff_cte_ = 0.0;
  total_cte_ = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  diff_cte_ = (!cte_.empty()) ? (cte - *(cte_.rbegin())) : cte;
  cte_.push_back(cte);
  total_cte_ += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // TODO: Add your total error calc here!
  double total_error = 0.0;
  int counter = 0;
  for (auto iter = cte_.rbegin(); iter != cte_.rend() /* && counter < 100 */;
       ++iter, ++counter) {
    if (fabs(*iter) > 1.0) {
      // total_error += std::pow(*iter, std::ceil(*iter));
      double error = std::pow(*iter, 2);
      if (fabs(*iter) > 3.0) {
        error *= 2.0;
      }
      total_error += error;
    } else {
      total_error += fabs(*iter);
    }
  }

  return total_error;
}

void PID::control_value(double &steer) {
  auto iter = cte_.rbegin();
  double cte = *(iter);
  double p_term = -Kp_ * cte;
  double d_term = -Kd_ * diff_cte_;
  double i_term = -Ki_ * total_cte_;
  steer = p_term + d_term + i_term;
  /* printf("steer: %10.5f, cte: %10.5f, diff_cte_: %10.5f, total_cte: %10.5f, "
         "pid : % 10.5f, % 10.5f, % 10.5f\n ",
         steer, cte, diff_cte_, total_cte_, p_term, d_term, i_term); */
  // printf("[%8.4f, %8.4f, %8.4f],\n ", p_term, d_term, i_term);
  // printf("[%8.4f, %8.4f, %8.4f],\n ", cte_0, diff_cte_, total_cte_);
}