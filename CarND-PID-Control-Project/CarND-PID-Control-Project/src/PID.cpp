#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  d_error = 0.0;
  p_error = 0.0;
  i_error = 0.0;
}

void PID::UpdateParameter(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  // differential error = current error - previous error
  d_error = cte - p_error;
  // proportional error = now the current error
  p_error = cte;
  // integral error = add current term
  i_error += cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double TError = (-(Kp * p_error) - (Ki * i_error) - (Kd * d_error));
  if(TError > 1){
    TError = 1;
  }
  if(TError < -1){
    TError = -1;
  }
  return TError;  // TODO: Add your total error calc here!
}

