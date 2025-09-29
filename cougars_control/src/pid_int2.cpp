/**
 * @author Nelson Durrant
 * @date September 2024
 *
 * This class is a simple implementation of a PID controller based on the BYU
 * ECEn 483 approach.
 *
 * It uses the integrator saturation anti-windup strategy.
 */

#include <iostream>

class PID {

public:
  /**
   * Creates a new PID controller.
   */
  PID() {};

  /**
   * This method initializes the PID controller with the given constants.
   *
   * @param p The proportional constant.
   * @param i The integral constant.
   * @param d The derivative constant.
   * @param min The minimum output value.
   * @param max The maximum output value.
   * @param interval The interval at which the PID controller is called.
   */
  void initialize(float p, float i, float d, float min, float max,
                  float interval) {
    this->kp = p;
    this->ki = i;
    this->kd = d;
    this->min_output = min;
    this->max_output = max;
    this->pid_interval = interval;

    this->x_dot = 0.0;      // estimated derivative of x
    this->x_d1 = 0.0;       // x delayed by one sample
    this->error_d1 = 0.0;   // error delayed by one sample
    this->integrator = 0.0; // integrator

    float sigma = 0.05; // cutoff freq for dirty derivative
    this->beta = (2.0 * sigma - interval) / (2.0 * sigma + interval);
  }

  /**
   * Simple PID control, based on the approach in BYU ECEn 483.
   *
   * @param x_r The desired value.
   * @param x The actual value.
   * @return The output value.
   */
  float compute(float x_r, float x) {

    float error = x_r - x;

    // differentiate x
    this->x_dot = this->beta * this->x_dot +
                  (1.0 - this->beta) * ((x - this->x_d1) / this->pid_interval);

    // std::cout << "x_dot " << this->x_dot << std::endl;

    float pd = this->kp * error - this->kd * this->x_dot;

    // integrate error in x
    this->integrator = this->integrator + error;

    // std::cout << "integrator " << this->integrator << std::endl;

    // calculate the force
    float force_unsat = pd + this->ki * this->integrator;

    // saturate the force and integrator error in x
    float force_sat;

    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
      if (pd > this->max_output) {
        this->integrator = 0.0;
      } else {
        this->integrator = (this->max_output - pd) / this->ki;
      }
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
      if (pd < this->min_output) {
        this->integrator = 0.0;
      } else {
        this->integrator = this->max_output - pd;
      }
    } else {
      force_sat = force_unsat;
    }

    // calculate the adjusted force
    float pid = pd + this->ki * this->integrator;
    // calculate the force
    // std::cout << "PD: " << pd << "PID: " << pid << std::endl;

    // update delayed variables
    this->error_d1 = error;
    this->x_d1 = x;

    return force_sat;
  }

private:
  float kp;
  float ki;
  float kd;
  float min_output;
  float max_output;
  float pid_interval;

  float beta;
  float x_dot;
  float x_d1;
  float error_d1;
  float integrator;
};
