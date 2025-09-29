/**
 * @author Nelson Durrant
 * @date September 2024
 *
 * This class is a simple implementation of a PID controller based on the BYU
 * ECEn 483 approach.
 *
 * It uses the integral on/off anti-windup strategy.
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
   * @param integral The integral constant.
   * @param d The derivative constant.
   * @param min The minimum output value.
   * @param max The maximum output value.
   * @param interval The interval at which the PID controller is called.
   */
  void initialize(float KP, float KI, float KD, float min, float max,
                  float interval, float scalar = 1.0) {
    this->kp = KP;
    this->ki = KI;
    this->kd = KD;
    this->min_output = min;
    this->max_output = max;
    this->pid_interval = interval;

    this->x_dot = 0.0;    // estimated derivative of x
    this->x_d1 = 0.0;     // x delayed by one sample
    this->error_d1 = 0.0; // error delayed by one sample
    integral = 0.0;       // integral

    float sigma = 0.05; // cutoff freq for dirty derivative
    this->beta = (2.0 * sigma - interval) / (2.0 * sigma + interval);
    multiplier = scalar;
  }

  float getKp() const { return kp; }

  float getKi() const { return ki; }

  float getKd() const { return kd; }

  float getP() const { return p; }

  float getI() const { return i; }

  float getD() const { return d; }

  float getPID() const { return pid; }

  float getXDot() const { return x_dot; }

  void reset_int() { integral = 0.0; }

  void print_values() {
    std::cout << " Kp: " << kp << " Ki: " << ki << " Kd: " << kd << std::endl;
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

    // integrate error in x with anti-windup
    if (std::abs(this->x_dot) < 0.08) {
      integral =
          integral + (this->pid_interval / 2.0) * (error + this->error_d1);
    }

    // std::cout << "i " << integral << std::endl;

    // differentiate x
    this->x_dot = this->beta * this->x_dot +
                  (1.0 - this->beta) * ((x - this->x_d1) / this->pid_interval);

    // std::cout << "x_dot " << this->x_dot << std::endl;

    // calculate the force
    p = this->kp * error;
    i = this->ki * integral;
    d = this->kd * this->x_dot;
    pid = p + i - d;
    float force_unsat = pid;

    // saturate the force
    float force_sat;
    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
    } else {
      force_sat = force_unsat;
    }

    // update delayed variables
    this->error_d1 = error;
    this->x_d1 = x;

    return force_sat;
  }

  float compute(float x_r, float x, float x_dot_in) {

    this->x_dot = x_dot_in;
    float error = x_r - x;

    integral = integral + (this->pid_interval / 2.0) * (error + this->error_d1);

    p = this->kp * error;
    i = this->ki * integral;
    d = this->kd * this->x_dot;

    pid = p + i - d;

    // calculate the force
    float force_unsat = pid;

    // saturate the force
    float force_sat;
    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
    } else {
      force_sat = force_unsat;
    }

    // update delayed variables
    this->error_d1 = error;

    return force_sat;
  }

  float compute(float x_r, float x, float x_dot_in, float velocity) {
    this->x_dot = x_dot_in;
    float error = x_r - x;

    integral = integral + (this->pid_interval / 2.0) * (error + this->error_d1);

    // calculate the scalar from velocity squared
    float scalar = (velocity * velocity * multiplier);

    p = (this->kp * error) / scalar;
    i = ((this->ki * integral) / scalar);
    d = (this->kd * this->x_dot) / scalar;

    float pd = p - d;
    float force_unsat = pd + i;

    // saturate the force and integral error in x
    float force_sat;

    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
      if (pd > this->max_output) {
        integral = 0.0;
      } else {
        integral = ((this->max_output - pd) * scalar) / this->ki;
      }
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
      if (pd < this->min_output) {
        integral = 0.0;
      } else {
        integral = ((this->min_output - pd) * scalar) / this->ki;
      }
    } else {
      force_sat = force_unsat;
    }

    i = ((this->ki * integral) / scalar);
    pid = pd + i;
    // calculate the force

    // update delayed variables
    this->error_d1 = error;

    return force_sat;
  }

private:
  float p;
  float i;
  float d;
  float pid;

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
  float integral;
  float multiplier;
};