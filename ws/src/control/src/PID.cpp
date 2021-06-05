#include "control/PID.hpp"

#include <rclcpp/time.hpp>

PID::PID(double P, double I, double D) : P(P), I(I), D(D) {}

double PID::update(double error, rclcpp::Time time) {
  if (ready) {
    const double time_step = (time - time_previous).seconds();
    error_integral = error_integral + error * time_step;
    const double error_rate = (error - error_previous) / time_step;

    command = P * error + I * error_integral + D * error_rate;
  }

  time_previous = time;
  error_previous = error;
  ready = true;
  return command;
}

double PID::update(double error, double error_rate, rclcpp::Time time) {
  // constexpr int TEN_TO_THE_NINE = 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10;

  if (ready) {
    const double time_step = (time - time_previous).seconds();
    error_integral = error_integral + error * time_step;
    command = P * error + I * error_integral + D * error_rate;
  }

  time_previous = time;
  error_previous = error;
  ready = true;
  return command;
}
