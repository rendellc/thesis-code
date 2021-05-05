#include "control/PID.hpp"

#include <rclcpp/time.hpp>

PID::PID(double P, double I, double D) : P(P), I(I), D(D) {}

double PID::update(double error, rclcpp::Time time) {
  constexpr int TEN_TO_THE_NINE = 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10;

  if (ready) {
    const double time_step =
        static_cast<double>((time - time_previous).nanoseconds()) /
        TEN_TO_THE_NINE;
    const double error_rate = (error - error_previous) / time_step;
    command = P * error + I * error_integral + D * error_rate;
    error_integral = error_integral + error_integral * time_step;
  }

  time_previous = time;
  error_previous = error;
  ready = true;
  return command;
}

double PID::update(double error, double error_rate, rclcpp::Time time) {
  constexpr int TEN_TO_THE_NINE = 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10;

  if (ready) {
    const double time_step =
        static_cast<double>((time - time_previous).nanoseconds()) /
        TEN_TO_THE_NINE;
    command = P * error + I * error_integral + D * error_rate;
    error_integral = error_integral + error_integral * time_step;
  }

  time_previous = time;
  error_previous = error;
  ready = true;
  return command;
}
