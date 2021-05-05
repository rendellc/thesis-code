#pragma once

#include <rclcpp/time.hpp>

class PID {
 public:
  PID() = default;
  PID(double P, double I, double D);

  double update(double error, rclcpp::Time time);
  double update(double error, double error_rate, rclcpp::Time time);

  double P = 0.0, I = 0.0, D = 0.0;

 private:
  bool ready = false;  // Not ready until data has been recieved
  double command = 0.0;
  double error_previous = 0.0, error_integral = 0.0;
  rclcpp::Time time_previous;
};