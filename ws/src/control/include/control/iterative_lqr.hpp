#pragma once

#include <control/dynamics/dynamical_system.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

class IterativeLQR {
 private:
  std::shared_ptr<control::dynamics::DynamicalSystem> dynsys_p;
  const double stepsize;

  Eigen::MatrixXd Q;
  Eigen::MatrixXd Qf;
  Eigen::MatrixXd R;
  Eigen::MatrixXd Rinv;

  std::vector<Eigen::VectorXd> state_sequence;
  std::vector<Eigen::VectorXd> state_change_sequence;
  std::vector<Eigen::VectorXd> input_sequence;
  std::vector<Eigen::VectorXd> input_change_sequence;
  std::vector<Eigen::MatrixXd> A;
  std::vector<Eigen::MatrixXd> B;
  std::vector<Eigen::MatrixXd> S;
  std::vector<Eigen::VectorXd> v;
  std::vector<Eigen::MatrixXd> K;
  std::vector<Eigen::MatrixXd> Kv;
  std::vector<Eigen::MatrixXd> Ku;

 public:
  IterativeLQR() = delete;
  IterativeLQR(std::shared_ptr<control::dynamics::DynamicalSystem> dynsys_p,
               const Eigen::VectorXd& cost_states,
               const Eigen::VectorXd& cost_states_final,
               const Eigen::VectorXd& cost_inputs,
               const std::vector<Eigen::VectorXd>& input_sequence,
               double stepsize);

  Eigen::VectorXd update(const Eigen::VectorXd& state,
                         const Eigen::VectorXd& state_target);
};
