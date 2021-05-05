#include <control/iterative_lqr.hpp>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>

IterativeLQR::IterativeLQR(std::shared_ptr<DynamicalSystem> dynsys_p,
                           const VectorXd& cost_states,
                           const VectorXd& cost_states_final,
                           const VectorXd& cost_inputs,
                           const std::vector<VectorXd>& input_sequence,
                           double stepsize)
    : dynsys_p(dynsys_p), stepsize(stepsize), input_sequence(input_sequence) {
  Q = cost_states.asDiagonal().toDenseMatrix();
  Qf = cost_states_final.asDiagonal().toDenseMatrix();
  R = cost_inputs.asDiagonal().toDenseMatrix();
  Rinv = cost_inputs.cwiseInverse().asDiagonal().toDenseMatrix();

  const int N = input_sequence.size();
  A.resize(N);
  B.resize(N);
  S.resize(N + 1);
  v.resize(N + 1);
  state_sequence.resize(N + 1);
  state_change_sequence.resize(N + 1);
  input_change_sequence.resize(N);
  K.resize(N);
  Kv.resize(N);
  Ku.resize(N);
}

VectorXd IterativeLQR::update(const VectorXd& state,
                              const VectorXd& state_target) {
  const int N = input_sequence.size();
  auto& xs = state_sequence;
  auto& dxs = state_change_sequence;
  const auto& x_target = state_target;
  auto& us = input_sequence;
  auto& dus = input_change_sequence;

  // Forward pass to propagate input sequence
  xs[0] = state;
  for (int k = 0; k < N; k++) {
    xs[k + 1] = dynsys_p->step(xs[k], us[k], stepsize);
    dynsys_p->discretize(xs[k], us[k], stepsize, A[k], B[k]);
  }

  // Backward pass to obtain S[k], v[k]
  S[N] = Qf;
  v[N] = Qf * (xs[N] - x_target);
  MatrixXd I;
  I.setIdentity(dynsys_p->number_of_states(), dynsys_p->number_of_states());
  for (int k = N - 1; k >= 0; k--) {
    MatrixXd V = (B[k].transpose() * S[k + 1] * B[k] + R).inverse();
    K[k] = V * B[k].transpose() * S[k + 1] * A[k];
    Kv[k] = V * B[k].transpose();
    Ku[k] = V * R;

    S[k] = A[k].transpose() * S[k + 1] * (A[k] - B[k] * K[k]) + Q;

    v[k] = (A[k] - B[k] * K[k]).transpose() * v[k + 1] -
           K[k].transpose() * R * us[k] + Q * xs[k];
  }

  // Forward pass to obtain dx[k], du[k] and update optimal trajectories
  dxs[0].setZero(dynsys_p->number_of_states());
  dus[0].setZero(dynsys_p->number_of_inputs());
  for (int k = 0; k < N; k++) {
    dus[k] = -K[k] * dxs[k] - Kv[k] * v[k + 1] - Ku[k] * us[k];
    dxs[k + 1] = A[k] * dxs[k] + B[k] * dus[k];

    us[k] = us[k] + dus[k];
    xs[k + 1] = xs[k + 1] + dxs[k + 1];
  }

  VectorXd input_now = input_sequence[0];
  // VectorXd state_trajectory_end = dynsys_p->step(xs[N], us[N], stepsize);
  // shift sequence forward by erasing first element and duplicating final
  input_sequence.erase(input_sequence.begin());
  input_sequence.push_back(*input_sequence.crbegin());

  return input_now;
}