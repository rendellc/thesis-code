// #include <control/path/path.hpp>
#include <control/dynamics/first_order.hpp>
#include <control/dynamics/inverted_pendulum.hpp>
#include <control/iterative_lqr.hpp>
#include <iostream>
#include <memory>
#include <vector>

int main() {
  const double t_stop = 10.0;
  const double dt = 0.01;

  // std::shared_ptr<DynamicalSystem> sys =
  //     std::make_shared<InvertedPendulum>(1.0, 0.01, 1.0, 9.8);
  const double gain = 2.0;
  std::shared_ptr<DynamicalSystem> sys = std::make_shared<FirstOrder>(gain);

  Eigen::VectorXd cost_states(2);
  cost_states << 0.0, 0.0;
  Eigen::VectorXd cost_states_final(2);
  cost_states << 1.0, 1.0;
  Eigen::VectorXd cost_inputs(1);
  cost_inputs << 1e-5;
  std::vector<Eigen::VectorXd> input_sequence;
  input_sequence.resize(10);
  for (auto& input : input_sequence) {
    input.setZero(1);
  }

  const double ilqr_dt = 0.1;
  std::shared_ptr<IterativeLQR> ilqr =
      std::make_shared<IterativeLQR>(sys, cost_states, cost_states_final,
                                     cost_inputs, input_sequence, ilqr_dt);

  Eigen::VectorXd states(2);
  states << 3.1415, 0.0;
  std::cout << states << std::endl;

  Eigen::VectorXd state_target(2);
  state_target << 0.0, 0.0;

  double t = 0;
  while (t < t_stop) {
    Eigen::VectorXd inputs = ilqr->update(states, state_target);

    states = sys->step(states, inputs, dt);
    // std::cout << states(0) << " " << states(2) << std::endl;
    t += dt;
    std::cout << states(0) << " " << states(1) << std::endl;
  }

  return 0;
}
