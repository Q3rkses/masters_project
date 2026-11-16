/**
 * @file iterated_extended_kalman_smoother.hpp
 * @brief Contains an iterated extended Kalman smoother class. It finds the
 * maximum a posteriori trajectory of the whole run with Levenberg-Marquardt,
 * which is Gauss-Newton with damping, as in Algorithm 13.5 of Bayesian
 * Filtering and Smoothing (Sarkka, p.274)
 */

#ifndef ITERATED_EXTENDED_KALMAN_SMOOTHER_HPP
#define ITERATED_EXTENDED_KALMAN_SMOOTHER_HPP

#include "models/nonlinear_models.hpp"
#include "smoothers/bayesian_batch_smoother.hpp"
#include <Eigen/Dense>
#include <vector>

class IteratedExtendedKalmanSmoother final : public BayesianBatchSmoother {
public:
  /**
   * @brief Constructor for the IteratedExtendedKalmanSmoother class.
   * @param smoother_config all configuration parameters that are
   * used by the constructor
   * @param lm_config the Levenberg-Marquardt parameters
   */
  explicit IteratedExtendedKalmanSmoother(
      const SmootherConfig &smoother_config,
      const LevenbergMarquardtConfig &lm_config);

  /**
   * @brief Each iteration linearizes the models about the current
   * trajectory, solves the resulting linear problem for the step with a
   * Kalman filter and an RTS smoother, and takes the step if the cost of the
   * trajectory goes down. The prior is node -1, as in run_filter.
   * @param smoother_config the smoother's prior and motion model
   * @param inputs the input of every step
   * @param sensors the sensors, with their measurements and when they fire
   * @param initial_guess a filter's recorded states, the trajectory the
   * iteration starts from. Its covariances scale the damping.
   * @return the smoothed state at every step, the covariance is the one of
   * the linear problem at the final trajectory, without damping
   */
  SmootherResult smooth(const SmootherConfig &smoother_config,
                        const std::vector<Input> &inputs,
                        const std::vector<Sensor> &sensors,
                        const FilterResult &initial_guess) override;

private:
  // The trajectory has timesteps + 1 nodes, node 0 is the prior and node
  // k + 1 is the state at step k.
  using Trajectory = std::vector<Eigen::VectorXd>;

  // The step to take from a trajectory and the covariance of that step.
  struct LinearSolution {
    std::vector<Eigen::VectorXd> delta;
    std::vector<Eigen::MatrixXd> P;
  };

  /**
   * @brief the negative log posterior of a trajectory, up to a constant
   */
  double cost(const Trajectory &x, const SmootherConfig &smoother_config,
              const std::vector<Input> &inputs,
              const std::vector<Sensor> &sensors) const;

  /**
   * @brief linearizes the models at x_ref, and solves for the step delta
   * with a Kalman filter and an RTS smoother. The step is in the error
   * coordinates delta_k = x_k - x_ref_k.
   * @param damping_covariance the covariance of each node's step when
   * damped, the damping is a measurement delta_k = 0 with the covariance
   * damping_covariance / lambda
   * @param lambda the damping, 0 for none
   */
  LinearSolution
  solve_linearized(const Trajectory &x_ref,
                   const SmootherConfig &smoother_config,
                   const std::vector<Input> &inputs,
                   const std::vector<Sensor> &sensors,
                   const std::vector<Eigen::MatrixXd> &damping_covariance,
                   const double lambda) const;

  std::shared_ptr<const MotionModel> motion_model_;
  LevenbergMarquardtConfig lm_config_;
};

#endif
