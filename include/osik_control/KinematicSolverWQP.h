#ifndef OSIK_KINEMATIC_SOLVER_WQP_H
#define OSIK_KINEMATIC_SOLVER_WQP_H

#include <osik_control/KinematicSolver.h>

#include <qpOASES.hpp>


class KinematicSolverWQP 
  : public KinematicSolver
{
public:
  
  /** 
   * Constructor
   * @param[in] model Pointer to the model of the robot in RBDL format
   * @param[in] qinit Initial joint configuration
   * @param[in] dt control time (default is 10ms)
   */
  KinematicSolverWQP(RigidBodyDynamics::Model* model, 
                     const Eigen::VectorXd& qinit, 
                     const double& dt=0.010);
  
  /** 
   * Solve the prioritized system and get the desired position control for the
   * given configuration using a weighted scheme based on QPs.
   * @param[in] q current configuration
   * @param[out] qdes joint control
   */
  void getPositionControl(const Eigen::VectorXd& q,
                          Eigen::VectorXd& qdes);

  /** 
   * Set the angular joint limits and velocity joint limits
   * @param[in] qmin angular lower bounds
   * @param[in] qmax angular upper bounds
   * @param[in] dqmax velocity limits
   */
  void setJointLimits(const std::vector<double>& qmin,
                      const std::vector<double>& qmax,
                      const std::vector<double>& dqmax);

private:

  /// Lower bound for the joints
  std::vector<double> qmin_;
  /// Upper bound for the joints
  std::vector<double> qmax_;
  /// Maximum joint velocity
  std::vector<double> dqmax_;
  /// Options for QP
  qpOASES::Options qpOptions_;
  
};

#endif
