#ifndef OSIK_KINEMATIC_SOLVER_NS_H
#define OSIK_KINEMATIC_SOLVER_NS_H

#include <osik_control/KinematicSolver.h>


class KinematicSolverNS :
  public KinematicSolver
{
public:
  
  /** 
   * Constructor
   * @param[in] model Pointer to the model of the robot in RBDL format
   * @param[in] qinit Initial joint configuration
   * @param[in] dt control time (default is 10ms)
   */
  KinematicSolverNS(RigidBodyDynamics::Model* model,
                    const Eigen::VectorXd& qinit, 
                    const double& dt=0.010);
  
  /** 
   * Solve the prioritized system and get the desired position control for the
   * given configuration using projections onto the nullspaces of tasks with
   * higher priorities.
   * @param[in] q current configuration
   * @param[out] qdes joint control
   */
  void getPositionControl(const Eigen::VectorXd& q,
                          Eigen::VectorXd& qdes);

};

#endif
