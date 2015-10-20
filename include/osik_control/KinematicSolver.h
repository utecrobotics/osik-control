#ifndef OSIK_KINEMATIC_SOLVER_H
#define OSIK_KINEMATIC_SOLVER_H

#include <osik_control/KinematicTask.h>


class KinematicSolver
{
public:

  /** 
   * Constructor
   * @param[in] model Pointer to the model of the robot in RBDL format
   * @param[in] qinit Initial joint configuration
   * @param[in] dt control time
   */
  KinematicSolver(RigidBodyDynamics::Model* model,
                  const Eigen::VectorXd& qinit, 
                  const double& dt);
  
  /** 
   * Push a task into the stack of tasks
   * @param[in] task Pointer to the kinematic task
   */
  void pushTask(KinematicTask* task);

  /** 
   * Remove the last added task (pop) from the stack of tasks
   */
  void popTask();

  /** 
   * Remove a task from the stack of tasks using the task name.
   * @param[in] taskName Name of the task to be removed
   */
  void removeTask(const std::string& taskName);


  /** 
   * Get the stack containing pointers to the currently existing tasks
   * @param tasks Current tasks
   */
  void getTaskStack(std::vector< KinematicTask* >& tasks);

  /**
   * Solve the system and get the desired position control for the given 
   * configuration 
   * @param[in] q current configuration
   * @param[out] qdes joint control
   */
  virtual void getPositionControl(const Eigen::VectorXd& q,
                                  Eigen::VectorXd& qdes) = 0;

  //KinematicTask getTaskByName(const std::string& taskName);


protected:

  /// Robot Model in RBDL format
  RigidBodyDynamics::Model* model_;
  /// Discretization (control) time
  double dt_;
  /// Desired value for the joints (TODO: check this name)
  Eigen::VectorXd qdes_;
  /// Stack of Tasks
  std::vector< KinematicTask* > taskStack_;
  /// Number of degrees of freedom
  unsigned int ndof_;

};

#endif
