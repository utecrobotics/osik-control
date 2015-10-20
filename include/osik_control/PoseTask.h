#ifndef OSIK_POSE_TASK_H
#define OSIK_POSE_TASK_H

#include <osik_control/KinematicTask.h>
#include <osik_control/tools.h>


class PoseTask : 
  public KinematicTask
{
public:

  /** 
   * @brief Constructor
   * @param[in] model Robot model in RBDL format
   * @param[in] linkNum Link number to be controlled
   * @param[in] taskType It can be "position", "orientation" or "pose"
   * @param[in] taskName Name for the task
   */
  PoseTask(RigidBodyDynamics::Model* model,
           const unsigned int& linkNum, 
           const std::string& taskType,
           const std::string& taskName = "6DTask");
  
  /* Virtual functions */
  void getSensedValue(const Eigen::VectorXd& q,
                      Eigen::VectorXd& xsensed);
  // TODO: Check this eigen or eigen ref????
  void getJacobian(const Eigen::VectorXd& q, 
                   Eigen::MatrixXd& Jacobian);
  /** 
   * Get the derivative of the task error. This task uses: 
   \f$\dot e=-\lambda e\f$ where \f$e=x-x^*\f$
   * @param[in] q Joint configuration
   * @param[out] de derivative of the error
   */
  void getDerivError(const Eigen::VectorXd& q, 
                     Eigen::VectorXd& de);

  void getIntegralError(const Eigen::VectorXd& q,
                        Eigen::VectorXd& eint);

  unsigned int getTaskSize();

private:

  /// Link number
  unsigned int linkNum_;
  /// Task type: 0=position, 1=orientation, 2=pose
  unsigned int taskType_;
  /// Local position with respect to the link frame
  RbdMath::Vector3d localpos_;

};

#endif
