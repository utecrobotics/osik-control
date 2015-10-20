#ifndef OSIK_KINEMATIC_TASK_H
#define OSIK_KINEMATIC_TASK_H

#include <Eigen/Dense>
#include <string>
#include <rbdl/rbdl.h>


/**
 * Base class for a generic Kinematic task
 * 
 */

class KinematicTask
{
public:

  /** 
   * Constructor.
   * @param[in] model Robot model in RBDL format
   * @param[in] taskName Task Name
   * @param[in] gain Task gain
   */
  KinematicTask(RigidBodyDynamics::Model* model,
                const std::string& taskName = "KineTask",
                const double& gain = 1.0);

  /** 
   * Set the kinematic task name
   * @param[in] taskName task name
   */
  void setName(const std::string& taskName);

  /** 
   * Set a kinematic constant task gain
   * @param[in] gain task gain
   */
  void setGain(const double& gain);

  /** 
   * @brief Set the kinematic gain to adaptive.
   * The adaptive gain describes a decreasing exponential according to the task
   * error \f${x-x^*}\f$ and is
   * \f$\lambda=(\lambda_{max}-\lambda_{min})e^{-k(\Vert x-x^*
   * \Vert)}+\lambda_{min}\f$.
   * @param max_gain maximum gain for a small error (\f$\lambda_{max}>0\f$)
   * @param min_gain miminum gain for a big error (\f$\lambda_{min}>0\f$)
   * @param rate_descent descent for the exponential (\f$ k>0 \f$)
   */
  void setAdaptiveGain(const double& max_gain, 
                       const double& min_gain, 
                       const double& rate_descent);
  
  /** 
   * Get the kinematic task gain
   * @param[in] error scalar value of the error (only used when the gain is 
   *                  adaptive, ignored for a constant gain)
   * @return task gain
   */
  double getGain(const double& error=0.0);

  /** 
   * Set the desired value for the kinematic task
   * @param[in] desiredValue task desired value \f$(x^*)\f$
   */
  void setDesiredValue(const Eigen::Ref<const Eigen::VectorXd>& desiredValue);
  
  /** 
   * Get the desired value previously set for the task
   * @param[out] desiredValue task desired value \f$(x^*)\f$
   */
  void getDesiredValue(Eigen::VectorXd& desiredValue);

  /**
   * Set the task weight (only used with KinematicSolverWQP)
   * @param[in] weight Task weight
   */
  void setWeight(const double& weight);

  /**
   * Get the task weight (only used with KinematicSolverWQP)
   * @return task weight
   */
  double getWeight();
  
  /** 
   * Get the sensed value for the kinematic task given a configuration
   * @param[in] q Joint configuration
   * @param[out] xsensed Sensed value for the joint configuration
   */
  virtual void getSensedValue(const Eigen::VectorXd& q,
                              Eigen::VectorXd& xsensed) = 0;

  /** 
   * Get the task Jacobian for a given configuration
   * @param[in] q Joint configuration
   * @param[out] Jacobian Jacobian for the joint configuration
   */
  virtual void getJacobian(const Eigen::VectorXd& q, 
                           Eigen::MatrixXd& Jacobian) = 0;

  /** 
   * Get the derivative of the task error \f$(\dot e)\f$
   * @param[in] q Joint configuration
   * @param[out] de derivative of the error
   */
  virtual void getDerivError(const Eigen::VectorXd& q, 
                             Eigen::VectorXd& de) = 0;

  /** 
   * Get the integral of the task error \f$(-k_i \int e dt)\f$. This term can
   * be added to \f$\dot e^*\f$ (check if useful!).
   * @param[in] q Joint configuration
   * @param[out] eint integral of the error multiplied by a constant gain
   */
  virtual void getIntegralError(const Eigen::VectorXd& q,
                                Eigen::VectorXd& eint) = 0;

  /**
   * Get the task size (number of rows of 'e')
   * @return task size
   */
  virtual unsigned int getTaskSize() = 0;

  //void setDesiredVelocity()


protected:

  /// Robot model in RBDL format
  RigidBodyDynamics::Model* model_;
  /// Kinematic task name
  std::string name_;
  /// Kinematic task gain
  double gain_;
  /// Number of degrees of freedom of the model
  unsigned int ndof_;
  /// Desired value for the kinematic task
  Eigen::VectorXd xdes_;
  /// Weight for the task (only used with weighted QP)
  double weight_;
  /// Integral (sum of) error
  Eigen::VectorXd esum_;
  // TODO: The value of xdes_ is not initialized in the KinematicTask
  // constructor. Then, initialize xdes_ to some value (in the
  // specific tasks??

  // For the adaptive gain:
  /// Flag that sets the adaptive gain
  bool adaptiveGain_;
  /// Maximum Gain
  double lmax_;
  /// Minimum Gain
  double lmin_;
  /// Rate of descent
  double k_;

};

#endif
