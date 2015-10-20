#include <osik_control/KinematicTask.h>



KinematicTask::KinematicTask(RigidBodyDynamics::Model* model,
                             const std::string& taskName,
                             const double& gain)
  :
  model_(model),
  name_(taskName),
  gain_(gain),
  weight_(1.0),
  adaptiveGain_(false),
  lmax_(gain),
  lmin_(gain),
  k_(0.0)
{
  ndof_ = model->dof_count;
}


void KinematicTask::setName(const std::string& taskName)
{
  name_ = taskName;
}


void KinematicTask::setGain(const double& gain)
{
  // Set a constant gain
  adaptiveGain_ = false;
  gain_ = gain;
}


void KinematicTask::setAdaptiveGain(const double& max_gain, 
                                    const double& min_gain, 
                                    const double& rate_descent)
{
  // Set an adaptive gain
  adaptiveGain_ = true;
  std::cout << "Task " << name_ << ": Using adaptive gain" << std::endl;
  // Set the gains
  lmax_ = max_gain;
  lmin_ = min_gain;
  k_ = rate_descent;
  // The gain is set to the minimum gain
  gain_ = lmin_;
}



double KinematicTask::getGain(const double& error)
{
  if (adaptiveGain_)
  {
    gain_ = (lmax_-lmin_)*exp(-k_*error)+lmin_;
  }
  std::cout << gain_ << std::endl;
  return gain_;
}


void KinematicTask::setDesiredValue(const Eigen::Ref<const Eigen::VectorXd>& desiredValue)
{
  xdes_ = desiredValue;
  esum_ = Eigen::VectorXd::Zero(xdes_.size());
}


void KinematicTask::getDesiredValue(Eigen::VectorXd& desiredValue)
{
  desiredValue = xdes_;
}


void KinematicTask::setWeight(const double& weight)
{
  weight_ = weight;
}


double KinematicTask::getWeight()
{
  return weight_;
}
