#include <osik_control/PoseTask.h>



PoseTask::PoseTask(RigidBodyDynamics::Model* model,
                   const unsigned int& linkNum, 
                   const std::string& taskType,
                   const std::string& taskName)
  :
  KinematicTask(model, taskName),
  linkNum_(linkNum)
{
  localpos_ = RbdMath::Vector3d::Zero();
  
  // The map is: position=0, orientation=1, pose=2
  if (!taskType.compare("position"))
    taskType_ = 0;
  else if (!taskType.compare("orientation"))
    taskType_ = 1;
  else if (!taskType.compare("pose"))
    taskType_ = 2;
  else
    std::cerr << "Unknown task type" << std::endl;
}


void PoseTask::getSensedValue(const Eigen::VectorXd& q,
                              Eigen::VectorXd& xsensed)
{
  if (taskType_ == 0)
    xsensed = CalcBodyToBaseCoordinates(*model_, q, linkNum_, 
                                        localpos_, true);
  if (taskType_ == 1)
  {
    std::cerr << "Not yet implemented !  TODO" << std::endl;
    //TODO: Handle orientation (rot matrix)
    //xsensed = CalcBodyWorldOrientation(*model_, q, linkNum_, true);
  }
  if (taskType_ == 2)
    std::cerr << "Not yet implemented !  TODO" << std::endl;
  
}


// TODO: eigen ref????
void PoseTask::getJacobian(const Eigen::VectorXd& q, 
                           Eigen::MatrixXd& Jacobian)
{
  // TODO: Check that the Jacobian is initially set to zero
  if (taskType_ == 0){
    //Jacobian.setZero();
    Jacobian.resize(3,ndof_);
    Jacobian.setZero(); // VERY IMPORTANT FOR THE JACOBIAN!!!
    //CalcPointJacobian(*model_, q, linkNum_, localpos_, Jacobian, false);
    CalcPointJacobian(*model_, q, linkNum_, localpos_, Jacobian, true);
    // std::cout << "qdes" << q.transpose() << std::endl;
    // std::cout << "rwrist: " << linkNum_ << std::endl;
    // std::cout << "localpos: " << localpos_.transpose() << std::endl;
  }
  else if (taskType_ == 1)
    std::cout << "TODO" << std::endl;
  else if (taskType_ == 2)
    std::cout << "TODO" << std::endl;
  // TODO: Use CalcBodySpatialJacobian for the orientation
}


void PoseTask::getDerivError(const Eigen::VectorXd& q, 
                             Eigen::VectorXd& de)
{
  Eigen::VectorXd xsensed, xdes, error;
  getDesiredValue(xdes);
  getSensedValue(q, xsensed);
  //de = -gain_*(xsensed-xdes);
  error = xsensed-xdes;
  de = -getGain(error.norm())*error;
}


unsigned int PoseTask::getTaskSize()
{
  return xdes_.size();
}


void PoseTask::getIntegralError(const Eigen::VectorXd& q,
                                Eigen::VectorXd& eint)
{
  Eigen::VectorXd xsensed, xdes;
  getDesiredValue(xdes);
  getSensedValue(q, xsensed);
  esum_ = esum_ + (xsensed-xdes);
  eint = -0.05*(esum_);
}
