#include <osik_control/KinematicSolver.h>



KinematicSolver::KinematicSolver(RigidBodyDynamics::Model* model,
                                 const Eigen::VectorXd& qinit, 
                                 const double& dt)
  :
  model_(model),
  dt_(dt),
  qdes_(qinit)
{
  taskStack_.clear();
  ndof_ = model->dof_count;
}


void KinematicSolver::getTaskStack(std::vector< KinematicTask* >& tasks)
{
  tasks = taskStack_;
}


void KinematicSolver::pushTask(KinematicTask* task)
{
  taskStack_.push_back(task);
  
  // // This is only useful for the weighting scheme (KinematicSolverWQP)
  // unsigned int stask = task->getTaskSize();  // task size
  // unsigned int sw = W_.rows();  // size of W matrix (must be squared)

  // Eigen::MatrixXd Wtask, Wprev;
  // Wtask = (task->getWeight())*Eigen::MatrixXd::Identity(stask, stask);
  // Wprev = W_;

  // W_.resize(sw+stask, sw+stask);
  // W_.topLeftCorner(sw,sw) = Wprev;
  // W_.bottomRightCorner(stask,stask) = Wtask;
}
