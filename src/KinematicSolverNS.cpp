#include <osik_control/KinematicSolverNS.h>
#include <osik_control/tools.h>



KinematicSolverNS::KinematicSolverNS(RigidBodyDynamics::Model* model,
                                     const Eigen::VectorXd& qinit, 
                                     const double& dt)
  :
  KinematicSolver(model, qinit, dt)
{
}


void KinematicSolverNS::getPositionControl(const Eigen::VectorXd& q,
                                           Eigen::VectorXd& qdes)
{
  // Fail if called without tasks
  if (taskStack_.size()==0){
    std::cerr << "Cannot get control without tasks ... "
              << "keeping previous configuration" << std::endl;
    qdes = q;
    return;
  }
  
  Eigen::MatrixXd J, pinvJ, P;
  Eigen::VectorXd dq, de;

  // Just to get the shape of the jacobian
  // TODO: get it in a better way
  taskStack_[0]->getJacobian(q, J);
  P = Eigen::MatrixXd::Identity(J.cols(), J.cols());
  dq = Eigen::VectorXd::Zero(J.cols());

  Eigen::MatrixXd A1, A1p, A2, Pt;
  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    taskStack_[i]->getJacobian(q, J);
    taskStack_[i]->getDerivError(q, de);
    A1 = J*P;
    pinv(A1, A1p);
    A2 = de - J*dq;
    Pt = Eigen::MatrixXd::Identity(A1p.rows(), A1p.rows()) - A1p*A1;
    dq = dq + A1p*A2;
    P = P*Pt;
  }
  
  qdes = qdes_ + dt_*dq; 

  qdes_ = qdes;
}
