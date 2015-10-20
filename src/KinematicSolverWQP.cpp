#include <osik_control/KinematicSolverWQP.h>


KinematicSolverWQP::KinematicSolverWQP(RigidBodyDynamics::Model* model, 
                                       const Eigen::VectorXd& qinit, 
                                       const double& dt)
  :
  KinematicSolver(model, qinit, dt)
{
  qpOptions_.setToMPC();
  qpOptions_.printLevel = qpOASES::PL_LOW;

}

void KinematicSolverWQP::setJointLimits(const std::vector<double>& qmin,
                                        const std::vector<double>& qmax,
                                        const std::vector<double>& dqmax)
{
  qmin_ = qmin;
  qmax_ = qmax;
  dqmax_ = dqmax;
}



void KinematicSolverWQP::getPositionControl(const Eigen::VectorXd& q,
                                            Eigen::VectorXd& qdes)
{
  // Fail if called without tasks
  if (taskStack_.size()==0){
    std::cerr << "Cannot get control without tasks ... "
              << "keeping previous configuration" << std::endl;
    qdes = q;
    return;
  }

  // Fail if no joint limits set
  if ( (qmin_.size()==0) || (qmax_.size()==0) ){
    std::cerr << "Joint limits are not set! ..."
              << "keeping previous configuration" << std::endl;
    qdes = q;
    return;
  }
    
  Eigen::VectorXd dq, de, p, eint;
  Eigen::MatrixXd J;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> W;
  double weight;

  // W = Eigen::MatrixXd::Zero(q.size(), q.size());
  // Adding regularization term
  W = Eigen::MatrixXd::Identity(q.size(), q.size());
  p = Eigen::VectorXd::Zero(q.size());

  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    taskStack_[i]->getJacobian(q, J);
    taskStack_[i]->getDerivError(q, de);
    taskStack_[i]->getIntegralError(q, eint);
    weight = taskStack_[i]->getWeight();

    W = W + weight*J.transpose()*J;
    //p = p - 2*weight*J.transpose()*(de+eint);
    p = p - 2*weight*J.transpose()*(de);
  }

  int nC=ndof_, nV=ndof_;

  double *H = W.data();
  double *g = p.data();

  double *lb = new double[ndof_];
  double *ub = new double[ndof_];
  double ltmp, utmp;
  for (unsigned int i=0; i<ndof_; ++i)
  {
    ltmp = (1.0/dt_)*(qmin_[i]-q[i]);
    utmp = (1.0/dt_)*(qmax_[i]-q[i]);
    lb[i] = ltmp > -dqmax_[i] ? ltmp : -dqmax_[i];
    ub[i] = utmp < dqmax_[i] ? utmp : dqmax_[i];
  }

  qpOASES::QProblemB qp(nV);
  qp.setOptions(qpOptions_);
  int nWSR = 10;  // Max number of working set recalculations
  qp.init(H, g, lb, ub, nWSR, 0);
  
  double dqArray[nV];
  qp.getPrimalSolution(dqArray);

  dq.resize(ndof_);
  for (unsigned int i=0; i<ndof_; ++i)
    dq[i] = dqArray[i];

  qdes = qdes_ + dt_*dq; 
  qdes_ = qdes;

}
