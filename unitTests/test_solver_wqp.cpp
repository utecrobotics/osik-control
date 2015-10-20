#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <osik_control/tools.h>
#include <osik_control/PoseTask.h>
#include <osik_control/KinematicSolverWQP.h>
#include <osik_control/KinematicTask.h>



int main(int argc, char **argv)
{
  // ************************************************************
  // Parse the robot urdf
  // ************************************************************

  RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();

  std::string model_name = "simple_humanoid.urdf";
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(model_name.c_str(), 
                                                   model, false)) 
  {
    std::cerr << "Error loading model " << model_name.c_str() << std::endl;
    abort();
  }
  unsigned int ndof = model->dof_count;
  
  std::vector<std::string> jnames;
  std::vector<double> qmin, qmax, dqmax;
  get_joint_limits(model_name, qmin, qmax, dqmax, jnames);

  // Initialize the joint configuration
  // **********************************

  RbdMath::VectorNd qsensed = RbdMath::VectorNd::Zero(ndof);
  for (unsigned int i=0; i<ndof; ++i)
    qsensed[i] = 0.0;

  // **********************************

  // Sampling time
  unsigned int f = 10;
  double dt = static_cast<double>(1.0/f);

  // Solver
  KinematicSolverWQP solver(model, qsensed, dt);

  // Set joint limits for the solver
  solver.setJointLimits(qmin, qmax, dqmax);

  // TODO: change this properly
  unsigned int RWRIST = 5;
  unsigned int LWRIST = 10;
  unsigned int RGRIPPER = 15;
  unsigned int RELBOW = 20;


  if (false)
  {
    KinematicTask* taskrh = new PoseTask(model, RWRIST, "position");
    KinematicTask* tasklh = new PoseTask(model, LWRIST, "position");
    taskrh->setGain(1.0);
    tasklh->setGain(1.0);

    RbdMath::VectorNd Prwrist;
    taskrh->getSensedValue(qsensed, Prwrist);
    Prwrist[0] = Prwrist[0] - 0.10;
    Prwrist[1] = Prwrist[1] - 0.10;
    Prwrist[2] = Prwrist[2] + 0.15;
    taskrh->setDesiredValue(Prwrist);

    RbdMath::VectorNd Plwrist;
    tasklh->getSensedValue(qsensed, Plwrist);
    Plwrist[0] = Plwrist[0] - 0.10;
    Plwrist[1] = Plwrist[1] + 0.10;
    Plwrist[2] = Plwrist[2] + 0.15;
    tasklh->setDesiredValue(Plwrist);

    solver.pushTask(taskrh);
    solver.pushTask(tasklh);
  }
  else {
    KinematicTask* taskrh = new PoseTask(model, RGRIPPER, "position");
    //KinematicTask* taskrh = new PoseTask(model, RWRIST, "position");
    KinematicTask* taskre = new PoseTask(model, RELBOW, "position");
    KinematicTask* tasklh = new PoseTask(model, LWRIST, "position");
    taskrh->setGain(1.0);
    taskre->setGain(1.0);
    tasklh->setGain(1.0);

    RbdMath::VectorNd Prwrist;
    taskrh->getSensedValue(qsensed, Prwrist);
    taskrh->setDesiredValue(Prwrist);

    RbdMath::VectorNd Prelbow;
    taskre->getSensedValue(qsensed, Prelbow);
    Prelbow[0] = Prelbow[0] - 0.00;
    Prelbow[1] = Prelbow[1] + 0.03;
    Prelbow[2] = Prelbow[2] + 0.00;
    taskre->setDesiredValue(Prelbow);

    RbdMath::VectorNd Plwrist;
    tasklh->getSensedValue(qsensed, Plwrist);
    Plwrist[0] = Plwrist[0] - 0.10;
    Plwrist[1] = Plwrist[1] + 0.10;
    Plwrist[2] = Plwrist[2] + 0.15;
    tasklh->setDesiredValue(Plwrist);

    solver.pushTask(taskrh);
    solver.pushTask(taskre);
    solver.pushTask(tasklh);

  }

  RbdMath::VectorNd qdes;

  solver.getPositionControl(qsensed, qdes);
  qsensed = qdes;
  std::cout << qsensed.transpose() << std::endl;
  
  return 0;
}


