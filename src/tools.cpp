#include <osik_control/tools.h>
#include <stack>



void pinv(const Eigen::MatrixXd& matrix_in, 
          Eigen::MatrixXd& pseudo_inv, 
          const double& pinvtoler)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix_in, Eigen::ComputeThinU | 
                                        Eigen::ComputeThinV);        
  Eigen::VectorXd singular_values;
  Eigen::VectorXd singular_values_inv;
  singular_values = svd.singularValues();
  singular_values_inv.setZero(singular_values.size());

  for (int w = 0; w < singular_values.size(); ++w)
    if( singular_values(w) > pinvtoler)
      singular_values_inv(w) = 1/singular_values(w);
  pseudo_inv = svd.matrixV() * singular_values_inv.asDiagonal() * 
    svd.matrixU().transpose();                 
  return;
}


void get_joint_limits(const std::string& model_name,
                      std::vector<double>& qmin,
                      std::vector<double>& qmax,
                      std::vector<double>& dqmax,
                      std::vector<std::string>& jnames)
{
  typedef boost::shared_ptr<urdf::Link> LinkPtr;
  typedef boost::shared_ptr<urdf::Joint> JointPtr;

  boost::shared_ptr<urdf::ModelInterface> 
    urdf_model = urdf::parseURDFFile(model_name);

  std::map<std::string, LinkPtr > link_map;
  link_map = urdf_model->links_;
  
  std::stack<LinkPtr > link_stack;
  std::stack<int> joint_index_stack;
  
  // Add the bodies in a depth-first order of the model tree
  link_stack.push (link_map[(urdf_model->getRoot()->name)]);
  
  if (link_stack.top()->child_joints.size() > 0) {
    joint_index_stack.push(0);
  }
  
  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();
    unsigned int joint_idx = joint_index_stack.top();
    
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];
      
      // Increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx+1);
      
      link_stack.push (link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (cur_joint->type == 1 || cur_joint->type == 2)
      {
        // std::cout << "joint: " << cur_joint->name << std::endl;
        jnames.push_back(cur_joint->name);
        // For revolute joints
        if (cur_joint->type == 1) {
          qmin.push_back(cur_joint->limits->lower);
          qmax.push_back(cur_joint->limits->upper);
          dqmax.push_back(cur_joint->limits->velocity);
        }
        // For continuous joints, set arbitrarily big values
        else {
          qmin.push_back(-10.0);
          qmax.push_back(10.0);
          dqmax.push_back(50.0);
        }
      }
    } 
    else 
    {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }
}


void print_body_names(RigidBodyDynamics::Model* model)
{
  std::map< std::string, unsigned int > bodymap = model->mBodyNameMap;
  for (std::map< std::string, unsigned int >::iterator it=bodymap.begin();
       it!=bodymap.end(); ++it)
  {
    // Show the bodies and their ids
    std::cout << "id: " << it->second << " - body name: " << it->first
              << std::endl;
  }
}
