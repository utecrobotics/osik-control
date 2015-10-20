#ifndef OSIK_TOOLS_H
#define OSIK_TOOLS_H

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rbdl/rbdl.h>


namespace RbdMath = RigidBodyDynamics::Math;


/** 
 * @brief Compute the Moore-Penrose pseudoinverse
 * @param[in] input_mat Input matrix
 * @param[out] pseudo_inv_mat Pseudo inverse of the input matrix
 * @param[in] pinvtoler Tolerance for discarding null singular values
 */
void pinv(const Eigen::MatrixXd& input_mat, 
          Eigen::MatrixXd& pseudo_inv_mat, 
          const double& pinvtoler = 1.0e-6);

/** 
 * @brief Get the names of the joints and the angular limits (only for revolute
 * joints). For continuous joints, the limits are set to -10, 10.
 * @param[in] path_to_model Path to the URDF model
 * @param[in] qmin lower bound for the joints
 * @param[in] qmax upper bound for the joints
 * @param[in] dqmax maximum joint velocity (assumed to exist in the URDF model)
 * @param[out] joint_names Names for the rotational and continuous joints
 */
void get_joint_limits(const std::string& path_to_model,
                      std::vector<double>& qmin,
                      std::vector<double>& qmax,
                      std::vector<double>& dqmax,
                      std::vector<std::string>& joint_names);

/** 
 * @brief Print to the screen the names of the bodies and their Ids as parsed
 *        by RBDL 
 * @param[in] model RBDL model
 */
void print_body_names(RigidBodyDynamics::Model* model);


#endif
