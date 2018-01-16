#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
 public:
  /**
   * A helper method to calculate RMSE.
   */
  static VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                                const vector<VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static MatrixXd CalculateJacobian(const VectorXd &x_state);

  /**
   * Returns the specified angle in the range -PI < 0 < PI
   * @param radians_in
   * @return
   */
  static double NormalizeAngle(double radians_in);
};

#endif /* TOOLS_H_ */
