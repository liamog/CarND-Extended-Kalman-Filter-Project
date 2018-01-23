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
  static bool CalculateJacobian(const VectorXd &x_state, MatrixXd *Hj);

  /**
   * Returns the specified angle in the range -PI < 0 < PI
   * @param radians_in
   * @return normalized angle in radians
   */
  static double NormalizeAngle(double radians_in);

  /**
   * Convert the vector x(px, py, vx, vy) to radar measurement space m(ro,
   * theta, ro_dot)
   * @param position state
   * @return radar measurement state
   */
  static VectorXd PositionSpaceToRadarMeasurementSpace(const VectorXd &x);
};

#endif /* TOOLS_H_ */
