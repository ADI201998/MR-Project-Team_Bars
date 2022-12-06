#include <iostream>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <eigen3/Eigen/Dense>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;

class ReprojectionError {
 public:
  ReprojectionError(
        const Eigen::Matrix<double, 3, 4>& projection_matrix,
        const Eigen::Vector2d& feature)
        : projection_matrix_(projection_matrix), feature_(feature) {}

  template <typename T>
  bool operator()(const T* input_point, T* reprojection_error) const {
        Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);

        // Multiply the point with the projection matrix, then perform homogeneous
        // normalization to obtain the 2D pixel location of the reprojection.

        const Eigen::Matrix<T, 2, 1> reprojected_pixel =  (projection_matrix_ * point).hnormalized();
        // Reprojection error is the distance from the reprojection to the observed
        // feature location.
        //cout << point << endl;
        //reprojection_error[0] = feature_[0] - reprojected_pixel[0]; 
        //reprojection_error[1] = feature_[1] - reprojected_pixel[1];
        auto a =  feature_ - reprojected_pixel;
        reprojection_error[0] = a[0];
        reprojection_error[1] = a[1];

        return true;
  }
  static ceres::CostFunction * Create(const Eigen::Matrix<double, 3, 4>& projection_matrix_, 
        const Eigen::Vector2d& feature_) {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2,4>
                (new ReprojectionError(projection_matrix_,feature_)));
    }

 private:
    const Eigen::Matrix<double, 3, 4>& projection_matrix_;
    const Eigen::Vector2d& feature_;
};

int main() {
        Eigen::Vector4d x;
        x << 0,0,0,1;
        Eigen::Matrix<double, 3, 4> a;
        a << 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1;
        Eigen::Vector2d b;
        b << 4, 5; 
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::LossFunction* loss= nullptr;
        ceres::Solver::Summary summary;
        CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2,4>
                (new ReprojectionError(a, b));
        problem.AddResidualBlock(cost_function,loss,&x[0]);
        ceres::Solve(options,&problem, &summary);
        //std::cout << summary.FullReport() << "\n";
        std::cout << x << "\n";
        return 0;
}
