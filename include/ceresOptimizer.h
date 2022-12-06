#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Dense>

class ceresOptimizer
{
private:
    /* data */
public:
    ceresOptimizer(/* args */);
    ~ceresOptimizer();

    Eigen::ArrayXXd readFile(std::string fileName);
};