#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

// Contains definitions of various problem structs
#include <problemStructs.hpp>
// Contains various cost function struct specifications
#include <costFunctions.hpp>

class singleViewPoseAdjuster
{
public:
    singleViewPoseAdjuster();
    ~singleViewPoseAdjuster();
    void solve_singleViewPoseAdjuster();
};