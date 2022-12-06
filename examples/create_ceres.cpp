#include <ceres/ceres.h>
#include <glog/logging.h>
#include <multiViewShapeAdjuster.h>

int main()
{
    //multiViewShapeAdjuster cci(10, 0, 240, 0);
    //multiViewShapeAdjuster cci(3, 42, 132, 1);
    multiViewShapeAdjuster cci(9, 31, 50, 1);
    //multiViewShapeAdjuster cci(5, 165, 191, 12);
    //multiViewShapeAdjuster cci(5, 0, 240, 31);
    cci.multiViewAdjuster_fn();

    return 0;
}