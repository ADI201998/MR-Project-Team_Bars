#include <iostream>
#include <string>
#include <vector>
#include <string>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <fstream>
#include <multiViewAdjuster.h>

class multiViewShapeAdjuster
{
private:
    std::string fileNameInput;
    std::string fileNameOutput;
    int seqID;
    int startFrm;
    int endFrm;
    int carID;
    
public:
    multiViewShapeAdjuster(int seqID, int startFrm, int endFrm, int carID);
    ~multiViewShapeAdjuster();

    struct Objects
    {
        int frame;
        int id;
        std::string type;
        int truncation;
        int occlusion;
        double alpha;
        double x1;
        double y1;
        double x2;
        double y2;
        double h;
        double w;
        double l;
        double t1;
        double t2;
        double t3;
        double ry;
        double score;
    };
    
    std::vector<Objects> objects;

    int numObs;
    int numPts;
    int numVecs;
    int numViews;
    Eigen::ArrayXXd K;
    double avgCarLength;
    double avgCarWidth;
    double avgCarHeight;

    void keypointLocalizations(Eigen::ArrayXd seq, Eigen::ArrayXd frm, Eigen::ArrayXd id, Eigen::ArrayXXd *new_seq_frm_id, Eigen::ArrayXXd *tracklets_op, Eigen::ArrayXXd *ground_truth, Eigen::ArrayXXd *keypoints_collection, Eigen::ArrayXXd *wkps);

    void mobili(Eigen::ArrayXXd tracklets_op, Eigen::ArrayXXd ground_truth, Eigen::ArrayXXd *B);

    void approxAlignWireFrame(Eigen::ArrayXXd tracklets_data, Eigen::ArrayXXd B, Eigen::ArrayXXd *wireframe_collection, Eigen::ArrayXXd *def_vectors_collection, Eigen::ArrayXXd *rotation_collection);

    void initialTransformations(Eigen::ArrayXXd *old_wireframe, Eigen::ArrayXXd *old_def_vectors);

    void scaleWireframes(Eigen::ArrayXXd *wireframe_scaled, Eigen::ArrayXXd *transformed_deformation_vectors);

    void shapeOptimizer(Eigen::ArrayXd frm, Eigen::ArrayXXd wireframe, Eigen::ArrayXXd def_vectors, Eigen::ArrayXXd rot_y, Eigen::ArrayXXd observation_wts, Eigen::ArrayXXd keypoints_collection, Eigen::ArrayXXd B, Eigen::ArrayXXd *def_vectors_collection, Eigen::ArrayXXd *rotation_collection, Eigen::ArrayXXd *translation_collection, Eigen::ArrayXXd *lambdas_collection);

    void poseOptimizer(Eigen::ArrayXd frm, Eigen::ArrayXXd wireframe, Eigen::ArrayXXd def_vectors, Eigen::ArrayXXd rot_y, Eigen::ArrayXXd observation_wts, Eigen::ArrayXXd keypoints_collection, Eigen::ArrayXXd B, Eigen::ArrayXXd lambdas, Eigen::ArrayXXd *rotation_collection, Eigen::ArrayXXd *translation_collection);

    void keypointsWeights(Eigen::ArrayXXd wkps, Eigen::ArrayXXd tracklets_data, Eigen::ArrayXXd *observation_weights);

    void tracklets_helper(Eigen::ArrayXd seq, Eigen::ArrayXd frm, Eigen::ArrayXd id, Eigen::ArrayXXd *tracklets_op, Eigen::ArrayXXd *ground_truth);

    void readLabels(std::string filePath, int seq);

    void multiViewAdjuster_fn();

    Eigen::ArrayXXd readFile(std::string fileName);
};