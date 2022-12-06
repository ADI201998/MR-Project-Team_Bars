#include <iostream>
#include <string>
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

    void multiViewAdjuster_fn();

    std::vector<<std::vector<double>> readFile(std::string fileName);
};