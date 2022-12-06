#include <readCeresInput.h>

readCeresInput::readCeresInput(std::string filename)
{
    this->fileName = filename;
}

readCeresInput::~readCeresInput()
{
}

/*
Read from File
*/
void readCeresInput::readFile()
{
    std::ifstream fin(this->fileName);
    
    fin>>this->numViews;
    fin>>this->numPts;
    fin>>this->numObs;
    fin>>this->numFaces;

    fin>>this->height;
    fin>>this->width;
    fin>>this->length;

    for(int i=0; i<9; i++)
    {
        double K_val;
        fin>>K_val;
        this->K.push_back(K_val);
    }

    for(int i=0; i<this->numViews; i++)
    {
        double cc_x, cc_y, cc_z;
        std::vector<double> cc;
        fin>>cc_x>>cc_y>>cc_z;
        cc.push_back(cc_x);
        cc.push_back(cc_y);
        cc.push_back(cc_z);
        this->centerOfCar.push_back(cc);
    }

    for(int i=0; i<this->numViews; i++)
    {
        for(int j=0; j<this->numObs; j++)
        {
            double ko_x, ko_y;
            std::vector<double> ko;
            fin>>ko_x>>ko_y;
            ko.push_back(ko_x);
            ko.push_back(ko_y);
            this->keypointObservations.push_back(ko);
        }
    }

    for(int i=0; i<this->numViews; i++)
    {
        for(int j=0; j<this->numObs; j++)
        {
            double ow;
            fin>>ow;
            this->observationWeights.push_back(ow);
        }
    }

    for(int i=0; i<this->numViews; i++)
    {
        for(int j=0; j<this->numObs; j++)
        {
            double ml_x, ml_y, ml_z;
            std::vector<double> ml;
            fin>>ml_x>>ml_y>>ml_z;
            ml.push_back(ml_x);
            ml.push_back(ml_y);
            ml.push_back(ml_y);
            this->meanLocations.push_back(ml);
        }
    }

    for(int i=0; i<this->numViews; i++)
    {
        for(int j=0; j<42; j++)
        {
            std::vector<double> ev;
            for(int k=0; k<this->numPts*3; k++)
            {
                double ev_val;
                fin>>ev_val;
                ev.push_back(ev_val);
            }
            this->eigenVectors.push_back(ev);
        }
    }

    for(int i=0; i<42; i++)
    {
        double l;
        fin>>l;
        this->lambdas.push_back(l);
    }

    for(int i=0; i<this->numViews; i++)
    {
        std::vector<double> re;
        for(int j=0; j<9; j++)
        {
            double re_val;
            fin>>re_val;
            re.push_back(re_val);
        }
        this->rotationEstimate.push_back(re);
    }

    for(int i=0; i<this->numViews; i++)
    {
        std::vector<double> te;
        for(int j=0; j<3; j++)
        {
            double te_val;
            fin>>te_val;
            te.push_back(te_val);
        }
        this->translationEstimate.push_back(te);
    }

    fin.close();
}

/*
Get Number of Faces
*/
int readCeresInput::get_numFaces()
{
    return this->numFaces;
}

/*
Get Number of Obstacles
*/
int readCeresInput::get_numObs()
{
    return this->numObs;
}

/*
Get Number of Points
*/
int readCeresInput::get_numPts()
{
    return this->numPts;
}

/*
Get Number of Views
*/
int readCeresInput::get_numViews()
{
    return this->numViews;
}

/*
Get Height of Car
*/
double readCeresInput::get_height()
{
    return this->height;
}

/*
Get Width of Car
*/
double readCeresInput::get_width()
{
    return this->width;
}

/*
Get Length of Car
*/
double readCeresInput::get_length()
{
    return this->length;
}

/*
Get Intrinsic Matrix
*/
std::vector<double> readCeresInput::get_K()
{
    return this->K;
}

/*
Get Center of Car
*/
std::vector<std::vector<double>> readCeresInput::get_centerOfCar()
{
    return this->centerOfCar;
}

/*
Get Keypoint Observations
*/
std::vector<std::vector<double>> readCeresInput::get_keypointObservations()
{
    return this->keypointObservations;
}

/*
Get Observation Weights
*/
std::vector<double> readCeresInput::get_observationWeights()
{
    return this->observationWeights;
}

/*
Get Mean Locations
*/
std::vector<std::vector<double>> readCeresInput::get_meanLocations()
{
    return this->meanLocations;
}

/*
Get Eigen Vectors
*/
std::vector<std::vector<double>> readCeresInput::get_eigenVectors()
{
    return this->eigenVectors;
}

/*
Get Lambdas
*/
std::vector<double> readCeresInput::get_lambdas()
{
    return this->lambdas;
}

/*
Get Rotation Estimate
*/
std::vector<std::vector<double>> readCeresInput::get_rotationEstimate()
{
    return this->rotationEstimate;
}

/*
Get Translation Estimate
*/
std::vector<std::vector<double>> readCeresInput::get_translationEstimate()
{
    return this->translationEstimate;
}