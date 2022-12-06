#include <iostream>
#include <string>
#include <vector>
#include <string>
#include <fstream>

class readCeresInput
{
    std::string fileName;
    int numViews, numObs, numPts, numFaces;
    double height, width, length;
    std::vector<double> K;
    std::vector<std::vector<double>> centerOfCar;
    std::vector<std::vector<double>> keypointObservations;
    std::vector<double> observationWeights;
    std::vector<std::vector<double>> meanLocations;
    std::vector<std::vector<double>> eigenVectors;
    std::vector<double> lambdas;
    std::vector<std::vector<double>> rotationEstimate;
    std::vector<std::vector<double>> translationEstimate;

    public:
    readCeresInput(std::string fileName);
    ~readCeresInput();
    void readFile();
    int get_numViews();
    int get_numObs();
    int get_numPts();
    int get_numFaces();
    double get_height();
    double get_width();
    double get_length();
    std::vector<double> get_K();
    std::vector<std::vector<double>> get_centerOfCar();
    std::vector<std::vector<double>> get_keypointObservations();
    std::vector<double> get_observationWeights();
    std::vector<std::vector<double>> get_meanLocations();
    std::vector<std::vector<double>> get_eigenVectors();
    std::vector<double> get_lambdas();
    std::vector<std::vector<double>> get_rotationEstimate();
    std::vector<std::vector<double>> get_translationEstimate();
};