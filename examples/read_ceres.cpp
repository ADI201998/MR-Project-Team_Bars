#include <ceres/ceres.h>
#include <glog/logging.h>
#include <readCeresInput.h>

int main()
{
    std::string filename = "../files/ceres_input_multiViewAdjuster.txt";
    readCeresInput rci(filename);
    rci.readFile();
    std::cout<<"Views = "<<rci.get_numViews()<<"\n";
    std::cout<<"Pts = "<<rci.get_numPts()<<"\n";
    std::cout<<"Obs = "<<rci.get_numObs()<<"\n";
    std::cout<<"Faces = "<<rci.get_numFaces()<<"\n";
    std::cout<<"Height = "<<rci.get_height()<<"\n";
    std::cout<<"Width = "<<rci.get_width()<<"\n";
    std::cout<<"Length = "<<rci.get_length()<<"\n";
    std::cout<<"K = "<<rci.get_K().size()<<"\n";
    std::cout<<"Center of Car = "<<rci.get_centerOfCar().size()<<"\n";
    std::cout<<"Keypoint Observations = "<<rci.get_keypointObservations().size()<<"\n";
    std::cout<<"observationWeights = "<<rci.get_observationWeights().size()<<"\n";
    std::cout<<"Mean Locations = "<<rci.get_meanLocations().size()<<"\n";
    std::cout<<"eigen Vectors = "<<rci.get_eigenVectors().size()<<"\n";
    std::cout<<"lambdas = "<<rci.get_lambdas().size()<<"\n";
    std::cout<<"Rotation Estimate = "<<rci.get_rotationEstimate().size()<<"\n";
    std::cout<<"Translation Estimate = "<<rci.get_translationEstimate().size()<<"\n";
    std::cout<<"Translation Estimate = "<<rci.get_translationEstimate()[91][2]<<"\n";

    return 0;
}