#include<ceresOptimizer.h>

/*
##############################################################################################

##############################################################################################
*/
ceresOptimizer::ceresOptimizer()
{

}

/*
##############################################################################################

##############################################################################################
*/
ceresOptimizer::~ceresOptimizer()
{

}

/*
##############################################################################################

##############################################################################################
*/
void multiViewShapeAdjuster::readFile(std::string fileName)
{
    int cols = 0, rows = 0;
    std::vector<std::vector<double>> buff;

    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(fileName);
    while (! infile.eof())
    {
        std::string line;
        std::getline(infile, line);

        int temp_cols = 0;
        std::stringstream stream(line);
        std::vector<double> val_temp;
        while(! stream.eof())
        {
            double val;
            stream >> val;
            val_temp.push_back(val);
            temp_cols+=1;
        }
		if(infile.eof())
			break;
        buff.push_back(val_temp);
        cols = temp_cols;
    }
    infile.close();

    // Populate matrix with numbers.
    Eigen::ArrayXXd result(buff.size(),cols);
    for (int i = 0; i < buff.size(); i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[i][j];
    return result;
}