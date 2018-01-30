#include "srExt_Eigen.h"

using namespace Eigen;

Matrix3d Eigen::skewSymm(double _v0, double _v1, double _v2)
{
	Matrix3d ret;
	ret(0, 0) = 0;
	ret(1, 1) = 0;
	ret(2, 2) = 0;

	ret(1, 0) = _v2;
	ret(0, 1) = -_v2;

	ret(2, 0) = -_v1;
	ret(0, 2) = _v1;

	ret(2, 1) = _v0;
	ret(1, 2) = -_v0;
	return ret;
}

Eigen::MatrixXd Eigen::readFromFile(const std::string & fileName)
{
	Eigen::MatrixXd  returnMat;
	std::ifstream inputFile(fileName);
	int col = 0;
	double tempDouble;
	std::stringstream ss;

	//  count number of columns in file
	std::string line;
	while (!inputFile.eof() && getline(inputFile, line))
	{
		if (line.empty())
			continue;
		else
		{
			returnMat.conservativeResize(returnMat.rows() + 1, NoChange);
			ss = std::stringstream(line);
			col = 0;
			while (ss >> tempDouble)
			{
				if (col >= returnMat.cols())
					returnMat.conservativeResize(NoChange, col + 1);
				returnMat(returnMat.rows() - 1, col) = tempDouble;
				col++;
			}
		}
	}

	return returnMat;
}

void Eigen::writeToFile(const std::map<std::string, Eigen::VectorXd>& vecWithName, const std::string & fileName, bool save_label)
{
	std::ofstream outText(fileName);

	// Firstly, write vector data
	for (auto iter : vecWithName)
	{
		for (int i = 0; i < iter.second.size(); i++)
		{
			outText << iter.second(i) << '\t';
		}
		outText << '\n';
	}

	// Secondly, write name of each row 
	if(save_label)
		for (auto iter : vecWithName)
		{
			outText << iter.first << '\n';
		}

	outText.close();
}
