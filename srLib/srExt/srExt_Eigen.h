#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <fstream>
#include <map>
#include <vector>

namespace Eigen
{
	typedef Eigen::Matrix<double, 6, 6>	Matrix6d;
	typedef Eigen::Matrix<double, 6, Eigen::Dynamic>	Matrix6Xd;
	typedef Eigen::Matrix<double, 6, 1>	Vector6d;

	template<typename Derived>
	MatrixXd pInv(const MatrixBase<Derived>& A)
	{
		//JacobiSVD<Derived> svd = (A.rows() < A.cols() ? Derived(A.transpose()).jacobiSvd(ComputeThinU | ComputeThinV) : A.jacobiSvd(ComputeThinU | ComputeThinV));
		//RealScalar tolerance = (RealScalar)std::numeric_limits<RealScalar>::epsilon() * std::max((RealScalar)A.cols(), (RealScalar)A.rows()) * svd.singularValues().array().abs().maxCoeff();
		//return svd.matrixV() * Derived((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint();

		typedef typename MatrixBase<Derived>::RealScalar RealScalar;
		JacobiSVD<Derived> svd(A, ComputeThinU | ComputeThinV);
		RealScalar tolerance = std::numeric_limits<RealScalar>::epsilon() * max(A.cols(), A.rows()) *
			svd.singularValues()[0];

		auto	singVal = svd.singularValues();
		singVal = (singVal.array() > tolerance).
			select(singVal.array().inverse(), 0);
		return svd.matrixV() * singVal.asDiagonal() * svd.matrixU().adjoint();
	}

	Matrix3d skewSymm(double _v0, double _v1, double _v2);

	template<typename vectorClass>
	Matrix3d skewSymm(const vectorClass& _v)
	{
		Matrix3d ret;
		ret(0, 0) = 0;
		ret(1, 1) = 0;
		ret(2, 2) = 0;

		ret(1, 0) = _v[2];
		ret(0, 1) = -_v[2];

		ret(2, 0) = -_v[1];
		ret(0, 2) = _v[1];

		ret(2, 1) = _v[0];
		ret(1, 2) = -_v[0];
		return ret;
	}

	template<typename Derived>
	void writeToFile(const MatrixBase<Derived>& mat, const std::string& fileName)
	{
        std::ofstream outText(fileName);
		for (int row = 0; row < mat.rows(); row++)
		{
			for (int col = 0; col < mat.cols(); col++)
				outText << mat(row, col) << '\t';
			outText << '\n';
		}
		outText.close();
	}

	template<typename T>
	void writeToFile(const std::vector<T>& vec, const std::string& fileName)
	{
		std::ofstream outText(fileName);
		for (unsigned row = 0; row < vec.size(); row++)
			outText << vec[row] << '\n';
		outText.close();
	}

	template<typename T>
	void writeToFile(const std::vector<std::vector<T>>& mat, const std::string& fileName)
	{
		std::ofstream outText(fileName);
		for (unsigned row = 0; row < mat.size(); row++)
		{
			for (unsigned col = 0; col < mat[row].size(); col++)
				outText << mat[row][col] << '\t';
			outText << '\n';
		}
		outText.close();
	}

	void writeToFile(const std::map<std::string, Eigen::VectorXd>& vecWithName, const std::string& fileName, bool save_label = true);

	Eigen::MatrixXd readFromFile(const std::string& fileName);
}
