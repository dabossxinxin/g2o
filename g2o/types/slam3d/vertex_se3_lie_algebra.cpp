#include "vertex_se3_lie_algebra.h"

namespace g2o
{
	VertexSE3LieAlgebra::VertexSE3LieAlgebra() :
		BaseVertex<6, Sophus::SE3>()
	{
		setToOriginImpl();
		updateCache();
	}

	bool VertexSE3LieAlgebra::read(std::istream& is)
	{
		double data[7];
		for (int it = 0; it < 7; ++it) {
			is >> data[it];
		}
		setEstimate(Sophus::SE3(
			Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
			Eigen::Vector3d(data[0], data[1], data[2])
		));
		return true;
	}

	bool VertexSE3LieAlgebra::write(std::ostream& os) const
	{
		os << id() << " ";
		Eigen::Quaterniond q = _estimate.unit_quaternion();
		os << _estimate.translation().transpose() << " ";
		//os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs[3];
		os << std::endl;
		return true;
	}

	void VertexSE3LieAlgebra::setToOriginImpl()
	{
		_estimate = Sophus::SE3();
	}

	void VertexSE3LieAlgebra::oplusImpl(const double* update)
	{
		Sophus::SE3 up(
			Sophus::SO3(update[3], update[4], update[5]),
			Eigen::Vector3d(update[0], update[1], update[2])
		);
		_estimate = up*_estimate;
	}
}