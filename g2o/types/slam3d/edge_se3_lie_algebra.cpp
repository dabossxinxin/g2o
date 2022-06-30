#include "edge_se3_lie_algebra.h"

namespace g2o
{
	EdgeSE3LieAlgebra::EdgeSE3LieAlgebra()
	{
		information().setIdentity();
	}

	bool EdgeSE3LieAlgebra::read(std::istream& is)
	{
		double data[7];
		for (int it = 0; it < 7; ++it)
			is >> data[it];
		Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
		q.normalize();
		setMeasurement(Sophus::SE3d(q, Eigen::Vector3d(data[0], data[1], data[2])));
		for (int iti = 0; iti < information().rows() && is.good(); ++iti) {
			for (int itj = iti; itj < information().cols() && is.good(); ++itj) {
				is >> information()(iti, itj);
				if (iti != itj)
					information()(itj, iti) = information()(iti, itj);
			}
		}
		return true;
	}

	bool EdgeSE3LieAlgebra::write(std::ostream& os) const
	{
		VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
		VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
		os << v1->id() << " " << v2->id() << " ";
		Sophus::SE3 m = _measurement;
		Eigen::Quaterniond q = m.unit_quaternion();
		os << m.translation().transpose() << " ";
		os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";
		/* information matrix */
		for (int iti = 0; iti < information().rows(); ++iti) {
			for (int itj = iti; itj < information().cols(); ++itj) {
				os << information()(iti, itj) << " ";
			}
		}
		os << std::endl;
		return true;
	}

	void EdgeSE3LieAlgebra::computeError()
	{
		Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*>(_vertices[0]))->estimate();
		Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*>(_vertices[1]))->estimate();
		_error = (_measurement.inverse()*v1.inverse()*v2).log();
	}

	void EdgeSE3LieAlgebra::linearizeOplus()
	{
		Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*>(_vertices[0]))->estimate();
		Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*>(_vertices[1]))->estimate();
		Matrix6d J = JR_INV(Sophus::SE3::exp(_error));

		_jacobianOplusXi = -J*v2.inverse().Adj();
		_jacobianOplusXj = J*v2.inverse().Adj();
	}
}