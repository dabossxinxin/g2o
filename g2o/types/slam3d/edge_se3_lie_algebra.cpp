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
	}

	bool EdgeSE3LieAlgebra::write(std::ostream& os) const
	{
		VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
		VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
		os << v1->id() << " " << v2->id() << " ";
		Sophus::SE3d m = _measurement;
	}
}