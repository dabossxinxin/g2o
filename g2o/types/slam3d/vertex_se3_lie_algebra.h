#pragma once

#include "EXTERNAL/sophus/so3.hpp"
#include "EXTERNAL/sophus/se3.hpp"

#include "g2o_types_slam3d_api.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/block_solver.h"

namespace g2o
{
	class G2O_TYPES_SLAM3D_API VertexSE3LieAlgebra : public g2o::BaseVertex<6, Sophus::SE3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexSE3LieAlgebra();

		~VertexSE3LieAlgebra() {};

		virtual bool read(std::istream& is);

		virtual bool write(std::ostream& os) const;

		virtual void setToOriginImpl();

		virtual void oplusImpl(const double* update);
	};
}