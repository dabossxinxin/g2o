#pragma once

#include "EXTERNAL/sophus/so3.hpp"
#include "EXTERNAL/sophus/se3.hpp"

#include "g2o_types_slam3d_api.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/block_solver.h"

#include "vertex_se3_lie_algebra.h"

namespace g2o
{
	class G2O_TYPES_SLAM3D_API EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		EdgeSE3LieAlgebra();

		virtual bool read(std::istream& is);

		virtual bool write(std::ostream& os) const;

		void computeError();

		virtual void setMeasurement(const Sophus::SE3& m)
		{
			_measurement = m;
		}

		virtual bool setMeasurementData(const number_t* d)
		{
			return true;
		}

		virtual bool getMeasurementData(number_t* d) const
		{
			return true;
		}

		void linearizeOplus();

		virtual int measurementDimension() const
		{
			return 7;
		}

		virtual bool setMeasurementFromState();

		virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet&,
			OptimizableGraph::Vertex*)
		{
			return 1.0;
		}

		virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

	protected:
	};

	/**
	* \brief Pose-Pose constraint to Gnuplot data file
	*/
	class G2O_TYPES_SLAM3D_API EdgeSE3LieAlgebraWriteGnuplotAction : public WriteGnuplotAction
	{
	public:
		EdgeSE3LieAlgebraWriteGnuplotAction();
		virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
			HyperGraphElementAction::Parameters* params_);
	};

#ifdef G2O_HAVE_OPENGL
	class G2O_TYPES_SLAM3D_API EdgeSE3LieAlgebraDrawAction :pubilc DrawAction
	{
	public:
		EdgeSE3LieAlgebraDrawAction();
		virtual HyperGraphElementAction* operator() (HyperGraph::HyperGraphElement* element,
			HyperGraphElementAction::Parameters* params_);
	};
#endif
}
