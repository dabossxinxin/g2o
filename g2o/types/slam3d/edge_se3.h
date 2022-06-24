// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_EDGE_SE3_H_
#define G2O_EDGE_SE3_H_

#include "g2o/core/base_binary_edge.h"

#include "g2o_types_slam3d_api.h"
#include "vertex_se3.h"

namespace g2o {

	/*!
	*  @brief 两个位姿顶点之间的边，使用Eigen中的Isometry表示位姿变换
	*  @detail 两个位姿顶点之间的位姿变换使用Isometry3形式表示
	*          若使用z表示测量值吗，那么误差使用z^-1*(x_i^-1*x_j)表示
	*/
	class G2O_TYPES_SLAM3D_API EdgeSE3 : public BaseBinaryEdge<6, Isometry3, VertexSE3, VertexSE3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		/*!
		*  @brief 默认构造函数
		*/
		EdgeSE3();

		/*!
		*  @brief 使用std中定义的输入流读取数据
		*/
		virtual bool read(std::istream& is);

		/*!
		*  @brief 使用std中定义的输出流读取数据
		*/
		virtual bool write(std::ostream& os) const;

		/*!
		*  @brief 计算当前边的残差
		*/
		void computeError();

		/*!
		*  @brief 设置当前边的测量值
		*  @param[in]	m	当前边的测量值
		*/
		virtual void setMeasurement(const Isometry3& m)
		{
			_measurement = m;
			_inverseMeasurement = m.inverse();
		}

		/*!
		*  @brief 设置当前边的测量值
		*  @param[in]	m		当前边的测量值的“四元数+平移”形式
		*  @return		bool	是否成功设定当前边的测量值
		*/
		virtual bool setMeasurementData(const number_t* d)
		{
			Eigen::Map<const Vector7> v(d);
			setMeasurement(internal::fromVectorQT(v));
			return true;
		}

		/*!
		*  @brief 获取当前边的测量值的“四元数+平移”形式
		*  @param[in]	m		当前边的测量值的“四元数+平移”形式
		*  @return		bool	是否成功获取当前边的测量值
		*/
		virtual bool getMeasurementData(number_t* d) const
		{
			Eigen::Map<Vector7> v(d);
			v = internal::toVectorQT(_measurement);
			return true;
		}

		/*!
		*  @brief 计算当前边相对于各个相连顶点的雅克比
		*/
		void linearizeOplus();

		/*!
		*  @brief 获取当前测量值的维度:"四元数+平移=7"
		*/
		virtual int measurementDimension() const
		{
			return 7;
		}

		/*!
		*  @brief TODO
		*/
		virtual bool setMeasurementFromState();

		/*!
		*  @brief TODO
		*/
		virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
			OptimizableGraph::Vertex* /*to*/)
		{
			return 1.;
		}

		/*!
		*  @brief 初始化相连顶点参数估计值
		*/
		virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

	protected:
		Isometry3 _inverseMeasurement;
	};

	/**
	 * \brief Pose-Pose constraint to Gnuplot data file
	 */
	class G2O_TYPES_SLAM3D_API EdgeSE3WriteGnuplotAction : public WriteGnuplotAction {
	public:
		EdgeSE3WriteGnuplotAction();
		virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
			HyperGraphElementAction::Parameters* params_);
	};

#ifdef G2O_HAVE_OPENGL
	/**
	 * \brief Visualize a 3D pose-pose constraint
	 */
	class G2O_TYPES_SLAM3D_API EdgeSE3DrawAction : public DrawAction {
	public:
		EdgeSE3DrawAction();
		virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
			HyperGraphElementAction::Parameters* params_);
	};
#endif

} // end namespace
#endif