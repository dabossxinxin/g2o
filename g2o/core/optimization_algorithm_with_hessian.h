// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#ifndef G2O_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H
#define G2O_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H

#include "optimization_algorithm.h"
#include "g2o_core_api.h"

namespace g2o {

	class Solver;

	/*!
	*  @brief Guass-Newton,Levenberg-Marquart,Dog-Leg策略实现基类
	*/
	class G2O_CORE_API OptimizationAlgorithmWithHessian : public OptimizationAlgorithm
	{
	public:
		/*!
		*  @brief 显式构造函数，使用已定义的求解器对该类进行初始化
		*  @param[in]	solver	求解器类型
		*/
		explicit OptimizationAlgorithmWithHessian(Solver& solver);

		/*!
		*  @brief 默认析构函数
		*/
		virtual ~OptimizationAlgorithmWithHessian();

		virtual bool init(bool online = false);

		virtual bool computeMarginals(SparseBlockMatrix<MatrixX>& spinv, const std::vector<std::pair<int, int> >& blockIndices);

		virtual bool buildLinearStructure();

		virtual void updateLinearSystem();

		virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges);

		/*!
		*  @brief 获取底层中用于实现线性方程求解的求解器
		*/
		Solver& solver() { return _solver; }

		/**
		 * write debug output of the Hessian if system is not positive definite
		 */
		virtual void setWriteDebug(bool writeDebug);
		virtual bool writeDebug() const { return _writeDebug->value(); }

	protected:
		/*!< @brief 当前使用的线性求解器 */
		Solver& _solver;
		Property<bool>* _writeDebug;

	};

}// end namespace

#endif
