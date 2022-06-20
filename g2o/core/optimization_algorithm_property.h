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

#ifndef G2O_OPTIMIZATION_ALGORITHM_PROPERTY_H
#define G2O_OPTIMIZATION_ALGORITHM_PROPERTY_H

#include <string>

#include "g2o_core_api.h"

namespace g2o {

	/**
	 * \brief 描述非线性求解方法的属性
	 */
	struct G2O_CORE_API OptimizationAlgorithmProperty
	{
		/*!< @brief 非线性求解方法的名称 */
		std::string name;
		/*!< @brief 非线性求解方法的简单描述 */
		std::string desc;
		/*!< @brief 法方程求解方法：CSparse Cholesky、PCG */
		std::string type;
		/*!< @brief 是否需要对LandMark边缘化 */
		bool requiresMarginalize;
		/*!< @brief 非线性优化中位姿的维度 */
		int poseDim;
		/*!< @brief 非线性优化中LandMark的维度 */
		int landmarkDim;

		/*!
		*  @brief 非线性优化方法属性默认构造函数
		*/
		OptimizationAlgorithmProperty() :
			name(), desc(), type(), requiresMarginalize(false), poseDim(-1), landmarkDim(-1) {}

		/*!
		*  @brief 非线性优化方法属性默认构造函数
		*  @param[in]	name_	非线性方法名称
		*  @param[in]	desc_	非线性方法描述
		*  @param[in]	type_	法方程求解方法
		*  @param[in]	requiresMarginalize_	是否需要LandMark边缘化
		*  @param[in]	poseDim_				位姿维度
		*  @param[in]	landmarkDim_			路标点维度
		*/
		OptimizationAlgorithmProperty(const std::string& name_, const std::string& desc_, const std::string& type_, bool requiresMarginalize_, int poseDim_, int landmarkDim_) :
			name(name_), desc(desc_), type(type_), requiresMarginalize(requiresMarginalize_), poseDim(poseDim_), landmarkDim(landmarkDim_)
		{
		}
	};

} // end namespace

#endif