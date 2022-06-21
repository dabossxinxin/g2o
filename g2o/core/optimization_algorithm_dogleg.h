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

#ifndef G2O_OPTIMIZATION_ALGORITHM_DOGLEG_H
#define G2O_OPTIMIZATION_ALGORITHM_DOGLEG_H

#include "optimization_algorithm_with_hessian.h"
#include "g2o_core_api.h"

#include <memory>

namespace g2o {

	class BlockSolverBase;

	/*!
	*  @brief Levenberg_Marquart策略实现类
	*/
	class G2O_CORE_API OptimizationAlgorithmDogleg : public OptimizationAlgorithmWithHessian
	{
	public:
		/*!< @brief 步长类型 */
		enum {
			/*!< @brief 未定义的步长 */
			STEP_UNDEFINED,
			/*!< @brief 最速下降步长 */
			STEP_SD,
			/*!< @brief 高斯牛顿步长 */
			STEP_GN,
			/*!< @brief DogLeg步长 */
			STEP_DL
		};

	public:

		/*!
		*  @brief 使用指定的求解器初始化Dog-Leg策略
		*  @param[in]	solver	用户指定的求解器
		*/
		explicit OptimizationAlgorithmDogleg(std::unique_ptr<BlockSolverBase> solver);
		virtual ~OptimizationAlgorithmDogleg();

		/*!
		*  @brief 当前迭代的求解优化问题
		*  @param[in]	iteration	当前迭代
		*  @param[in]	online		TODO
		*/
		virtual SolverResult solve(int iteration, bool online = false);

		/*!
		*  @brief 打印优化迭代过程中的调试信息
		*  @param[in]	os	信息输出流
		*/
		virtual void printVerbose(std::ostream& os) const;

		/*!
		*  @brief 获取迭代中最后一步采用的步长类型
		*  @return	int	步长类型
		*/
		int lastStep() const { return _lastStep; }
		//! return the diameter of the trust region

		/*!
		*  @brief 获取当前迭代中置信域大小
		*  @return	number_t	置信域大小
		*/
		number_t trustRegion() const { return _delta; }

		/*!
		*  @brief 将迭代类型转化为字符串类型
		*  @param[in]	stepType	步长类型（整型）
		*  @return		char*		步长类型（字符串）
		*/
		static const char* stepType2Str(int stepType);

	protected:
		/*!< @brief 一次迭代中使chi2下降的最大尝试次数 */
		Property<int>* _maxTrialsAfterFailure;
		/*!< @brief 用户设置的置信域初始值 */
		Property<number_t>* _userDeltaInit;
		/*!< @brief 用来保证Hessian矩阵为正定矩阵的初始阻尼因子 */
		Property<number_t>* _initialLambda;
		/*!< @brief 用来保证Hessian矩阵为正定矩阵的阻尼因子 */
		Property<number_t>* _lamdbaFactor;

		/*!< @brief 当前迭代最速下降步长 */
		VectorX _hsd;
		/*!< @brief 当前迭代DogLeg下降步长 */
		VectorX _hdl;
		VectorX _auxVector;   ///< auxilary vector used to perform multiplications or other stuff

		/*!< @brief 当前迭代阻尼因子 */
		number_t _currentLambda;
		/*!< @brief 当前迭代置信域直径 */
		number_t _delta;
		/*!< @brief 优化程序采用的步长类型 */
		int _lastStep;
		bool _wasPDInAllIterations;   ///< the matrix we solve was positive definite in all iterations -> if not apply damping
		/*!< @brief 当前迭代为使chi2下降的尝试次数 */
		int _lastNumTries;

	private:
		std::unique_ptr<BlockSolverBase> m_solver;
	};

} // end namespace

#endif
