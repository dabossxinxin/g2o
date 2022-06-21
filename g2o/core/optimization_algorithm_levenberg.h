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

#ifndef G2O_SOLVER_LEVENBERG_H
#define G2O_SOLVER_LEVENBERG_H

#include "optimization_algorithm_with_hessian.h"
#include "g2o_core_api.h"

#include <memory>

namespace g2o {

	/*!
	*  @brief Levenberg_Marquart策略实现类
	*/
	class G2O_CORE_API OptimizationAlgorithmLevenberg : public OptimizationAlgorithmWithHessian
	{
	public:
		/*!
		*  @brief 使用指定的求解器初始化Levenberg_Marquart
		*  @param[in]	solver	用户指定的求解器
		*/
		explicit OptimizationAlgorithmLevenberg(std::unique_ptr<Solver> solver);

		/*!
		*  @brief 默认析构函数
		*/
		virtual ~OptimizationAlgorithmLevenberg();

		virtual SolverResult solve(int iteration, bool online = false);

		/*!
		*  @brief 打印LM算法中的调试信息
		*  @param[in]	os	输出流
		*/
		virtual void printVerbose(std::ostream& os) const;

		/*!
		*  @brief 获取当前迭代步骤的阻尼因子lambda
		*  @return	number_t	当前阻尼因子
		*/
		number_t currentLambda() const { return _currentLambda; }

		/*!
		*  @brief 设置一次迭代中连续使得chi2上升的最大次数
		*  @param[in]	max_trials	最大尝试次数
		*/
		void setMaxTrialsAfterFailure(int max_trials);

		/*!
		*  @brief 获取一次迭代中连续使得chi2上升的最大次数
		*  @return	max_trials	最大尝试次数
		*/
		int maxTrialsAfterFailure() const { return _maxTrialsAfterFailure->value(); }

		/*!
		*  @brief 返回用户设定的初始阻尼因子
		*  @detail 若用户设定的初始阻尼因子小于0，那么算法内部将自动计算一个合理的初始值
		*  @param[in]	lambda	初始阻尼因子值
		*/
		number_t userLambdaInit() { return _userLambdaInit->value(); }

		/*!
		*  @brief 用户设定初始的阻尼因子接口
		*  @detail 若用户并没有调用该接口，那么算法内部将自动计算一个合理的初始值
		*  @param[in]	lambda	初始阻尼因子值
		*/
		void setUserLambdaInit(number_t lambda);

		/*!
		*  @brief 获取Levenberg-Marquart方法的迭代次数
		*  @return	int	LM策略迭代次数
		*/
		int levenbergIteration() { return _levenbergIterations; }

	protected:
		/*!< @brief 一次迭代中使chi2下降的最大尝试次数 */
		Property<int>* _maxTrialsAfterFailure;
		/*!< @brief 用户设定的初始阻尼因子值 */
		Property<number_t>* _userLambdaInit;
		/*!< @brief 当前迭代过程阻尼因子值 */
		number_t _currentLambda;
		/*!< @brief 阻尼因子控制参数 */
		number_t _tau;
		/*!< @brief lower bound for lambda decrease if a good LM step */
		number_t _goodStepLowerScale;
		/*!< @brief upper bound for lambda decrease if a good LM step */
		number_t _goodStepUpperScale;
		/*!< @brief 阻尼因子控制参数 */
		number_t _ni;
		/*!< @brief LM策略迭代次数 */
		int _levenbergIterations;

		/*!
		*  @brief 算法内部用于计算初始阻尼因子的接口
		*  @detail 若用户并没有调用该接口，那么算法内部将自动计算一个合理的初始值
		*  @return	number_t	初始阻尼因子值
		*/
		number_t computeLambdaInit() const;
		number_t computeScale() const;

	private:
		std::unique_ptr<Solver> m_solver;
	};

} // end namespace

#endif
