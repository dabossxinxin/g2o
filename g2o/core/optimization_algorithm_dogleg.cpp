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

#include "optimization_algorithm_dogleg.h"

#include <iostream>

#include "g2o/stuff/timeutil.h"

#include "block_solver.h"
#include "sparse_optimizer.h"
#include "solver.h"
#include "batch_stats.h"
using namespace std;

namespace g2o {

	OptimizationAlgorithmDogleg::OptimizationAlgorithmDogleg(std::unique_ptr<BlockSolverBase> solver)
		: OptimizationAlgorithmWithHessian(*solver.get()),
		m_solver{ std::move(solver) }
	{
		_userDeltaInit = _properties.makeProperty<Property<number_t>>("initialDelta", (number_t)100);
		_maxTrialsAfterFailure = _properties.makeProperty<Property<int>>("maxTrialsAfterFailure", 100);
		_initialLambda = _properties.makeProperty<Property<number_t>>("initialLambda", (number_t)1e-7);
		_lamdbaFactor = _properties.makeProperty<Property<number_t>>("lambdaFactor", 10.);
		_delta = _userDeltaInit->value();
		_lastStep = STEP_UNDEFINED;
		_wasPDInAllIterations = true;
	}

	OptimizationAlgorithmDogleg::~OptimizationAlgorithmDogleg()
	{}

	OptimizationAlgorithm::SolverResult OptimizationAlgorithmDogleg::solve(int iteration, bool online)
	{
		/* 检查成员变量_optimizer的合法性 */
		assert(_optimizer && "_optimizer not set");
		assert(_solver.optimizer() == _optimizer && "underlying linear solver operates on different graph");

		BlockSolverBase& blockSolver = static_cast<BlockSolverBase&>(_solver);

		if (iteration == 0 && !online)
		{ // built up the CCS structure, here due to easy time measure
			bool ok = _solver.buildStructure();
			if (!ok) {
				cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
				return OptimizationAlgorithm::Fail;
			}

			/* 使用当前求解器中的成员维度初始化当前类中的成员 */
			_hsd.resize(_solver.vectorSize());
			_hdl.resize(_solver.vectorSize());
			_auxVector.resize(_solver.vectorSize());
			_delta = _userDeltaInit->value();
			_currentLambda = _initialLambda->value();
			_wasPDInAllIterations = true;
		}

		/* 计算鲁棒核函数加持下当前迭代总体chi2 */
		number_t t = get_monotonic_time();
		_optimizer->computeActiveErrors();
		G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
		if (globalStats) {
			globalStats->timeResiduals = get_monotonic_time() - t;
			t = get_monotonic_time();
		}
		number_t currentChi = _optimizer->activeRobustChi2();

		/* 构造当前迭代的线性方程 */
		_solver.buildSystem();
		if (globalStats) {
			globalStats->timeQuadraticForm = get_monotonic_time() - t;
		}

		VectorX::ConstMapType b(_solver.b(), _solver.vectorSize());

		/* 计算alpha */
		_auxVector.setZero();
		blockSolver.multiplyHessian(_auxVector.data(), _solver.b());
		number_t bNormSquared = b.squaredNorm();
		number_t alpha = bNormSquared / _auxVector.dot(b);

		/* 计算最速下降步长 */
		_hsd = alpha * b;
		number_t hsdNorm = _hsd.norm();

		number_t hgnNorm = -1.;
		bool solvedGaussNewton = false;
		bool goodStep = false;
		int& numTries = _lastNumTries;
		numTries = 0;
		do {
			++numTries;

			if (!solvedGaussNewton) {
				const number_t minLambda = cst(1e-12);
				const number_t maxLambda = cst(1e3);
				solvedGaussNewton = true;
				// apply a damping factor to enforce positive definite Hessian, if the matrix appeared
				// to be not positive definite in at least one iteration before.
				// We apply a damping factor to obtain a PD(positive definition) matrix.
				bool solverOk = false;
				while (!solverOk)
				{
					if (!_wasPDInAllIterations)
						_solver.setLambda(_currentLambda, true);   // add _currentLambda to the diagonal
					solverOk = _solver.solve();
					if (!_wasPDInAllIterations)
						_solver.restoreDiagonal();				   // restore original hessian matrix
					_wasPDInAllIterations = _wasPDInAllIterations && solverOk;
					if (!_wasPDInAllIterations) {
						// simple strategy to control the damping factor
						if (solverOk) {
							_currentLambda = std::max(minLambda, _currentLambda / (cst(0.5) * _lamdbaFactor->value()));
						}
						else {
							_currentLambda *= _lamdbaFactor->value();
							if (_currentLambda > maxLambda) {
								_currentLambda = maxLambda;
								return Fail;
							}
						}
					}
				}
				if (!solverOk) {
					return Fail;
				}
				hgnNorm = VectorX::ConstMapType(_solver.x(), _solver.vectorSize()).norm();
			}

			VectorX::ConstMapType hgn(_solver.x(), _solver.vectorSize());
			assert(hgnNorm >= 0. && "Norm of the GN step is not computed");

			if (hgnNorm < _delta) {
				_hdl = hgn;
				_lastStep = STEP_GN;
			}
			else if (hsdNorm > _delta) {
				_hdl = _delta / hsdNorm * _hsd;
				_lastStep = STEP_SD;
			}
			else {
				_auxVector = hgn - _hsd;  // b - a
				number_t c = _hsd.dot(_auxVector);
				number_t bmaSquaredNorm = _auxVector.squaredNorm();
				number_t beta;
				if (c <= 0.)
					beta = (-c + sqrt(c*c + bmaSquaredNorm * (_delta*_delta - _hsd.squaredNorm()))) / bmaSquaredNorm;
				else {
					number_t hsdSqrNorm = _hsd.squaredNorm();
					beta = (_delta*_delta - hsdSqrNorm) / (c + sqrt(c*c + bmaSquaredNorm * (_delta*_delta - hsdSqrNorm)));
				}
				assert(beta > 0. && beta < 1 && "Error while computing beta");
				_hdl = _hsd + beta * (hgn - _hsd);
				_lastStep = STEP_DL;
				assert(_hdl.norm() < _delta + 1e-5 && "Computed step does not correspond to the trust region");
			}

			// compute the linear gain
			_auxVector.setZero();
			blockSolver.multiplyHessian(_auxVector.data(), _hdl.data());
			number_t linearGain = -1 * (_auxVector.dot(_hdl)) + 2 * (b.dot(_hdl));

			// apply the update and see what happens
			_optimizer->push();
			_optimizer->update(_hdl.data());
			_optimizer->computeActiveErrors();
			number_t newChi = _optimizer->activeRobustChi2();
			number_t nonLinearGain = currentChi - newChi;
			if (fabs(linearGain) < 1e-12)
				linearGain = cst(1e-12);
			number_t rho = nonLinearGain / linearGain;
			//cerr << PVAR(nonLinearGain) << " " << PVAR(linearGain) << " " << PVAR(rho) << endl;
			if (rho > 0) { // step is good and will be accepted
				_optimizer->discardTop();
				goodStep = true;
			}
			else { // recover previous state
				_optimizer->pop();
			}

			// update trust region based on the step quality
			if (rho > 0.75)
				_delta = std::max<number_t>(_delta, 3 * _hdl.norm());
			else if (rho < 0.25)
				_delta *= 0.5;
		} while (!goodStep && numTries < _maxTrialsAfterFailure->value());
		if (numTries == _maxTrialsAfterFailure->value() || !goodStep)
			return Terminate;
		return OK;
	}

	void OptimizationAlgorithmDogleg::printVerbose(std::ostream& os) const
	{
		os
			<< "\t Delta= " << _delta
			<< "\t step= " << stepType2Str(_lastStep)
			<< "\t tries= " << _lastNumTries
			<< "\t delta_x= " << _solver.x_norm();
		if (!_wasPDInAllIterations)
			os << "\t lambda= " << _currentLambda;
	}

	const char* OptimizationAlgorithmDogleg::stepType2Str(int stepType)
	{
		switch (stepType) {
		case STEP_SD: return "Descent";
		case STEP_GN: return "Gauss-Newton";
		case STEP_DL: return "Dog-Leg";
		default: return "Undefined";
		}
	}
} // end namespace
