/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <memory>
#include "Block.h"
#include <Eigen/Core>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		/*
		A must be an N-by-N matrix, where n is the number of states. -> State transition matrix
		B must be an N-by-M matrix, where m is the number of inputs.
		C must be an R-by-N matrix, where r is the number of outputs.
		D must be an R-by-M matrix, where r is the number of outputs
		*/

		template< std::uint32_t N, std::uint32_t M, std::uint32_t R>
		class StateSpace final : public Block
		{
		public:


			using AMatrix = Eigen::Matrix<double, N, N>;
			using BMatrix = Eigen::Matrix<double, N, M>;
			using CMatrix = Eigen::Matrix<double, R, N>;
			using DMatrix = Eigen::Matrix<double, R, M>;

			StateSpace() {};

			~StateSpace() {};

			void setParameters(const AMatrix& A, const BMatrix& B, const CMatrix& C, const DMatrix& D, const std::string& name)
			{
				if (false == _isParamsSet)
				{
					_A = A;
					_B = B;
					_C = C;
					_D = D;

					setName(name);
					_isParamsSet = true;
					reset();
				}
			}

			void setInput(Eigen::Vector<double, M>& U)
			{
				_U = U;
			}

			void reset()
			{
				if (_isParamsSet)
				{
					_X.setZero();
					_X1.setZero();
					_Y.setZero();
					_U.setZero();
				}
			}

			void process()
			{
				if (_isParamsSet)
				{
					_X = _A * _X1 + _B * _U;
					_Y = _C * _X + _D * _U;
					_X1 = _X;
				}
			}

			const Eigen::Vector<double, R>& getY() const
			{
				return _Y;
			}

		private:


			std::shared_ptr<Signal<Eigen::Vector<double, M>>> _ptrIn;
			std::shared_ptr<Signal<Eigen::Vector<double, R>>> _ptrOut;


			Eigen::Vector<double, N> _X;
			Eigen::Vector<double, N> _X1;
			Eigen::Vector<double, R> _Y;
			Eigen::Vector<double, M> _U;

			AMatrix _A;
			BMatrix _B;
			CMatrix _C;
			DMatrix _D;

			bool _isParamsSet = false;
		};
	}
}



