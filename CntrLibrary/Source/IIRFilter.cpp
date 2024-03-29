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

#include "IIRFilter.h"

#include "Integrator.h"
#include <algorithm>
#include <exception>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{
			IIRFilter::IIRFilter() :Block()
			{
				/*create input and aouput*/
				_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
				_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

				this->addInput(_ptrIn);
				this->addOutput(_ptrOut);
			}

			IIRFilter::~IIRFilter()
			{
			}

			void IIRFilter::setParameters(const std::vector<std::double_t>& numerator, const std::vector<std::double_t>& denominator, const std::string& name)
			{
				if ((numerator.size() == 0U) || (denominator.size() == 0U))
				{
					throw new std::exception("wrong number of coefficients in IIR filer.");
				}

				if (false == _isParamsSet)
				{
					_numerator.insert(_numerator.begin(), numerator.begin(), numerator.end());
					_denominator.insert(_denominator.begin(), denominator.begin(), denominator.end());

					//initialize _x and _y vectors
					for (size_t i = 0; i < denominator.size(); i++)
					{
						_x.push_back(0.00);
						_y.push_back(0.00);
					}
					setName(name);
					_isParamsSet = true;
				}
			}

			void IIRFilter::reset()
			{
				for (size_t i = 0; i < _x.size(); i++)
				{
					_x[i] = 0.00;
					_y[i] = 0.00;
				}
			}


			/*
			The impulse response of the overall system is thus the convolution of the impulse responses
			of the feed-forward and the feed-back system. Accordingly, in terms of z-transforms,
			the overall transfer function is the product of the transfer functions of the two cascade elements

			y(n) = -SUM(from k= 1 to N) (a(k) * y(n-k)   + SUM(from k=0 to m= M) b(k) * x(n-k)
			*/
			double IIRFilter::process(std::double_t u)
			{
				//the number of numerator coefficients
				std::size_t N = _numerator.size();

				//the number of denominator coefficients
				std::size_t M = _denominator.size();
				/*
				The current output sample is computed from the past M inputs and the current input
				sample plus the previous N output samples.
				*/

				_ptrIn->set(u);

				if (_isParamsSet)
				{

					//The first term is the feedback or recursive part of the equation
					std::double_t feedback = 0.00;
					for (size_t i = 1; i < N; i++)
					{
						feedback += _denominator[i] * _y[N - i - 1]; //SUM(from k= 1 to N) (a(k) * y(n-k)
					}

					//shift and update input states
					for (size_t i = 0; i < (M - 1); i++)
					{
						_x[i] = _x[i + 1];
					}
					_x[N - 1] = u; //update

					//second, nonrecursive part is the same as for FIR filter (only inputs)
					std::double_t fir = 0.00;
					for (size_t i = 0; i < M; i++)
					{
						fir += _numerator[i] * _x[M - i - 1];
					}


					_y[N - 1] = fir - feedback; //update output

					//shift and update output
					for (size_t i = 0; i < (N - 1); i++)
					{
						_y[i] = _y[i + 1];
					}

					_ptrOut->set(_y[N - 1]);

					return _y[N - 1];
				}


				_ptrOut->set(0);
				return 0.00;
			}
		}
	}
}