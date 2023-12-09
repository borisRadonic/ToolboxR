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
#include <cmath>
#include <memory>
#include "Integrator.h"
#include "Derivative.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{

		class PIDController : public Block
		{
		public:

			PIDController();

			PIDController(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t upSaturation);

			~PIDController();

			void setParameters(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t upSaturation);

			void reset();

			double process(std::double_t error);

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::double_t _Kp = 0.00; //proportional gain coefficient
			std::double_t _Ki = 0.00; //integral gain coefficient
			std::double_t _Kd = 0.00; //differencial gain coefficient
			std::double_t _Kb = 0.00; //anti-windup gain coefficient.
			std::double_t _Ts = 1.00; //sampling period
			std::double_t _upSat = 0.00; //Output saturation upper limit



			std::double_t _du1 = 0.00;

			bool _isParamsSet = false;

			std::double_t iii = 0.00;

			std::unique_ptr<Integrator> _pIntegrator;
			std::unique_ptr<Derivative> _pDerivative;

		};
	}
}