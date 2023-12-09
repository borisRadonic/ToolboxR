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

#include "DCMotor.h"
#include <memory>

namespace CntrlLibrary
{
	using namespace DiscreteTime;

	namespace Models
	{
		DCMotor::DCMotor()
		{
			_pIntegratorI = std::make_unique<Integrator>();
			_pIntegratorW = std::make_unique<Integrator>();
		}

		DCMotor::~DCMotor()
		{
		}

		void DCMotor::setParameters(std::double_t ts, std::double_t b, std::double_t Kb, std::double_t J, std::double_t R, std::double_t L, std::double_t Kt)
		{
			_Ts = ts;
			_B = b;
			_Kb = Kb;
			_J = J;
			_R = R;
			_L = L;
			_Kt = Kt;
			_pIntegratorI->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorW->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_isParamsSet = true;
		}

		void DCMotor::reset()
		{
			_i1 = 0.00;
			_w1 = 0.00;
			_i = 0.00;
			_w = 0.00;
			_a = 0.00;
			_outTorque = 0.00;
		}

		void DCMotor::setInputs(std::double_t u, std::double_t lt)
		{
			_u = u;
			_lt = lt;
		}

		void DCMotor::process()
		{
			if (_isParamsSet)
			{
				std::double_t i_d = (1.00 / _L) * (_u - (_Kb * _w1) - (_R * _i1));
				_i = _pIntegratorI->process(i_d);
				_i1 = _i;
				_a = (1.00 / _J) * (_Kt * _i - _B * _w1 - _lt);
				_w = _pIntegratorW->process(_a);
				_w1 = _w;
				_outTorque = _Kt * _i;
			}
		}
	}
}