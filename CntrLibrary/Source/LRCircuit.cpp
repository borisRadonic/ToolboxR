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

#include "LRCircuit.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
	using namespace DiscreteTime;

	namespace Models
	{
		LRCircuit::LRCircuit()
		{
			_pIntegratorI = std::make_unique<Integrator>();
		}

		LRCircuit::~LRCircuit()
		{
		}

		void LRCircuit::setParameters(std::double_t ts, std::double_t L, std::double_t R)
		{
			_Ts = ts;
			_L = L;
			_R = R;			
			_pIntegratorI->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_isParamsSet = true;
		}

		void LRCircuit::reset()
		{
			_i1 = 0.00;
			_i = 0.00;
			_u = 0.00;
		}

		void LRCircuit::setInput(std::double_t u)
		{
			_u = u;
		}

		void LRCircuit::process()
		{
			if (_isParamsSet)
			{
				std::double_t i_der = (1.00/_L) * (_u - (_R * _i1) );
				_i = _pIntegratorI->process(i_der);
				_i1 = _i;
			}
		}
	}
}