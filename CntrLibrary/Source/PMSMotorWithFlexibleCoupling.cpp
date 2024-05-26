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
#include "PMSMotorWithFlexibleCoupling.h"

#define _USE_MATH_DEFINES
#include <math.h>


namespace CntrlLibrary
{
	using namespace DiscreteTime;

	namespace Models
	{
		PMSMotorWithFlexibleCoupling::PMSMotorWithFlexibleCoupling():PMSMotor()
		{
			_pSOsys = std::make_unique<SOSystem>();
			_pIntegratorLW = std::make_unique<Integrator>();
			_pIntegratorLR = std::make_unique<Integrator>();
		}

		PMSMotorWithFlexibleCoupling::~PMSMotorWithFlexibleCoupling()
		{
		}

		void PMSMotorWithFlexibleCoupling::setLoadAndCouplingParameters( std::double_t c_damp, std::double_t c_tor_stif)
		{			
			_c_damp = c_damp;
			_c_tor_stif = c_tor_stif;

			if (_isParamsSet)
			{
				_isCouplingParamsSet = true;

				_D = _c_damp / (2 * (1.0 / _invJ));
				_omega0 = sqrt(_c_tor_stif / (1.0 / _invJ) );

				std::double_t pa1(0.0);
				std::double_t pa2(0.0);

				if (_D < 1.0)
				{
					//the system is underdamped
					pa1 = -_D * _omega0 + _omega0 * sqrt(abs(_D * _D - 1));
					pa2 = -_D * _omega0 - _omega0 * sqrt(abs(_D * _D - 1));
				}
				else if (_D == 1.0)
				{

					//the system is critically damped
					pa1 = -_omega0;
					pa2 = -_omega0;
				}
				else
				{
					//the system is overdamped
					pa1 = -_D * _omega0 + _omega0 * sqrt(_D * _D - 1);
					pa2 = -_D * _omega0 - _omega0 * sqrt(_D * _D - 1);
				}

				std::double_t p1 = exp(pa1 * _Ts);
				std::double_t p2 = exp(pa2 * _Ts);
				std::double_t b0 = 1.0;
				std::double_t b1 = 2.0;
				std::double_t b2 = 1.0;

				std::double_t a1 = -(p1 + p2);
				std::double_t a2 = p1 * p2;

				std::double_t k = abs((1.0 + a1 + a2) / (b0 + b1 + b2));

				_pSOsys->setParameters(k, a1, a2, b0, b1, b2);
			}
		}

		void PMSMotorWithFlexibleCoupling::reset()
		{
			PMSMotor::reset();
			_pSOsys->reset();
			_aL		= 0.00;
			_wL		= 0.00;
			_angleL = 0.00;
		}
		
		void PMSMotorWithFlexibleCoupling::process()
		{
			PMSMotor::process();
			if (_isCouplingParamsSet)
			{
				_aL		= _pSOsys->process(_invJ * (_Ktq * _iq - _static_friction - _B * _wM1 - _lt) );
				_wL		= _pIntegratorLW->process(_aL);
				_angleL = _pIntegratorLW->process(_aL);
			}
		}
	}
}


