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

#include "PMSMotor.h"

#define _USE_MATH_DEFINES
#include <math.h>


namespace CntrlLibrary
{
	using namespace DiscreteTime;

	namespace Models
	{
		PMSMotor::PMSMotor()
		{
			_pIntegratorIq = std::make_unique<Integrator>();
			_pIntegratorId = std::make_unique<Integrator>();
			_pIntegratorW = std::make_unique<Integrator>();
			_pIntegratorR = std::make_unique<Integrator>();
		}

		PMSMotor::~PMSMotor()
		{
		}

		void PMSMotor::setParameters(std::double_t ts, std::uint16_t p, std::double_t b, std::double_t Kemf, std::double_t J, std::double_t Rs, std::double_t Lq, std::double_t Ld, std::double_t Ktq, std::double_t Tsf)
		{
			_Ts = ts;					
			_polePairs = p;
			_B = b;
			_Kemf = Kemf;
			_invJ = 1.00 / J; //Inverse inertia
			_R = Rs;
			_Lq = Lq;
			_Ld = Ld;
			_invLq = 1.00/Lq;
			_invLd = 1.00/Ld;
			_Ktq = Ktq;
			_Tsf = Tsf;
			_pIntegratorIq->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorId->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorW->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorR->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_isParamsSet = true;
		}

		void PMSMotor::reset()
		{
			_iq1 = 0.00;
			_id1 = 0.00;
			_wM1 = 0.00;
			_iq = 0.00;
			_id = 0.00;
			_angleE = 0.00;
			_angleM = 0.00;
			_wM = 0.00;
			_aM = 0.00;
			_tE = 0.00;
			_uq = 0.00;
			_ud = 0.00;
			_static_friction = 0.00;
		}

		void PMSMotor::setInputs(std::double_t uq, std::double_t ud, std::double_t lt)
		{
			_uq = uq;
			_ud = ud;
			_lt = lt;
		}

		void PMSMotor::process()
		{
			if (_isParamsSet)
			{
				std::double_t id_der = _invLd * (_ud - (_R * _id1) + _polePairs * _wM * _Lq * _iq1);
				std::double_t iq_der = _invLq * ( _uq - (_R * _iq1) - _wM * (_polePairs * _Ld * _id1 + _Kemf) );
				_id = _pIntegratorId->process(id_der);
				_iq = _pIntegratorIq->process(iq_der);
							
				
				//Tc - Coulomb or kinetic friction
				
				std::double_t  tc = 0.00;				
				
				std::double_t  signW = 1.00;
				std::double_t  signT = 1.00;

				if (std::signbit(_wM1))
				{
					//negative velocity
					signW = -1.00;
				}

				if (std::signbit(_iq))
				{
					//negative input torque
					signT = -1.00;
				}

				if (std::abs(_aM) < std::numeric_limits<std::double_t>::min())
				{
					// acceleration is nearly zero
					if (abs( (_iq * _Ktq) - ( (1.00/ _invJ) * _aM)) < _Tsf)
					{
						_static_friction = _iq * _Ktq;
					}
					else
					{
						_static_friction = _Tsf * signW;
					}
				}
				else
				{
					if (abs(_iq * _Ktq) < _Tsf)
					{
						_static_friction = _iq * _Ktq;
					}
					else
					{
						_static_friction = _Tsf * signT;
					}
				}

				if (std::abs(_wM) >= std::numeric_limits<std::double_t>::min())
				{
					_static_friction = 0.00;
				}
				
				_aM = _invJ * (_Ktq * _iq - _static_friction - _B * _wM1 - _lt);

				_wM = _pIntegratorW->process(_aM);
				_angleM = _pIntegratorR->process(_wM);
				double angleE = _angleM * _polePairs;
				_angleE = angleE;
				_iq1 = _iq;
				_id1 = _id;
				_wM1 = _wM;
				_tE = _Ktq * _iq;
			}
		}
	}
}
