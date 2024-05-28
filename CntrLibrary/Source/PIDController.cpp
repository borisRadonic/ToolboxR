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

#include "PIDController.h"

namespace CntrlLibrary
{

	namespace DiscreteTime
	{
		PIDController::PIDController():Block()
		{
			_pIntegrator = std::make_unique<Integrator>();
			_pDerivative = std::make_unique<Derivative>();

			/*create input and aouput*/
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("PIDin", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("PIDout", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}

		PIDController::PIDController(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t upSaturation)
			: Block(), _Kp(kp), _Ki(ki), _Kd(kd), _Kb(kb), _Ts(ts), _upSat(upSaturation), _isParamsSet(true)
		{
		}

		PIDController::~PIDController()
		{
		}

		//upSaturation used by "back-calculation" antiwindup mechanism
		//Use the Back-calculation coefficient (Kb) parameter to specify the gain of the anti-windup feedback circuit. It is usually satisfactory to set Kb = I, or for controllers with derivative action, Kb = sqrt(I*D)
		void PIDController::setParameters(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t saturation)
		{
			_Kp = kp;
			_Ki = ki;
			_Kd = kd;
			_Kb = kb;
			_Ts = ts;
			_upSat = saturation;
			_loSat = saturation;
			_pIntegrator->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pDerivative->setParameters(_Ts, 1.00);
			_isParamsSet = true;
		}

		void PIDController::setParameters(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t upsat, std::double_t losat)
		{
			_Kp = kp;
			_Ki = ki;
			_Kd = kd;
			_Kb = kb;
			_Ts = ts;
			_upSat = upsat;
			_loSat = losat;
			_pIntegrator->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pDerivative->setParameters(_Ts, 1.00);
			_isParamsSet = true;
		}

		void PIDController::reset()
		{
			_du1 = 0.00;
			_pIntegrator->reset();
			_pDerivative->reset();

		}

		double PIDController::process(std::double_t error)
		{
			_ptrIn->set(error);
			
			if (false == _isParamsSet)
			{
				_ptrOut->set(0.0);
				return 0.0;
			}

			std::double_t P = _Kp * error;
			std::double_t I = _pIntegrator->process(_Ki * (error + (_Kb * _du1)));
			std::double_t D = 0.00;

			if (std::abs(_Kd) >= std::numeric_limits<std::double_t>::min())
			{
				D = _Kd * _pDerivative->process(error);
			}

			std::double_t u = P + I + D;
			std::double_t uSat = u;

			if (u >= 0.00)
			{
				uSat = std::fmin(u, _upSat);
			}
			else
			{
				uSat = std::fmax(u, -_upSat);
			}
			_du1 = uSat - u;
			_ptrOut->set(uSat);
			return uSat;
		}
	}
}