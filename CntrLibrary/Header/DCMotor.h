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
#include <cmath>
#include <memory>
#include "Integrator.h"

namespace CntrlLibrary
{
	namespace Models
	{
		using namespace DiscreteTime;

		class DCMotor final
		{
		public:

			DCMotor();

			~DCMotor();

			//Ts - Sample time
			//B  - Friction
			//Kb - EMF constant
			//J  - Moment of inertia
			//R  - resistance in Ohm
			//L  - induction in H
			//Kt - Torque constant in A/Nm

			void setParameters(std::double_t ts, std::double_t b, std::double_t Kb, std::double_t J, std::double_t R, std::double_t L, std::double_t Kt);

			void reset();

			//u  - DC Voltage
			//lt - Load torque
			void setInputs(std::double_t u, std::double_t lt);

			void process();


			std::double_t getCurrent() const { return _i; };

			std::double_t getVelocity() const { return _w; };

			std::double_t getAccell() const { return _a; };

			std::double_t getTorque() const { return _outTorque; };

		private:

			std::double_t _Ts = 1.00; //sampling period
			std::double_t _B = 1.00;
			std::double_t _Kb = 1.00;
			std::double_t _J = 1.00;
			std::double_t _R = 1.00;
			std::double_t _L = 1.00;
			std::double_t _Kt = 1.00;

			bool _isParamsSet = false;

			std::double_t _i1 = 0.00; //Current at n-1
			std::double_t _w1 = 0.00; //velocity at n-1

			std::double_t _i = 0.00; //Current

			std::double_t _w = 0.00; //velocity in rad/sec
			std::double_t _a = 0.00; //acceleration in rad/sec^2
			std::double_t _outTorque = 0.00;

			std::double_t _u = 0.00;
			std::double_t _lt = 0.00;

			std::unique_ptr<Integrator> _pIntegratorI;
			std::unique_ptr<Integrator> _pIntegratorW;
		};
	}
}
