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

		class PMSMotor
		{
		public:

			/*Basic Simulation of Permanent-magnet synchronous motor in DQ domain*/
			PMSMotor();

			virtual ~PMSMotor();

			//Ts - Sample time
			//p - Number of pole pairs
			//b  - Combined friction of rotor and load
			//Kemf - Back EMF constant is the peak voltage induced by the permanent magnet in the per-unit rotational speed of each of the phases. The relationship between the peak permanent magnet flux linkage and the back EMF is: k e = N ψ m )
		 	//J  -  Combined moment of inertia of rotor and load
			//Rs  - Stator resistance in [Ohm]
			//Lq  - Inductance of stator in dq frame (q part) in [H]
			//Ld  - Inductance of stator in dq frame (d part) in [H]
			//Ktq - Torque constant in [A/Nm]
			//Fs - Static friction in Nm

			void setParameters(std::double_t ts, std::uint16_t p, std::double_t b, std::double_t Kemf, std::double_t J, std::double_t Rs, std::double_t Lq, std::double_t Ld, std::double_t Ktq, std::double_t Tsf);

			virtual void reset();

			//uq  - Inpult voltage (q part)
			//ud  - Inpult voltage (d part)
			//lt - Load torque
			void setInputs(std::double_t uq, std::double_t ud, std::double_t lt);

			virtual void process();


			std::double_t getIq() const { return _iq; };

			std::double_t getId() const { return _id; };

			//returns rotor angle
			std::double_t getPos() const { return _angleM; };

			std::double_t getAccell() const { return _aM; };

			std::double_t getVel() const { return _wM; };

			std::double_t getTorque() const { return _tE; };

		protected:

			std::double_t _Ts = 1.00; //sampling period
			std::uint16_t _polePairs = 1U; //The number of motor pol-pairs
			std::double_t _B = 1.00; //Combined friction of rotor and load
			std::double_t _Kemf = 0.00; //Back EMF constant
			std::double_t _invJ = 1.00; //Inverse Combined moment of inertia of rotor and load
			std::double_t _R = 1.00; //Stator resistance in [Ohm]
			std::double_t _Lq = 1.00; //Inductance of stator in dq frame (q part) in [H]
			std::double_t _Ld = 1.00; //Inductance of stator in dq frame(d part) in[H]
			std::double_t _invLq = 1.00; //Inverse inductance of stator in dq frame (q part) in [H]
			std::double_t _invLd = 1.00; //Inverse inductance of stator in dq frame (d part) in [H]
			std::double_t _Ktq = 1.00; //Torque constant in[A / Nm]
			std::double_t _Tsf = 1.00; //static friction of shaft in [Nm]

			bool _isParamsSet = false;

			std::double_t _iq1 = 0.00; //q Current at n-1
			std::double_t _id1 = 0.00; //s Current at n-1

			std::double_t _wM1 = 0.00; //velocity at n-1

			std::double_t _iq = 0.00; //q Current
			std::double_t _id = 0.00; //d Current
						
			std::double_t _aM = 0.00; //acceleration in rad/sec^2
			std::double_t _wM = 0.00; //velocity in rad/sec
			std::double_t _angleE = 0.00; //electrical angle
			std::double_t _angleM = 0.00; //mechanical angle
			
						
			std::double_t _uq = 0.00;
			std::double_t _ud = 0.00;

			//Ts - static friction
			std::double_t  _static_friction = 0.00;

			std::double_t _lt = 0.00; //load torque

			std::double_t _tE = 0.00; //torque

			std::unique_ptr<Integrator> _pIntegratorIq;
			std::unique_ptr<Integrator> _pIntegratorId;			
			std::unique_ptr<Integrator> _pIntegratorW;
			std::unique_ptr<Integrator> _pIntegratorR;

		};
	}
}

