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
#

namespace CntrlLibrary
{
	//The Coulomband Viscous Friction block models Coulomb(static) and viscous(dynamic) friction.
	//The friction torque is a function of relative velocity and is calculated as a sum of Coulomb and viscous components

	namespace Models
	{

		class FrictionModelCSV final
		{
		public:

			FrictionModelCSV();

			~FrictionModelCSV();

			//Ts - Sample time
			//B  - Friction
			//Tc - Coulomb friction torque
			//Tbrk  - Breakaway friction torque
			//Wbrk  - Breakaway friction velocity
			//J			- Moment of inertia
			void setParameters(std::double_t ts, std::double_t b, double_t c0, std::double_t J);

			void reset();

			//w  - velocity
			//ti - input torque
			//a  - acceleration
			void setInputs(std::double_t w, std::double_t ti, std::double_t a);

			void process();

			std::double_t getFrictionTorque() const { return _T; };

		private:

			std::double_t _Ts = 1.00; //sampling period
			std::double_t _B = 0.00;	//Viscous friction coefficient
			std::double_t _C0 = 0.00; //Coulomb friction coefficient
			std::double_t _J = 0.00; //Moment of inertia


			bool _isParamsSet = false;

			std::double_t _w = 0.00; //velocity in rad/sec
			std::double_t _a = 0.00; //acceleration in rad/sec^2
			std::double_t _Ti = 0.00; //Input torque
			std::double_t _T = 0.00; //Friction torque

		};
	}
}

