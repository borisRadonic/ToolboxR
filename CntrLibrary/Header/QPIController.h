
/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2024 Boris Radonic

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

#ifndef QPICONTROLLER_H
#define QPICONTROLLER_H

#include "QIntegrator.h"
#include "MathFunctions.h"

#include <cstdint>
#include <algorithm> // For std::min etc.
#include <array>
#include <cmath>
#include <limits>
#include <numbers>


namespace CntrlLibrary
{
	template <typename TYPE_OUT, typename TYPE_IN, typename TYPE_KP, typename TYPE_KI, typename TYPE_KB>
	class QPIController
	{
		public:

			explicit QPIController(float kp, float ki, float kb, float ts, float upsat)
				:Kp(kp)
				,Kb(kb)
				,Ts(ts)
				,upSat(upsat)
				,integrator(ts,ki, -upsat, upsat)
			{
			}

			~QPIController() = default;
						

			void reset()
			{
				du1 = 0.00;
				integrator.reset();
			}

			TYPE_OUT process(TYPE_IN error)
			{
				
				auto propPart = FixedPointOps::mul(Kp, error);
				auto inputI = FixedPointOps::add(error, FixedPointOps::mul(Kb, du1) );

				using ResultType = decltype(inputI);
								
				
				FixedPoint< ResultType::IntegerBitsValue,
							ResultType::FractionalBitsValue,
							typename ResultType::UnderlyingType,
							typename ResultType::MultiplierType> resultI;
				

				//auto resultI;
				TYPE_OUT inputInt;
				FixedPointOps::convert(inputI, inputInt);
				auto intPart = integrator.process(inputInt);
				TYPE_OUT intPartO;
				FixedPointOps::convert(intPart, intPartO);

				TYPE_OUT propPartO;
				FixedPointOps::convert(propPart, propPartO);

				auto uout = intPartO + propPartO;

				TYPE_OUT u;
				FixedPointOps::convert(uout, u);
				
				TYPE_OUT uSat = u;
				
				if (uSat >= TYPE_OUT(0.0f) )
				{
					uSat = std::min<TYPE_OUT>(u, upSat);
				}
				else
				{
					uSat = std::max<TYPE_OUT>(u, TYPE_OUT(0.0f) - upSat);
				}
				du1 = uSat - u;
				
				FixedPointOps::convert(error, du1);
				return uSat;
			}

		private:

			TYPE_KP Kp{}; //proportional gain coefficient
			TYPE_KB Kb{}; //anti-windup gain coefficient.
			std::int32_t Ts{}; //sampling period
			TYPE_OUT upSat{}; //Output saturation upper limit
			TYPE_OUT du1{0.0f};

			QIntegrator<Q10_22, TYPE_KI> integrator;
	};
}
#endif
