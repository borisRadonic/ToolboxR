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
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		/*! \brief The discrete time derivative block.
		*
		* \f$ y(t_n)= K  \frac{u(t_n)-u(t_n_-_1)} {T_s} \f$
		* where
		* \f$ u(t_n) \f$ and \f$ y(t_n) \f$  are the input and output at the current time step, respectively.
		* \f$ u(t_n_-_1) \f$ is the input previous time step.
		* \f$ K \f$ is the gain.
		* \f$ T_s \f$ is the discrete step size.
		* 
		*/ 
		class Derivative final : public Block
		{
		public:

			/**
			 * Constructor
			 */
			Derivative();

			/**
			* Destructor
			*/
			~Derivative();

			void setParameters(std::double_t ts, std::double_t gain, const std::string& name = "");

			void setSaturation(std::double_t min, std::double_t max);

			inline void setInitialConditions(std::double_t ic)
			{
				_old = ic;
			}

			void reset();

			double process(std::double_t u);

			bool getIsSaturationUsed() const
			{
				return _isUseSaturation;
			}

			std::int32_t getSaturate() const
			{
				return _saturate;
			}
		protected:

			void checkSaturation(std::double_t val);

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::double_t _gain = 1.00; //value to multiply with integrator input
			std::double_t _Ts = 1.00; //sampling period

			std::double_t _outMin = 0.00; //Output saturation limit
			std::double_t _outMax = 0.00; //Output saturation limit
			bool _isUseSaturation = false;

			std::int32_t _saturate = 0U;

			std::double_t _old = 0.00;
			std::double_t _y = 0.00;

			bool _isParamsSet = false;
		};
	}
}

