#pragma once

#include <string>

#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		//Discrete-time derivative

		class Derivative final : public Block
		{
		public:

			Derivative();

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

			std::int32_t _saturate = 0;

			std::double_t _old = 0.00;
			std::double_t _y = 0.00;

			bool _isParamsSet = false;
		};
	}
}

