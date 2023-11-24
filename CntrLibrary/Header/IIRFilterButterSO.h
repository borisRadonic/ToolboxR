
#pragma once

#include <string>
#include <vector>
#include "Block.h"
#include "IIRFilterFO.h"
#include "IIRFilterSO.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{
			//Discrete time second order Low-Pass Butterworth filter 
			//
			class ButterworthLowPassII : public IIRSecondOrderFilter
			{
			public:

				ButterworthLowPassII();

				virtual ~ButterworthLowPassII();

				// omega_c - cutoff frequency in rad/s (radians per second)
				//
				void setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name);

				const std::double_t getCutoffFrequency() const
				{
					return _omega_c;
				}

			protected:

				std::double_t  _omega_c = 0.00;
				std::double_t _ts = 1.00;
			};


			//Discrete time second order High-Pass Butterworth filter 
			//
			class ButterworthHighPassII : public IIRSecondOrderFilter
			{
			public:

				ButterworthHighPassII();

				virtual ~ButterworthHighPassII();

				// omega_c - cutoff frequency in rad/s (radians per second)
				//
				void setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name);

				const std::double_t getCutoffFrequency() const
				{
					return _omega_c;
				}

			protected:

				std::double_t  _omega_c = 0.00;
				std::double_t _ts = 1.00;
			};
		}
	}
}