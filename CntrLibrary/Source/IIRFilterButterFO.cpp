#include "IIRFilterButterFO.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{

			/// <summary>
			/// Butterworth LP Filter 1st order
			/// </summary>
			/// 
			ButterworthLowPassI::ButterworthLowPassI()
			{
			}

			ButterworthLowPassI::~ButterworthLowPassI()
			{
			}

			void ButterworthLowPassI::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{
					std::double_t alpha_vel = exp(-omega_c * ts);
					_a1 = -alpha_vel;
					_b0 = 1.00 - alpha_vel;
					_b1 = 0.00;
					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}

			/// <summary>
			/// Butterworth HP Filter 1st order
			/// </summary>
			/// 
			ButterworthHighPassI::ButterworthHighPassI()
			{
			}

			ButterworthHighPassI::~ButterworthHighPassI()
			{
			}

			void ButterworthHighPassI::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{
					std::double_t alpha_vel = exp(-omega_c * ts);
					_a1 = -alpha_vel;
					_a1 = -alpha_vel;
					_b0 = (1.00 + alpha_vel) / 2.0;
					_b1 = -(1.00 + alpha_vel) / 2.0;
					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}
		}
	}
}
