#include "IIRFilterButterSO.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{

			/// <summary>
			/// Butterworth LP Filter
			/// </summary>
			/// 
			ButterworthLowPassII::ButterworthLowPassII()
			{
			}

			ButterworthLowPassII::~ButterworthLowPassII()
			{
			}

			void ButterworthLowPassII::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{
					std::double_t f_c = omega_c / (2 * M_PI);
					std::double_t f_s = 1.00 / ts;

					std::double_t omega_norm =  f_c / (f_s/2.00);

					std::double_t omega_p = 1.00/  tan( M_PI * f_c/ f_s );
					
					std::double_t  omega_p_squ = omega_p * omega_p;
					_b0 = 1.00 / (1.00 + M_SQRT2 * omega_p + omega_p_squ);
					_b1 = 2.00 * _b0;
					_b2 = _b0;

					
					_a1 = -(2.00 * (omega_p_squ - 1.00)) * _b0;
					_a2 =  (1.00 - M_SQRT2 * omega_p + omega_p_squ) * _b0;

					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}

			/// <summary>
			/// Butterworth High-pass Filter
			/// </summary>
			ButterworthHighPassII::ButterworthHighPassII()
			{
			}

			ButterworthHighPassII::~ButterworthHighPassII()
			{
			}

			void ButterworthHighPassII::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{

					std::double_t f_c = omega_c / (2 * M_PI);
					std::double_t f_s = 1.00 / ts;
					std::double_t omega_p = 1.00 / tan(M_PI * f_c / f_s);

					std::double_t  omega_p_squ = omega_p * omega_p;
					_b0 = 1.00 / (1.00 + M_SQRT2 * omega_p + omega_p_squ);
					_b1 = 2.00 * _b0;
					_b2 = _b0;

					_a1 = -(2.00 * (omega_p_squ - 1.00)) * _b0;
					_a2 = (1.00 - M_SQRT2 * omega_p + omega_p_squ) * _b0;

					_b0 = _b0 * omega_p_squ;
					_b1 = -_b1 * omega_p_squ;
					_b2 = _b2 * omega_p_squ;

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