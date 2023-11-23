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
			/// Butterworth HP Filter
			/// </summary>

			ButterworthHighPassII::ButterworthHighPassII()
			{
			}

			ButterworthHighPassII::~ButterworthHighPassII()
			{
			}

			//TODO: High-pass filter verification!!!!!!
			//
			void ButterworthHighPassII::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{

					std::double_t f_c = omega_c / (2 * M_PI);
					std::double_t f_s = 1.00 / ts;

					std::double_t omega_norm = f_c / (f_s / 2.00);

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

			/// <summary>
			/// Butterworth Band-pass (BP) Filter
			/// </summary>

			ButterworthBandPassII::ButterworthBandPassII()
			{
			}

			ButterworthBandPassII::~ButterworthBandPassII()
			{
			}

			//TODO: Band-pass filter verification!!!!!!
			//
			void ButterworthBandPassII::setBandPassParameters(const std::double_t omega0, const std::double_t bw, const std::double_t ts, const std::string& name)
			{
				_omega0 = omega0;
				_bw = bw;
				_ts = ts;
				if ((ts > 0.00) && (_omega0 > 0.00))
				{
					std::double_t omega_p0 = (2.0 / ts) * tan(omega0 * ts / 2.0);
					std::double_t omega_pbw = (2.0 / ts) * tan(bw * ts / 2.0);

					std::double_t oott = omega_p0 * omega_p0 * ts * ts;
					std::double_t den = (1 + omega_pbw * ts + oott);


					_b0 = (omega_p0 * ts) / den;
					_b1 = 0.00;
					_b2 = -_b0;

					_a1 = 2.00 * (1.00 - oott) / den;
					_a2 = (1.00 - omega_pbw * ts + oott) / den;

					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}


			/// <summary>
			/// Butterworth Band-pstop (Notch) Filter
			/// </summary>

			ButterworthBandStopII::ButterworthBandStopII()
			{
			}

			ButterworthBandStopII::~ButterworthBandStopII()
			{
			}

			//TODO: Band-stop filter verification!!!!!!
			//
			void ButterworthBandStopII::setBandStopParameters(const std::double_t omega0, const std::double_t bw, const std::double_t ts, const std::string& name)
			{
				_omega0 = omega0;
				_bw = bw;
				_ts = ts;
				if ((ts > 0.00) && (_omega0 > 0.00))
				{
					std::double_t omega_p0 = (2.0 / ts) * tan(omega0 * ts / 2.0);
					std::double_t omega_pbw = (2.0 / ts) * tan(bw * ts / 2.0);

					std::double_t oott = omega_p0 * omega_p0 * ts * ts;
					std::double_t den = (1 + omega_pbw * ts + oott);


					_b0 = (1.00 + oott) / den;
					_b1 = 2.00 * (1.00 - oott) / den;
					_b2 = -_b0;

					//This symmetry arises from the fact that the filter has a notch centered at the specified frequency
					//and rejects that frequency component both in phase and magnitude.
					_a1 = _b1; 

					_a2 = (1.00 - omega_pbw * ts + oott) / den;

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