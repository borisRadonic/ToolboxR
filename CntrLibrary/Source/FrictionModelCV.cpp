#include "FrictionModelCV.h"
#include <math.h>

namespace CntrlLibrary
{
	namespace Models
	{
		FrictionModelCSV::FrictionModelCSV()
		{
		}
		FrictionModelCSV::~FrictionModelCSV()
		{
		}

		void FrictionModelCSV::setParameters(std::double_t ts, std::double_t b, double_t c0, std::double_t J)
		{
			_Ts = ts;
			_B = b;
			_C0 = c0;
			_J = J;
			_isParamsSet = true;
		}

		void FrictionModelCSV::reset()
		{
			_w = 0.00;
			_a = 0.00;
			_T = 0.00;
		}


		void FrictionModelCSV::setInputs(std::double_t w, std::double_t ti, std::double_t a)
		{
			_w = w;
			_Ti = ti;
			_a = a;
		}

		void FrictionModelCSV::process()
		{
			if (_isParamsSet)
			{
				//Tf = Ts +Tc + Tv
				//Ts - static friction
				//Tc - Coulomb or kinetic friction
				//Tv - viscous friction
				std::double_t  tc = 0 - 00;
				std::double_t  tv = _B * _w;
				std::double_t  ts = 0.00;
				std::double_t  signW = 1.00;
				std::double_t  signT = 1.00;

				if (std::signbit(_w))
				{
					//negative velocity
					signW = -1.00;
				}

				if (std::signbit(_Ti))
				{
					//negative input torque
					signT = -1.00;
				}

				if (std::abs(_a) < std::numeric_limits<std::double_t>::min())
				{
					//acceleration is != 0				

					if (abs(_Ti - (_J * _a)) < _C0)
					{
						ts = _Ti;
					}
					else
					{
						ts = _C0 * signW;
					}
				}
				else
				{
					if (abs(_Ti) < _C0)
					{
						ts = _Ti;
					}
					else
					{
						ts = _C0 * signT;
					}
				}

				if (std::abs(_w) >= std::numeric_limits<std::double_t>::min())
				{
					{
						ts = 0.00; //velocity > 0.00
						tc = _C0 * signW;
					}
				}
				_T = ts + tc + tv;
			}
		}
	}
}