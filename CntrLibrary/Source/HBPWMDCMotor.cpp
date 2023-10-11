#include "HBPWMDCMotor.h"

#include <memory>

namespace CntrlLibrary
{
	using namespace DiscreteTime;

	namespace Models
	{

		#define MIN_CURRENT 0.00001f

		HBPWMDCMotor::HBPWMDCMotor()
		{
			_pIntegratorI = std::make_unique<Integrator>();
			_pIntegratorW = std::make_unique<Integrator>();
		}

		HBPWMDCMotor::~HBPWMDCMotor()
		{
		}

		void HBPWMDCMotor::setParameters(	std::double_t ts,
											std::double_t b,
											std::double_t Kb,
											std::double_t J,
											std::double_t R,
											std::double_t L,
											std::double_t Kt,
											std::double_t Vf_di,
											std::double_t Slope_di,
											std::double_t Vf_ce,
											std::double_t Slope_ic)
		{
			_Ts = ts;
			_B = b;
			_Kb = Kb;
			_J = J;
			_R = R;
			_L = L;
			_Kt = Kt;
			_Vf_di = Vf_di;
			_Slope_di = Slope_di;
			_Vf_ce = Vf_ce;
			_Slope_ic = Slope_ic;
			_pIntegratorI->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorW->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_isParamsSet = true;
		}

		void HBPWMDCMotor::reset()
		{
			_i1 = 0.00;
			_w1 = 0.00;
			_i = 0.00;
			_w = 0.00;
			_a = 0.00;
			_sw14ON = false;
			_sw23ON = false;
			_outTorque = 0.00;
			_Udiode = 0.70;
			_Uce = 0.70;
		}

		void HBPWMDCMotor::setInputs(std::double_t u, std::double_t lt, bool sw14ON, bool sw23ON)
		{
			_u = u;
			_lt = lt;
			_sw14ON = sw14ON;
			_sw23ON = sw23ON;
		}

		void HBPWMDCMotor::process()
		{
			if (_isParamsSet)
			{
				std::double_t i_d = 0.00;
				std::double_t u_r = 0.00;
				if (!(_sw23ON == true && _sw14ON == true))
				{
					if (_sw14ON == true )
					{
						_Uce = _Vf_ce +  abs(_i1)/_Slope_ic;
						_Udiode = _Vf_di + abs(_i1) / _Slope_di;
						u_r = _u - (_Kb * _w1) - (_R * _i1) - 2 * _Uce;
						if (_i1 < 0.00)
						{
							/*Current flow through diode D4 and SW1*/
							u_r = -_u + (_Kb * _w1) + (_R * _i1) + _Uce + _Udiode;
							i_d = -(1.00 / _L) * u_r;
						}
						else
						{
							/*Current flow through SW1 and SW4*/
							u_r = _u - (_Kb * _w1) - (_R * _i1) - 2.0 * _Uce;
							i_d = (1.00 / _L) * u_r;
						}
					}
					else if (_sw23ON == true)
					{
						_Uce = _Vf_ce + abs(_i1)/ _Slope_ic;
						_Udiode = _Vf_di + abs(_i1) / _Slope_di;

						if (_i1 > 0.00)
						{
							/*Current flow through diode D2 and SW2*/
							u_r = -_u - (_Kb * _w1) + (_R * _i1) + _Uce + _Udiode;
							i_d = (1.00 / _L) * u_r;
						}
						else
						{
							/*Current flow through SW3 and SW2*/
							u_r = _u + (_Kb * _w1) - (_R * _i1) - 2.0 *_Uce;
							i_d = -(1.00 / _L) * u_r;
						}						
					}
					else
					{
						//all OFF
						if (abs(_i1) > MIN_CURRENT)
						{
							_Udiode = _Vf_di + abs(_i1) / _Slope_di;
							i_d = (1.00 / _L) * (_u - (_Kb * _w1) - (_R * _i1) - 2.0 * _Udiode);
						}
					}
					_i = _pIntegratorI->process(i_d);
					_i1 = _i;
					_a = (1.00 / _J) * (_Kt * _i - _B * _w1 - _lt);
					_w = _pIntegratorW->process(_a);
					_w1 = _w;
					_outTorque = _Kt * _i;
				}
			}
		}
	}
}