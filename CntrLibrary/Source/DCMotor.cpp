#include "DCMotor.h"
#include <memory>

using namespace DiscreteTime;

namespace Models
{
	DCMotor::DCMotor()
	{
		_pIntegratorI = std::make_unique<Integrator>();
		_pIntegratorW = std::make_unique<Integrator>();
	}

	DCMotor::~DCMotor()
	{
	}

	void DCMotor::setParameters(std::double_t ts, std::double_t b, std::double_t Kb, std::double_t J, std::double_t R, std::double_t L, std::double_t Kt)
	{
		_Ts = ts;
		_B = b;
		_Kb = Kb;
		_J = J;
		_R = R;
		_L = L;
		_Kt = Kt;
		_pIntegratorI->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
		_pIntegratorW->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
		_isParamsSet = true;
	}

	void DCMotor::reset()
	{
		_i1 = 0.00;
		_w1 = 0.00;
		_i = 0.00;
		_w = 0.00;
	}

	void DCMotor::process(std::double_t u)
	{
		if (_isParamsSet)
		{
			std::double_t i_d = (1.00 / _L) * (u - (_Kb * _w1) - (_R * _i1));
			_i = _pIntegratorI->process(i_d);
			_i1 = _i;
			std::double_t w_d = (1.00 / _J) * (_Kt * _i - _B * _w1);
			_w = _pIntegratorW->process(w_d);
			_w1 = _w;
		}
	}
}