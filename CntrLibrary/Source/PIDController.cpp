#include "PIDController.h"

namespace DiscreteTime
{

	PIDController::PIDController()
	{
		_pIntegrator = std::make_unique<Integrator>();
		_pDerivative = std::make_unique<Derivative>();
	}

	PIDController::PIDController(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kaw, std::double_t ts, std::double_t upSaturation)
		: _Kp(kp), _Ki(ki), _Kd(kd), _Kaw(kaw), _Ts(ts), _upSat(upSaturation), _isParamsSet(true)
	{
	}

	PIDController::~PIDController()
	{
	}

	void PIDController::setParameters(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kaw, std::double_t ts, std::double_t upSaturation)
	{
		_Kp = kp;
		_Ki = ki;
		_Kd = kd;
		_Kaw = kaw;
		_Ts = ts;
		_upSat = upSaturation;
		_pIntegrator->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
		_pDerivative->setParameters(_Ts, 1.00);
		_isParamsSet = true;
	}

	void PIDController::reset()
	{
		_du1 = 0.00;
		_pIntegrator->reset();
		_pDerivative->reset();

	}

	double PIDController::process(std::double_t error)
	{
		if (false == _isParamsSet)
		{
			return 0.0;
		}


		std::double_t P = _Kp * error;

		std::double_t I = _Ki * _pIntegrator->process(error + (_Kaw * _du1));


		std::double_t D = 0.00;
		if (std::abs(_Kd) >= std::numeric_limits<std::double_t>::min())
		{
			D = _Kd * _pDerivative->process( error);
		}

		double u = P + I + D;
		double uSat = u;
		
		//if (std::abs(u) >= std::numeric_limits<std::double_t>::min())
		if( u >= 0.00)
		{
			uSat = std::fmin(u, _upSat);
		}
		else
		{
			uSat = std::fmax(u, _upSat);
		}
		_du1 = uSat - u  ;

		return uSat;
	}
}