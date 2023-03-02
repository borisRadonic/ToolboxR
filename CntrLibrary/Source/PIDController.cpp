#include "PIDController.h"

namespace DiscreteTime
{

	PIDController::PIDController()
	{
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
		_isParamsSet = true;
	}

	double PIDController::process(std::double_t error)
	{
		if (false == _isParamsSet)
		{
			return 0.0;
		}

		std::double_t prop = _Kp * error;

		//Backward Euler
		_int = _Ki * ((error * _Ts) / (1.00 - _invZ)) + du * _Kaw * ((error * _Ts) / (1.00 - _invZ));

		double diff = ((error * (1.00 - _invZ)) / (_Ts)) * _Kd;

		double u = prop + _int + diff;
		double uSat = u;
		if (u >= 0.00)
		{
			uSat = std::fmin(u, _upSat);
		}
		else
		{
			uSat = std::fmax(u, _upSat);
		}
		du = uSat - u;

		_invZ = error;

		return u;
	}
}