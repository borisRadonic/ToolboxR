#pragma once
#include <string>

class PIDController
{
public:

	PIDController();

	PIDController(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kaw, std::double_t ts, std::double_t upSaturation);

	~PIDController();
	
	void setParameters(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kaw, std::double_t ts, std::double_t upSaturation);

	double process(std::double_t error);

private:

	std::double_t _Kp = 0.00; //proportional gain coefficient
	std::double_t _Ki = 0.00; //integral gain coefficient
	std::double_t _Kd = 0.00; //differencial gain coefficient
	std::double_t _Kaw = 0.00; //anti-windup gain coefficient.
	std::double_t _Ts = 1.00; //sampling period
	std::double_t _upSat = 0.00; //Output saturation upper limit

	std::double_t du = 0.00;

	std::double_t _invZ = 0.00;
	std::double_t _int = 0.00;

	bool _isParamsSet = false;
};

