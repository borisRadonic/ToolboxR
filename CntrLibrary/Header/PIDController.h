#pragma once
#include <string>
#include <cmath>
#include <memory>
#include "Integrator.h"
#include "Derivative.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{

		class PIDController : public Block
		{
		public:

			PIDController();

			PIDController(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t upSaturation);

			~PIDController();

			void setParameters(std::double_t kp, std::double_t ki, std::double_t kd, std::double_t kb, std::double_t ts, std::double_t upSaturation);

			void reset();

			double process(std::double_t error);

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::double_t _Kp = 0.00; //proportional gain coefficient
			std::double_t _Ki = 0.00; //integral gain coefficient
			std::double_t _Kd = 0.00; //differencial gain coefficient
			std::double_t _Kb = 0.00; //anti-windup gain coefficient.
			std::double_t _Ts = 1.00; //sampling period
			std::double_t _upSat = 0.00; //Output saturation upper limit



			std::double_t _du1 = 0.00;

			bool _isParamsSet = false;

			std::double_t iii = 0.00;

			std::unique_ptr<Integrator> _pIntegrator;
			std::unique_ptr<Derivative> _pDerivative;

		};
	}
}