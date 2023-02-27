#pragma once
#include <string>
#include "Block.h"

namespace DiscreteTime
{
	enum class IntegratorMethod
	{
		ForwardEuler	= 0,
		BackwardEuler	= 1,
		Trapezoidal		= 2
	};

	class Integrator final : public Block
	{
	public:

		Integrator();

		~Integrator();
		
		void setParameters(IntegratorMethod method, std::double_t ts, std::double_t gain, const std::string& name = "");

		void setInitialConditions(std::double_t ic);

		void setSaturation(std::double_t min, std::double_t max);
		
		void reset();

		double process(std::double_t u);

		bool getIsSaturationUsed() const
		{
			return _isUseSaturation;
		}

		std::int32_t getSaturate() const
		{
			return _saturate;
		}
	protected:
		
		void checkSaturation(std::double_t val);

	private:

		IntegratorMethod _method;
		std::double_t _gain = 1.00; //value to multiply with integrator input
		std::double_t _Ts = 1.00; //sampling period
		
		std::double_t _outMin = 0.00; //Output saturation limit
		std::double_t _outMax = 0.00; //Output saturation limit
		bool _isUseSaturation = false;

		std::int32_t _saturate = 0;

		std::double_t _ic = 0.00; //initial conditions

		std::double_t _x = 0.00;
		std::double_t _y = 0.00;

		bool _isParamsSet = false;
	};
}

