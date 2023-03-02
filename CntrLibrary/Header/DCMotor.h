#pragma once
#include <cmath>
#include <memory>
#include "Integrator.h"

namespace Models
{

	using namespace DiscreteTime;

	class DCMotor final
	{
	public:

		DCMotor();

		~DCMotor();
		
		//Ts - Sample time
		//B  - Friction
		//Kb - EMF constant
		//J  - Moment of inertia
		//R  - resistance in Ohm
		//L  - induction in H
		//Kt - Torque constant in A/Nm

		void setParameters(std::double_t ts, std::double_t b, std::double_t Kb, std::double_t J, std::double_t R, std::double_t L, std::double_t Kt);

		void reset();

		void process(std::double_t u);

		

		std::double_t getCurrent() const { return _i; };

		std::double_t getVelocity() const {	return _w; };

	private:

		std::double_t _Ts = 1.00; //sampling period
		std::double_t _B = 1.00;
		std::double_t _Kb = 1.00;
		std::double_t _J = 1.00;
		std::double_t _R = 1.00;
		std::double_t _L = 1.00;
		std::double_t _Kt = 1.00;
	
		bool _isParamsSet = false;

		std::double_t _i1 = 0.00; //Current at n-1
		std::double_t _w1 = 0.00; //velocity at n-1

		std::double_t _i = 0.00; //Current

		std::double_t _w = 0.00; //velocity in rad

		std::unique_ptr<Integrator> _pIntegratorI;
		std::unique_ptr<Integrator> _pIntegratorW;
	};
}

