#pragma once
#include <cmath>
#include <memory>
#

namespace CntrlLibrary
{
	//The Coulomband Viscous Friction block models Coulomb(static) and viscous(dynamic) friction.
	//The friction torque is a function of relative velocity and is calculated as a sum of Coulomb and viscous components

	namespace Models
	{

		class FrictionModelCSV final
		{
		public:

			FrictionModelCSV();

			~FrictionModelCSV();

			//Ts - Sample time
			//B  - Friction
			//Tc - Coulomb friction torque
			//Tbrk  - Breakaway friction torque
			//Wbrk  - Breakaway friction velocity
			//J			- Moment of inertia
			void setParameters(std::double_t ts, std::double_t b, double_t c0, std::double_t J);

			void reset();

			//w  - velocity
			//ti - input torque
			//a  - acceleration
			void setInputs(std::double_t w, std::double_t ti, std::double_t a);

			void process();

			std::double_t getFrictionTorque() const { return _T; };

		private:

			std::double_t _Ts = 1.00; //sampling period
			std::double_t _B = 0.00;	//Viscous friction coefficient
			std::double_t _C0 = 0.00; //Coulomb friction coefficient
			std::double_t _J = 0.00; //Moment of inertia


			bool _isParamsSet = false;

			std::double_t _w = 0.00; //velocity in rad/sec
			std::double_t _a = 0.00; //acceleration in rad/sec^2
			std::double_t _Ti = 0.00; //Input torque
			std::double_t _T = 0.00; //Friction torque

		};
	}
}

