#pragma once

#include <cmath>
#include <memory>
#include "Integrator.h"

namespace CntrlLibrary
{
	namespace Models
	{
		using namespace DiscreteTime;

		class PMSMotor final
		{
		public:

			/*Basic Simulation of Permanent-magnet synchronous motor in DQ domain*/
			PMSMotor();

			~PMSMotor();

			//Ts - Sample time
			//p - Number of pole pairs
			//b  - Combined friction of rotor and load
			//Kemf - EMF constant (There is ralation between Kemf and Ktq, but the most of manufactors provided both of them)
			//J  - Combined moment of inertia of rotor and load
			//Rs  - Stator resistance in [Ohm]
			//Lq  - induction of stator in dq frame (q part) in [H]
			//Ld  - induction of stator in dq frame (d part) in [H]
			//Ktq - Torque constant in [A/Nm]
			//Tf  - Static friction torque in [Nm]

			void setParameters(std::double_t ts, std::uint16_t p, std::double_t b, std::double_t Kemf, std::double_t J, std::double_t Rs, std::double_t Lq, std::double_t Ld, std::double_t Ktq, std::double_t Tf);

			void reset();

			//uq  - Inpult voltage (q part)
			//ud  - Inpult voltage (d part)
			//lt - Load torque
			void setInputs(std::double_t uq, std::double_t ud, std::double_t lt);

			void process();


			std::double_t getIq() const { return _iq; };

			std::double_t getId() const { return _id; };

			//returns rotor angle
			std::double_t getPos() const { return _angleM; };

			std::double_t getAccell() const { return _aM; };

			std::double_t getVel() const { return _wM; };

			std::double_t getTorque() const { return _tE; };

		private:

			std::double_t _Ts = 1.00; //sampling period
			std::uint16_t _polePairs = 1U;
			std::double_t _B = 1.00;
			std::double_t _Kemf = 0.00;
			std::double_t _invJ = 1.00; //Inverse inertia
			std::double_t _R = 1.00;
			std::double_t _Lq = 1.00;
			std::double_t _Ld = 1.00;
			std::double_t _invLq = 1.00;
			std::double_t _invLd = 1.00;
			std::double_t _Ktq = 1.00;
			std::double_t _Tf = 1.00; //static friction of shaft in [Nm]

			bool _isParamsSet = false;

			std::double_t _iq1 = 0.00; //q Current at n-1
			std::double_t _id1 = 0.00; //s Current at n-1

			std::double_t _wM1 = 0.00; //velocity at n-1

			std::double_t _iq = 0.00; //q Current
			std::double_t _id = 0.00; //d Current
						
			std::double_t _aM = 0.00; //acceleration in rad/sec^2
			std::double_t _wM = 0.00; //velocity in rad/sec
			std::double_t _angleE = 0.00; //electrical angle
			std::double_t _angleM = 0.00; //mechanical angle
			
						
			std::double_t _uq = 0.00;
			std::double_t _ud = 0.00;
			std::double_t _lt = 0.00; //load torque

			std::double_t _tE = 0.00; //torque

			std::unique_ptr<Integrator> _pIntegratorIq;
			std::unique_ptr<Integrator> _pIntegratorId;			
			std::unique_ptr<Integrator> _pIntegratorW;
			std::unique_ptr<Integrator> _pIntegratorR;

		};
	}
}

