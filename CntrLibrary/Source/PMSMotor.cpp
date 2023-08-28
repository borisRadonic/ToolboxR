#include "PMSMotor.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
	using namespace DiscreteTime;

	namespace Models
	{
		PMSMotor::PMSMotor()
		{
			_pIntegratorIq = std::make_unique<Integrator>();
			_pIntegratorId = std::make_unique<Integrator>();
			_pIntegratorW = std::make_unique<Integrator>();
			_pIntegratorR = std::make_unique<Integrator>();
		}

		PMSMotor::~PMSMotor()
		{
		}

		void PMSMotor::setParameters(std::double_t ts, std::uint16_t p, std::double_t b, std::double_t Kemf, std::double_t J, std::double_t Rs, std::double_t Lq, std::double_t Ld, std::double_t Ktq, std::double_t Tf)
		{
			_Ts = ts;					
			_polePairs = p;
			_B = b;
			_Kemf = Kemf;
			_invJ = 1.00 / J; //Inverse inertia
			_R = Rs;
			_Lq = Lq;
			_Ld = Ld;
			_invLq = 1.00/Lq;
			_invLd = 1.00/Ld;
			_Ktq = Ktq;
			_Tf = Tf;
			_pIntegratorIq->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorIq->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorW->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_pIntegratorR->setParameters(IntegratorMethod::ForwardEuler, _Ts, 1.00);
			_isParamsSet = true;
		}

		void PMSMotor::reset()
		{
			_iq1 = 0.00;
			_id1 = 0.00;
			_wM1 = 0.00;
			_iq = 0.00;
			_id = 0.00;
			_angleE = 0.00;
			_angleM = 0.00;
			_wM = 0.00;
			_aM = 0.00;
			_tE = 0.00;
			_uq = 0.00;
			_ud = 0.00;
		}

		void PMSMotor::setInputs(std::double_t Uq, std::double_t Ud, std::double_t lt)
		{
			_uq = 0.00;
			_ud = 0.00;
			_lt = lt;
		}

		void PMSMotor::process()
		{
			if (_isParamsSet)
			{
				std::double_t id_der = _invLd * (_ud - (_R * _id1) + _polePairs * _wM * _Lq * _iq1);
				std::double_t iq_der = _invLq * ( _uq - (_R * _iq1) - _wM * (_polePairs * _Ld * _id1 - _Kemf) );
				_id = _pIntegratorId->process(id_der);
				_iq = _pIntegratorIq->process(iq_der);
				_aM = _invJ * (_Ktq * _iq - _B * _wM1 - _Tf - _lt);
				_wM = _pIntegratorW->process(_aM);
				_angleM = _pIntegratorR->process(_wM);
				double angleE = _angleM * _polePairs;
				_angleE = angleE - M_PI * floor(angleE * M_2_PI);
				_iq1 = _iq;
				_id1 = _id;
				_wM1 = _wM;
				_tE = _Ktq * _iq;
			}
		}
	}
}
