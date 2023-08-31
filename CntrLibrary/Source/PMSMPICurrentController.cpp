#include "PMSMPICurrentController.h"


namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		// Constructor: Initialize the unique pointers for the filters and controllers.
		PMSMPICurrentController::PMSMPICurrentController() :Block()
		{
			_pIRFltPreQ = std::make_unique<IIRFilterFO>();
			_pPIq		= std::make_unique<PIDController>();
			_pPId		= std::make_unique<PIDController>();
			_pDerAngle	= std::make_unique<Derivative>();
			_pIRFltVel	= std::make_unique<IIRFilterFO>();
		}
		// Set parameters for the PI current controller, filters, and motor properties.
		void PMSMPICurrentController::setParameters(std::double_t iirPreFlt_a1,
													std::double_t iirPreFlt_b0,
													std::double_t iirPreFlt_b1,
													std::double_t iirVelFlt_a1,
													std::double_t iirVelFlt_b0,
													std::double_t iirVelFlt_b1,
													std::double_t kp_q,
													std::double_t ki_q,
													std::double_t kd_q,
													std::double_t kb_q,
													std::double_t upSat_q,
													std::double_t kp_d,
													std::double_t ki_d,
													std::double_t kd_d,
													std::double_t kb_d,
													std::double_t upSat_d,
													std::double_t l_d,
													std::double_t l_q,
													std::double_t kemf,
													std::uint16_t motor_pole_pairs,
													std::double_t ts)
		{						
			// Setting the basic parameters for the motor.
			_polePairs = motor_pole_pairs;
			_Kemf = kemf;
			_Lq = l_q;
			_Ld = l_d;

			_Ts = ts;

			// Flag to indicate that the parameters have been set.
			_isParamsSet = true;
			
			// Setting the parameters for the filters and PI controllers.
			_pIRFltPreQ->setParameters(iirPreFlt_a1, iirPreFlt_b0, iirPreFlt_b1);
			_pPIq->setParameters(kp_q, ki_q, 0.00, kb_q, _Ts, upSat_q);
			_pPId->setParameters(kp_d, ki_d, 0.00, kb_d, _Ts, upSat_d);
			_pDerAngle->setParameters(_Ts, 1.00);
			_pIRFltVel->setParameters(iirVelFlt_a1, iirVelFlt_b0, iirVelFlt_b1 );
		}

		// Reset: Reset all the parameters, filters, and controllers to default or zero values.
		void PMSMPICurrentController::reset()
		{
			_u_q = 0.00;
			_u_d = 0.00;
			_vel_mech_flt = 0.00;
			_pIRFltPreQ->reset();
			_pPIq->reset();
			_pPId->reset();
			_pDerAngle->reset();
			_pIRFltVel->reset();
		}

		void PMSMPICurrentController::process(std::double_t iq_ref, std::double_t id_ref, std::double_t iq_s, std::double_t id_s, std::double_t rotor_angle)
		{
			if (_isParamsSet)
			{
				// Filtering the reference value for iq.
				std::double_t iq_ref_flt = _pIRFltPreQ->process(iq_ref);

				// Calculating and filtering the mechanical velocity.
				_vel_mech_flt = _pIRFltVel->process(_pDerAngle->process(rotor_angle));

				// Compute errors for q and d axis currents.
				std::double_t errorQ = iq_ref_flt - iq_s;
				std::double_t errorD = id_ref - id_s;

				// Process the errors through the PI controllers to get outputs
				std::double_t outQ = _pPIq->process(errorQ);
				std::double_t outD = _pPId->process(errorD);

				// Compute the output voltage values for q and d axis based on PI output and motor parameters.
				// Feed-forward back EMF compensation: Using motor speed (vel_mech_flt) and back EMF constant (_Kemf) to estimate back EMF and compensate it.
				// L coupling: Considering the mutual inductance effect between d and q axis in synchronous machines, the change in one axis affects the other.
				_u_q = outQ + _polePairs * _vel_mech_flt * _Kemf + id_s * _vel_mech_flt * _Ld;
				_u_d = outD - iq_s * _polePairs * _vel_mech_flt * _Lq;
			}
		}
	}
}