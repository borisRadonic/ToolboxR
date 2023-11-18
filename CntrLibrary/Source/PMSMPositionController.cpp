
#include "PMSMPositionController.h"
#include <algorithm>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		// Constructor: Initialize the unique pointers for the filters and controllers.
		PMSMPositionController::PMSMPositionController() :Block()
		{
			_pCurrentController = std::make_unique<PMSMPICurrentController>();
			_pPosController = std::make_unique<PIDController>();
			_pVelController = std::make_unique<PIDController>();
			_pIRFltVel = std::make_unique<Filters::ButterworthLowPassII>();
			_pIRFltNotch = std::make_unique<Filters::IIRSecondOrderFilter>();
		}

		void PMSMPositionController::setPosVelControllerParameters(std::double_t pos_kp, std::double_t vel_kp, std::double_t vel_ki, std::double_t vel_cntrl_pre_filt_frequency, std::double_t Ktq)
		{

			if (_isSamplingPeriodSet && _isLimitParametersSet)
			{
				_pPosController->setParameters(pos_kp, 0.00, 0.00, 1.00, _Ts, _max_vel);
				_pVelController->setParameters(vel_kp, vel_ki, 0.00, 1.00, _Ts, _max_torque);
				_pIRFltVel->setCutoffFrequency(vel_cntrl_pre_filt_frequency, _Ts, "");
				_Ktq = Ktq;
				_isPosVelControllerParametersSet = true;
			}
		}

		void PMSMPositionController::setNotchFilterParameters(const std::double_t a1, const std::double_t a2, const std::double_t b0, const std::double_t b1, const std::double_t b2)
		{
			if (_isSamplingPeriodSet)
			{
				_pIRFltNotch->setParameters(a1, a2, b0, b1, b2, "");
			}
		}

		// Set parameters for the PI current controller, filters, and motor properties.
		void PMSMPositionController::setCurrentControllerParameters(std::double_t iirPreFlt_a1,
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
			std::uint16_t motor_pole_pairs)
		{
			_pCurrentController->setParameters(	iirPreFlt_a1,
												iirPreFlt_b0,
												iirPreFlt_b1,
												iirVelFlt_a1,
												iirVelFlt_b0,
												iirVelFlt_b1,
												kp_q,
												ki_q,
												kd_q,
												kb_q,
												upSat_q,
												kp_d,
												ki_d,
												kd_d,
												kb_d,
												upSat_d,
												l_d,
												l_q,
												kemf,
												motor_pole_pairs,
												_Ts);
			is_CurrentControllerParameters = true;
		}

		// Reset: Reset all the parameters, filters, and controllers to default or zero values.
		void PMSMPositionController::reset()
		{
			_u_q = 0.00;
			_u_d = 0.00;
			_pCurrentController->reset();
			_pPosController->reset();
			_pVelController->reset();
			_pIRFltVel->reset();
			_pIRFltNotch->reset();
		}

		void PMSMPositionController::process(std::double_t ref_position,
											 std::double_t iq_s,
											 std::double_t id_s,
											 std::double_t s_position,
											 std::double_t s_velocity,
											 std::double_t ff_acceleration,
											 std::double_t ff_velocity,
											 std::double_t ff_torque_offset,
											 std::double_t ff_torque_compensations)
		{

			if( _isSamplingPeriodSet &&
				_isLimitParametersSet &&
				_isFFParametersSet &&
				_isPosVelControllerParametersSet &&
				is_CurrentControllerParameters )
			{
				std::double_t tq_ref = 0.00;
				std::double_t iq_ref = 0.00;
				std::double_t id_ref = 0.00; //0.00 for working in const Torque area
			
				ref_position = std::clamp( ref_position, _pos_limit_neg, _pos_limit_pos ); //limit position
				std::double_t errorPos = ref_position - s_position;
				std::double_t posCntrlOut = _pPosController->process(errorPos); // process position P controller
				std::double_t velRef = posCntrlOut + ff_velocity * _ff_vel_gain;				
				velRef = std::clamp(velRef, -_max_vel, _max_vel); //limit velocity				
				velRef = _pIRFltVel->process(velRef); //velocity PI controller pre-filter
				std::double_t errorVel = velRef - s_velocity;
				std::double_t veCntrllOut = _pVelController->process(errorVel); //process PI velocity controller
				std::double_t refTq = veCntrllOut + ff_acceleration * _ff_accel_gain + ff_torque_offset + ff_torque_compensations;
				refTq = std::clamp(refTq, -_max_torque, _max_torque); //limit torque 
				iq_ref = refTq / _Ktq;				
				_pCurrentController->process(iq_ref, id_ref, iq_s, id_s, s_position); //process current controller
				_u_q = _pCurrentController->getUq();
				_u_d = _pCurrentController->getUd();
				
				_v_alpha = _pCurrentController->get_v_alpha();
				_v_beta = _pCurrentController->get_v_beta();
			}
		}
	}
}