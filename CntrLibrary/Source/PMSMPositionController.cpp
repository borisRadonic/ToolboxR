/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

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
											 std::double_t ff_torque_compensations,
											 std::double_t ff_jerk)
		{

			if( _isSamplingPeriodSet &&
				_isLimitParametersSet &&
				_isFFParametersSet &&
				_isPosVelControllerParametersSet &&
				is_CurrentControllerParameters )
			{
				ref_position = std::clamp( ref_position, _pos_limit_neg, _pos_limit_pos ); //limit position
				std::double_t errorPos = ref_position - s_position;
				std::double_t posCntrlOut = _pPosController->process(errorPos); // process position P controller
				std::double_t velRef = posCntrlOut + ff_velocity * _ff_vel_gain;
				velRef = std::clamp(velRef, -_max_vel, _max_vel); //limit velocity				
				velRef = _pIRFltVel->process(velRef); //velocity PI controller pre-filter
				std::double_t errorVel = velRef - s_velocity;
				std::double_t veCntrllOut = _pVelController->process(errorVel); //process PI velocity controller
				tq_ref = veCntrllOut + ff_acceleration * _ff_accel_gain + ff_torque_offset + ff_torque_compensations;
				tq_ref += _ff_jerk_gain * ff_jerk;
				tq_ref = std::clamp(tq_ref, -_max_torque, _max_torque); //limit torque 
				iq_ref = tq_ref / _Ktq;
				iq_ref = _pIRFltNotch->process(iq_ref);
				_pCurrentController->process(iq_ref, id_ref, iq_s, id_s, s_position); //process current controller
			}
		}
	}
}