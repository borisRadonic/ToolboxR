

#pragma once
#include <string>
#include <cmath>
#include <memory>
#include "PIDController.h"
#include "IIRFilterButterSO.h"
#include "IIRFilterSO.h"
#include "Derivative.h"
#include "PMSMPICurrentController.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		/*
		*@class PMSMPositionController
		* @brief Position Controller for Permanent Magnet Synchronous Motors(PMSM).
		*
		* This class handles the position control of PMSMs.It integrates multiple functionalities
		* such as setting controller parameters, processing referencesand measurements and obtaining the state variables.
		* @extends Block
		*/
		class PMSMPositionController final: public Block
		{
		public:

			/**
			* @brief Constructs a new PMSMPositionController with default initializations.
			*/
			PMSMPositionController();

			/**
			* @brief Default destructor.
			*/
			~PMSMPositionController()
			{
			}

			/*this is the first set function which must be called*/
			inline void setSamplinkPeriod(std::double_t ts)
			{
				_Ts = ts;
				_isSamplingPeriodSet = true;
			}

			/*this is the second set function which must be called*/
			inline void setLimitParameters(std::double_t pos_limit_neg, std::double_t pos_limit_pos, std::double_t max_vel, std::double_t max_torque)
			{
				_pos_limit_neg = pos_limit_neg;
				_pos_limit_pos = pos_limit_pos;
				_max_vel = max_vel;
				_max_torque = max_torque;
				_isLimitParametersSet = true;
			}


			void setFeedForwardParameters(std::double_t ff_vel_gain, std::double_t ff_accel_gain)
			{
				_ff_vel_gain = ff_vel_gain;
				_ff_accel_gain = ff_accel_gain;
				_isFFParametersSet = true;
			}


			void setPosVelControllerParameters(	std::double_t pos_kp, std::double_t vel_kp, std::double_t vel_ki, std::double_t vel_cntrl_pre_filt_frequency, std::double_t Ktq);


			void setNotchFilterParameters(const std::double_t omega_c, const std::double_t bw);

			
			/**
			* @brief Sets the parameters for the PI current controller, filters, and motor properties.
			*
			* This function configures the parameters necessary for the operation of the PMSM PI Current Controller.
			* It provides means to set the coefficients for the filters, the PI controllers, and other motor-specific properties.
			*
			* @param iirPreFlt_a1 First coefficient for the IIR pre-filter.
			* @param iirPreFlt_b0 Zeroth coefficient for the IIR pre-filter.
			* @param iirPreFlt_b1 Second coefficient for the IIR pre-filter.
			* @param iirVelFlt_a1 First coefficient for the IIR velocity filter.
			* @param iirVelFlt_b0 Zeroth coefficient for the IIR velocity filter.
			* @param iirVelFlt_b1 Second coefficient for the IIR velocity filter.
			* @param kp_q Proportional gain coefficient of PI q-side controller.
			* @param ki_q Integral gain coefficient of PI q-side controller.
			* @param kd_q Derivative gain coefficient of PI q-side controller (not typically used in a PI controller, but included for completeness).
			* @param kb_q Anti-windup gain coefficient of PI q-side controller.
			* @param upSat_q Output saturation upper limit of PI q-side controller.
			* @param kp_d Proportional gain coefficient of PI d-side controller.
			* @param ki_d Integral gain coefficient of PI d-side controller.
			* @param kd_d Derivative gain coefficient of PI d-side controller (not typically used in a PI controller, but included for completeness).
			* @param kb_d Anti-windup gain coefficient of PI d-side controller.
			* @param upSat_d Output saturation upper limit of PI d-side controller.
			* @param l_d Inductance of stator in dq frame (d part) in Henrys.
			* @param l_q Inductance of stator in dq frame (q part) in Henrys.
			* @param kemf Back EMF constant.
			* @param motor_pole_pairs The number of motor pole pairs.
			*/
			void setCurrentControllerParameters(std::double_t iirPreFlt_a1,
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
				std::uint16_t motor_pole_pairs);

			void process(std::double_t ref_position,
				std::double_t iq_s,
				std::double_t id_s,
				std::double_t s_position,
				std::double_t s_velocity,
				std::double_t ff_acceleration,
				std::double_t ff_velocity,
				std::double_t ff_torque_offset,
				std::double_t ff_torque_compensations);

			void reset();


			/**
			* @brief Retrieves the computed q-axis voltage.
			*
			* @return Computed q-axis voltage in the d-q space.
			*/
			std::double_t getUq() const
			{
				return _pCurrentController->getUq();
			}

			/**
			* @brief Retrieves the computed d-axis voltage.
			*
			* @return Computed d-axis voltage in the d-q space.
			*/
			std::double_t getUd() const
			{
				return _pCurrentController->getUd();
			}

			/**
			* @brief Retrieves the computed alpha-axis voltage.
			*
			* @return Computed alpha-axis voltage in the alpha-beta space.
			*/
			std::double_t get_v_alpha() const
			{
				return _pCurrentController->get_v_alpha();
			}

			/**
			* @brief Retrieves the computed beta-axis voltage.
			*
			* @return Computed beta-axis voltage in the alpha-beta space.
			*/
			std::double_t get_v_beta() const
			{
				return _pCurrentController->get_v_beta();
			}

			std::double_t get_refTq() const
			{
				return tq_ref;
			}

			

		private:

			std::double_t _Ts = 1.00; /**< Sampling period. */

			bool _isSamplingPeriodSet = false;
			bool _isLimitParametersSet = false;
				
			std::double_t _pos_limit_neg = 0.00;
			std::double_t _pos_limit_pos = 0.00;
			std::double_t _max_vel = 0.00;
			std::double_t _max_torque = 0.00;


			std::double_t tq_ref = 0.00;
			std::double_t iq_ref = 0.00;
			std::double_t id_ref = 0.00; //0.00 for working in const Torque area


			std::double_t _Ktq = 1.00; //Motor Torque konstant

			bool _isFFParametersSet = false;
			std::double_t _ff_vel_gain = 1.00;
			std::double_t _ff_accel_gain = 1.00;
			bool _isPosVelControllerParametersSet = false;
			bool is_CurrentControllerParameters = false;

			std::unique_ptr<PMSMPICurrentController> _pCurrentController; /**< Pointer to the PMSMPICurrentController */
			std::unique_ptr<PIDController> _pPosController; /**< Pointer to the PID controller for position. */
			std::unique_ptr<PIDController> _pVelController; /**< Pointer to the PID controller for d-axis. */
			std::unique_ptr<Filters::ButterworthLowPassII> _pIRFltVel; /**< Pointer to the IIR filter for velocity controller. */
			std::unique_ptr<Filters::ButterworthBandStopII> _pIRFltNotch; /**< Torque Notch filter. */
		};
	}
}