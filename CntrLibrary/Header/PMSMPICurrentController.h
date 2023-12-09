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

#pragma once
#include <string>
#include <cmath>
#include <memory>
#include "PIDController.h"
#include "IIRFilterFO.h"
#include "Derivative.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		/*
		*@class PMSMPICurrentController
		* @brief Proportional - Integral(PI) Current Controller for Permanent Magnet Synchronous Motors(PMSM).
		*
		* This class handles the current control of PMSMs.It integrates multiple functionalities
		* such as setting controller parameters, processing current referencesand measurements,
		* resetting internal states, and obtaining the output voltages.
		* @extends Block
		*/		
		class PMSMPICurrentController : public Block
		{
		public:

			/**
			* @brief Constructs a new PMSM PI Current Controller with default initializations.
			*/
			PMSMPICurrentController();

			/**
			* @brief Default destructor.
			*/
			~PMSMPICurrentController()
			{
			}

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
			* @param ts Sampling period.
			*/
			void setParameters( std::double_t iirPreFlt_a1,
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
								std::double_t ts );
			void reset();

			/**
			* @brief Processes the provided current references and measurements.
			*
			* Takes in current references and actual measurements, and computes the necessary
			* control outputs based on set parameters and the PI control strategy.
			*
			* @param iq_ref q-axis current reference.
			* @param id_ref d-axis current reference.
			* @param iq_s q-axis sensed current.
			* @param id_s d-axis sensed current.
			* @param rotor_angle Current rotor angle measurement.
			*/
			void process( std::double_t iq_ref, std::double_t id_ref, std::double_t iq_s, std::double_t id_s, std::double_t rotor_angle );

			/**
			* @brief Retrieves the computed q-axis voltage.
			*
			* @return Computed q-axis voltage in the d-q space.
			*/
			std::double_t getUq() const
			{
 				return _u_q;
			}

			/**
			* @brief Retrieves the computed d-axis voltage.
			*
			* @return Computed d-axis voltage in the d-q space.
			*/
			std::double_t getUd() const
			{
				return _u_d;
			}

			/**
			* @brief Retrieves the computed alpha-axis voltage.
			*
			* @return Computed alpha-axis voltage in the alpha-beta space.
			*/
			std::double_t get_v_alpha() const
			{
				return _v_alpha;
			}

			/**
			* @brief Retrieves the computed beta-axis voltage.
			*
			* @return Computed beta-axis voltage in the alpha-beta space.
			*/
			std::double_t get_v_beta() const
			{
				return _v_beta;
			}

			/**
			* @brief Retrieves the calculated and filtered mechanical velocity of the motor.
			*
			* @return Filtered mechanical velocity.
			*/
			std::double_t get_velocity_flt() const
			{
				return _vel_mech_flt;
			}

		private:
					
			std::uint16_t _polePairs = 1U; /**< Number of motor pole pairs. */
			std::double_t _Kemf = 0.00; /**< Back EMF constant. */
			std::double_t _Lq = 1.00; /**< Inductance of stator in dq frame (q part) in Henry. */
			std::double_t _Ld = 1.00; /**< Inductance of stator in dq frame (d part) in Henry. */
			std::double_t _Ts = 1.00; /**< Sampling period. */

			bool _isParamsSet = false; /**< Flag to indicate if the parameters are set. */

			std::double_t _u_q = 0.00; /**< Output voltage on the q-axis in dq space. */
			std::double_t _u_d = 0.00; /**< Output voltage on the d-axis in dq space. */

			std::double_t _v_alpha;  /**< Output voltage on the alpha in alpha/beta space. */
			std::double_t _v_beta; /**< Output voltage on the beta in alpha/beta space. */

			std::double_t _vel_mech_flt = 0.00; /**< Calculated and filtered mechanical velocity of the motor. */

			// Pointers to various filters and controllers.

			std::unique_ptr<Filters::IIRFirstOrderFilter> _pIRFltPreQ; /**< Pointer to the IIR pre-filter for q-axis. */
			std::unique_ptr<PIDController> _pPIq; /**< Pointer to the PID controller for q-axis. */
			std::unique_ptr<PIDController> _pPId; /**< Pointer to the PID controller for d-axis. */
			std::unique_ptr<Derivative> _pDerAngle; /**< Pointer to calculate the derivative of the angle. */
			std::unique_ptr<Filters::IIRFirstOrderFilter> _pIRFltVel; /**< Pointer to the IIR filter for velocity. */

		};
	}
}
