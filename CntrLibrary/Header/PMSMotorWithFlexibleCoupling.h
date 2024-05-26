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
#include "PMSMotor.h"
#include "SOSystem.h"


namespace CntrlLibrary
{
	namespace Models
	{
		
		class PMSMotorWithFlexibleCoupling : public PMSMotor
		{
		public:

			PMSMotorWithFlexibleCoupling();

			virtual ~PMSMotorWithFlexibleCoupling();

			
			//Jload - Moment of inertia of the load
			//c_damp - Damping coefficient
			//c_tor_stif - Torsional stiffness
			void setLoadAndCouplingParameters(std::double_t c_damp, std::double_t c_tor_stif);
			
			virtual void reset();

			virtual void process();

		private:
			std::unique_ptr<SOSystem> _pSOsys;

			std::double_t _c_damp = 0.00;
			std::double_t _c_tor_stif = 0.00;

			std::double_t _D = 0.00;// Damping  = _c_damp/ ( 2 * (Jmot+ Jload) );
			std::double_t _omega0 = 0.00; //Frequency  = sqrt( _c_tor_stif/(Jmot+ Jload) );

			std::double_t _aL = 0.00; //load acceleration in rad/sec^2
			std::double_t _wL = 0.00; //load velocity in rad/sec
			std::double_t _angleL = 0.00;//load position

			bool _isCouplingParamsSet = false;

			std::unique_ptr<Integrator> _pIntegratorLW;
			std::unique_ptr<Integrator> _pIntegratorLR;

		
		};
	}
}

