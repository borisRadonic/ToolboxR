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

#include "SineWaveGenerator.h"
#define _USE_MATH_DEFINES
#include <math.h>


namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		SineWaveGenerator::SineWaveGenerator() :
			_amplitude(0),
			_frequency(0),
			_phaseOffset(0),
			_isParamsSet(false)
		{
			/*create input and aouput*/
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}

		SineWaveGenerator::~SineWaveGenerator()
		{
		}

		std::double_t SineWaveGenerator::process(std::double_t time, bool& maximum )
		{
			maximum = false;
			double minDifference = 0.5 * _amplitude* (1.00 - std::sin(0.5 * M_PI - 2 * M_PI * _frequency * _samplingPeriod));
			if (_isParamsSet)
			{
				std::double_t out = _amplitude * std::sin(2 * M_PI * _frequency * time + _phaseOffset);
				if (out > 0.00)
				{
					if ( (out > abs(_amplitude*0.7)) && (abs(_amplitude) - out) <= minDifference)
					{
						maximum = true;
					}
				}
				_ptrIn->set(time);
				_ptrOut->set(out);
				return out;
			}
			return 0.00;
		}
	}
}
