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
