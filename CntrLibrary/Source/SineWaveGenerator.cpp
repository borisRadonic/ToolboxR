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

		std::double_t SineWaveGenerator::process(std::double_t time)
		{
			
			if (_isParamsSet)
			{
				std::double_t out = _amplitude * std::sin(2 * M_PI * _frequency * time + _phaseOffset);	
				_ptrIn->set(time);
				_ptrOut->set(out);
				return out;
			}
			return 0.00;
		}
	}
}
