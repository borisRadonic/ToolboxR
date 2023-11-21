#pragma once
#pragma once

#include <string>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{

        class SineWaveGenerator final : public Block
        {
        public:

            SineWaveGenerator();

            ~SineWaveGenerator();

            inline void setParameters(std::double_t amplitude, std::double_t frequency, std::double_t phaseOffset, const std::string& name)
            {
                _amplitude = amplitude;
                _frequency = frequency;
                _phaseOffset = phaseOffset;
                _isParamsSet = true;
                setName(name);
            }

            inline bool isParametersSet() const
            {

            }
            std::double_t process(std::double_t time);

        private:

            std::shared_ptr<Signal<std::double_t>> _ptrIn;
            std::shared_ptr<Signal<std::double_t>> _ptrOut;

            std::double_t _amplitude = 1.00;
            std::double_t _frequency = 1.00;
            std::double_t _phaseOffset = 0.00;
            bool _isParamsSet = false;
		};
	}
}

