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
#include <random>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		
		class DiscreteRandomGenerator final : public Block
		{
		public:

			DiscreteRandomGenerator() = delete;

			// Constructor with default values
			explicit DiscreteRandomGenerator(std::double_t mean = 0.0, std::double_t variance = 1.0, unsigned int seed = 0, std::double_t sampleTime = 1.0)
				: Block(),
				_mean(mean),			
				_seed(seed),
				_Ts(sampleTime)				
			{
				if (variance < 0.00)
				{
					_variance = 1.00;
				}
				_distribution = std::normal_distribution<std::double_t>(mean, sqrt(_variance));

				if (seed == 0U)
				{
					std::random_device rd;
					seed = rd();
				}
				_generator.seed(seed);
			}

			~DiscreteRandomGenerator()
			{
			}
			
			double process()
			{
				return _distribution(_generator);
			}

			double getSampleTime() const
			{
				return _Ts;
			}

		private:
			std::double_t _mean;

			std::double_t _variance;

			unsigned int _seed;

			std::default_random_engine _generator;

			std::normal_distribution<std::double_t> _distribution;

			std::double_t _Ts = 1.00; //sampling period

		};
	}
}