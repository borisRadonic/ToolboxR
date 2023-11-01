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