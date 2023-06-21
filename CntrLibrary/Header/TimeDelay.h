#pragma once

#include <string>
#include <queue>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		//Implementation of the delay of input signal by a specified number of samples

		//Time Range						Output
		//start to (start + lenght)			0.00
		//after (start + lenght)			Input signal

		class TimeDelay final : public Block
		{
		public:


			TimeDelay();

			~TimeDelay();

			void setParameters(const std::uint32_t lenght);

			double process(const std::double_t u);

			void reset();

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			bool _isParamsSet = false;

			std::uint32_t _lenght = 0U;

			std::queue<std::double_t> _queue;

		};
	}
}

