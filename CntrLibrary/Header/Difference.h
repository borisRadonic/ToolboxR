#pragma once

#include <string>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		//Discrete-time derivative

		class Difference final : public Block
		{
		public:

			Difference();

			~Difference();

			inline void setInitialConditions(std::double_t old)
			{
				_old = old;
			}

			double process(std::double_t u);

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::double_t _old = 0.00;
			std::double_t _y = 0.00;
		};
	}
}

