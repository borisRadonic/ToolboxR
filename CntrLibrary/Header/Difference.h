#pragma once

#include <string>
#include "Block.h"

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


		std::double_t _old = 0.00;
		std::double_t _y = 0.00;
	};
}


