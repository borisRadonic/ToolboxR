
#pragma once
#include <string>
#include <vector>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		class IIRFilterFO final : public Block
		{
		public:

			IIRFilterFO();

			~IIRFilterFO();

			void setParameters(const std::double_t a1, const std::double_t b0, const std::double_t b1, const std::string& name = "");

			double process(std::double_t u);

			const std::double_t getA1() const
			{
				return _a1;
			}

			const std::double_t getB0() const
			{
				return _b0;
			}

			const std::double_t getB1() const
			{
				return _b1;
			}

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::double_t  _a1 = 1.00;       //denominator for z^(-1)
			std::double_t  _b0 = 0.00;       //numerator for z^0
			std::double_t  _b1 = 0.00;       //numerator for z^(-1)

			std::double_t  _x1 = 0.00;       //input at n=-1
			std::double_t  _y1 = 0.00;       //output at n=-1

			std::double_t  _x0 = 0.00;       //input at n=1
			std::double_t  _y0 = 0.00;       //output at n=1

			bool _isParamsSet = false;
		};
	}
}

