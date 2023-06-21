#pragma once

#include <string>
#include <vector>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		class IIRFilterSO final : public Block
		{
		public:

			IIRFilterSO();

			~IIRFilterSO();

			void setParameters(const std::double_t a1, const std::double_t a2, const std::double_t b0, const std::double_t b1, const std::double_t b2, const std::string& name = "");

			double process(std::double_t u);

			const std::double_t getA1() const
			{
				return _a1;
			}

			const std::double_t getA2() const
			{
				return _a2;
			}

			const std::double_t getB0() const
			{
				return _b0;
			}

			const std::double_t getB1() const
			{
				return _b1;
			}

			const std::double_t getB2() const
			{
				return _b2;
			}

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::double_t  _a1 = 1.00;       //denominator for z^(-1)
			std::double_t  _a2 = 1.00;       //denominator for z^(-2)

			std::double_t  _b0 = 0.00;       //numerator for z^0
			std::double_t  _b1 = 0.00;       //numerator for z^(-1)
			std::double_t  _b2 = 0.00;       //numerator for z^(-2)

			std::double_t  _x2 = 0.00;       //input at n=-2
			std::double_t  _y2 = 0.00;       //output at n=-2

			std::double_t  _x1 = 0.00;       //input at n=-1
			std::double_t  _y1 = 0.00;       //output at n=-1

			std::double_t  _x0 = 0.00;       //input at n=1
			std::double_t  _y0 = 0.00;       //output at n=1

			bool _isParamsSet = false;
		};
	}
}



