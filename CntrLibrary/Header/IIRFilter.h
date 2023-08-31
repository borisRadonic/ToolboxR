
#pragma once
#include <string>
#include <vector>
#include "Block.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		class IIRFilter final : public Block
		{
		public:

			IIRFilter();

			~IIRFilter();

			//Filter coefficients are numerator coefficients and denominator coefficients (vectors must have the same dimensions)
			//
			void setParameters(const std::vector<std::double_t>& numerator, const std::vector<std::double_t>& denominator, const std::string& name = "");

			double process(std::double_t u);

			void reset();

		private:

			std::shared_ptr<Signal<std::double_t>> _ptrIn;
			std::shared_ptr<Signal<std::double_t>> _ptrOut;

			std::vector <std::double_t> _x;
			std::vector <std::double_t> _y;

			bool _isParamsSet = false;

			std::vector<std::double_t> _numerator;
			std::vector<std::double_t> _denominator;
		};
	}
}
