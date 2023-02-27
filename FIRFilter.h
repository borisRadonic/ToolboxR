#pragma once

#include <string>
#include <vector>
#include "Block.h"

namespace DiscreteTime
{
	//Implementation of finite impulse response (FIR) filter
	class FIRFilter final : public Block
	{
	public:

		FIRFilter();

		~FIRFilter();

		void setParameters(const std::vector<std::double_t>& coefficients, const std::string& name = "");

		double process(std::double_t u);

	private:

		bool _isParamsSet = false;

		std::vector<std::double_t> _b;

		double _y = 0.00;

		std::vector<std::double_t> _x;

	};
}