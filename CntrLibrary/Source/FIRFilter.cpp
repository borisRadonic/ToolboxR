#include "FIRFilter.h"

namespace DiscreteTime
{

	FIRFilter::FIRFilter()
	{
	}

	FIRFilter::~FIRFilter()
	{
	}

	void FIRFilter::setParameters(const std::vector<std::double_t>& coefficients, const std::string& name)
	{
		_b = coefficients;
		for (auto& b : _b)
		{
			_x.push_back(0.00);
		}
		setName(name);
		_isParamsSet = true;
	}

	double FIRFilter::process(std::double_t u)
	{
		if (_isParamsSet && _b.size() > 0U)
		{
			std::size_t count = _b.size() - 1;

			//skip values in _x
			//
			for (std::size_t i = 0U; i < count; i++)
			{
				_x[i] = _x[i + 1];
			}
			//update last value
			_x[count] = u;

			std::double_t val = 0.00;
			std::size_t ccount = 0U;
			for (auto& b : _b)
			{
				val = _x[count] * _b[ccount];
				count--;
				ccount++;
			}
			_y = val;
		}
		return _y;
	}
}

