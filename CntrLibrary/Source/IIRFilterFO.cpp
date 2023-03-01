#include "IIRFilterFO.h"

namespace DiscreteTime
{
	IIRFilterFO::IIRFilterFO()
	{
	}

	IIRFilterFO::~IIRFilterFO()
	{
	}
	
	void IIRFilterFO::setParameters(const std::double_t a1, const std::double_t b0, const std::double_t b1, const std::string & name)
	{
		_a1 = a1;
		_b0 = b0;
		_b1 = b1;
		setName(name);
		_isParamsSet = true;
	}

	double IIRFilterFO::process(std::double_t u)
	{
		if (_isParamsSet)
		{
			_x0 = u;
			_y0 = (_b0 * u) + (_b1 * _x1) - (_a1 * _y1);
		
			_x1 = u;
			_y1 = _y0;
		}
		return _y0;
	}
}