#include "TrapezoidalFuzzySet.h"

TrapezoidalFuzzySet::TrapezoidalFuzzySet(std::double_t a, std::double_t b, std::double_t c, std::double_t d, const std::string & name)
:m_a(a), m_b(b), m_c(c), m_d(d), FuzzySet(name)
{
}

FuzzyMembershipFunctionType TrapezoidalFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::Trapezoidal;
}

std::double_t TrapezoidalFuzzySet::getMembership(std::double_t y)
{
	if (y <= m_a || y >= m_d)
	{
		return std::double_t(0.0);
	}

	//y is inside a and d

	if (y < m_b)
	{
		//y is inside a and b
		if ((m_b - m_a) > 0.0)
		{
			return((y - m_a) / (m_b - m_a));
		}
		else
		{
			if (abs(m_b - m_a) <= MIN_DOUBLE_DIFF)
			{
				//a = b
				return std::double_t(0.0);
			}
			return std::double_t(1.0);
		}
	}

	// y must be inside b and d
	if (y <= m_c)
	{
		return std::double_t(1.0);
	}
	// y must be inside c and d
	
	return ((m_d - y) / (m_d - m_c));
}