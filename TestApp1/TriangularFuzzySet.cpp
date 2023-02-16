#include "TriangularFuzzySet.h"


TriangularFuzzySet::TriangularFuzzySet(std::double_t a, std::double_t b, std::double_t c, const std::string & name)
	:m_a(a), m_b(b), m_c(c), FuzzySet(name)
{
}

FuzzyMembershipFunctionType TriangularFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::Triangular;
}

std::double_t TriangularFuzzySet::getMembership(std::double_t y)
{
	if( (y <= m_a) || (y >= m_c))
	{
		return std::double_t(0.0);
	}

	//y is inside a and c
	if (y <= m_b)
	{
		//y is inside a and b
		if ((m_b - m_a) > 0.0)
		{
			//y is inside a and b
			return((y - m_a) / (m_b - m_a));
		}
		else
		{
			//something is wrong
			if (abs(m_b - m_a) <= MIN_DOUBLE_DIFF)
			{
				//a = b
				return std::double_t(1.0);
			}
			//b>a !!!!!
			return std::double_t(0.0);
		}
	}
	//y is inside b and c
	return ((m_c - y) / (m_c - m_b));
}

std::double_t TriangularFuzzySet::getFirstCore()
{
	return m_b;
}

