
#pragma once

#include <string>

#include "FuzzyTypes.h"
#include <cmath>

//Operations between two fuzzy sets:
//UNION, INTERSECTION, COMPLEMENT

#define MIN_DOUBLE_DIFF 0.0000000000000001

/*FuzzySet represents a linguistic Value (for example cold,hot... Lingvistic variable contains one or more lingvistic values (Fuzzy sets)*/
class FuzzySet
{
public:
		

	FuzzySet() = delete;

	FuzzySet(const std::string& name = "");

	virtual ~FuzzySet();

	virtual FuzzyMembershipFunctionType getMSFType() = 0;

	//returns degree of membership of y
	virtual std::double_t getMembership(std::double_t y) = 0;

	const std::string& getName()
	{
		return m_FuzzySetName;
	}
	

private:

	std::string m_FuzzySetName;
	
};

