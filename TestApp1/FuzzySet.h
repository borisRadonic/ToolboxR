
#pragma once



#include <string>

#include "FuzzyTypes.h"

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

	//The core of a membership function is that region of universe that is characterize by full membership in the set. 
	//Hence, core consists of all those elements y of the universe of information such that, μA˜(y) = 1
	virtual std::double_t getFirstCore() = 0;

	const std::string& getName()
	{
		return m_FuzzySetName;
	}
	

private:

	std::string m_FuzzySetName;
	
};

