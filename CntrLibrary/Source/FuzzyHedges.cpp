#include "FuzzyHedges.h"

namespace CntrlLibrary
{
	FuzzyHedge::FuzzyHedge(const std::string& name)
		:m_HedgeName(name)
	{
	}

	FuzzyHedge::~FuzzyHedge()
	{
	}

	FuzzyHedge::FuzzyHedgeType FuzzyHedge::getFuzzyHedgeTypeFromString(const std::string& hedge)
	{
		if (hedge == "more")
		{
			return FuzzyHedgeType::More;
		}
		else if (hedge == "very")
		{
			return FuzzyHedgeType::Very;
		}
		else if (hedge == "very very")
		{
			return FuzzyHedgeType::VeryVery;
		}
		else if (hedge == "indeed")
		{
			return FuzzyHedgeType::Intensification;
		}
		else if (hedge == "less")
		{
			return FuzzyHedgeType::Less;
		}
		else if (hedge == "more or less")
		{
			return FuzzyHedgeType::MoreOrLess;
		}
		else if (hedge == "is")
		{
			return FuzzyHedgeType::Is;
		}
		else if (hedge == "is not")
		{
			return FuzzyHedgeType::IsNot;
		}
		else if (hedge == "not")
		{
			return FuzzyHedgeType::IsNot;
		}
		else if (hedge == "not very")
		{
			return FuzzyHedgeType::NotVery;
		}
		return FuzzyHedgeType::NotExisting;
	}

	std::unique_ptr<FuzzyHedge> FuzzyHedge::create(FuzzyHedge::FuzzyHedgeType type)
	{
		switch (type)
		{
		case FuzzyHedge::FuzzyHedgeType::More:
		{
			return std::make_unique<FuzzyHedgeMore>();
		}
		case FuzzyHedge::FuzzyHedgeType::Very:
		{
			return std::make_unique<FuzzyHedgeVery>();
		}
		case FuzzyHedge::FuzzyHedgeType::VeryVery:
		{
			return std::make_unique<FuzzyHedgeVeryVery>();
		}
		case FuzzyHedge::FuzzyHedgeType::Intensification:
		{
			return std::make_unique<FuzzyHedgeIntensification>();
		}
		case FuzzyHedge::FuzzyHedgeType::Less:
		{
			return std::make_unique<FuzzyHedgeLess>();
		}
		case FuzzyHedge::FuzzyHedgeType::MoreOrLess:
		{
			return std::make_unique<FuzzyHedgeMoreOrLess>();
		}
		case FuzzyHedge::FuzzyHedgeType::Is:
		{
			return std::make_unique<FuzzyHedgeIs>();
		}
		case FuzzyHedge::FuzzyHedgeType::IsNot:
		{
			return std::make_unique<FuzzyHedgeNot>();
		}
		case FuzzyHedge::FuzzyHedgeType::NotVery:
		{
			return std::make_unique<FuzzyHedgeNotVery>();
		}
		case FuzzyHedge::FuzzyHedgeType::NotExisting:
		default:
			break;
		}
		return nullptr;
	}

	//FuzzyHedgeMore
	inline FuzzyHedge::FuzzyHedgeType FuzzyHedgeMore::getHedgeType()
	{
		return FuzzyHedgeType::More;
	}

	inline std::double_t FuzzyHedgeMore::transform(std::double_t u)
	{
		return pow(u, std::double_t(1.25));
	}

	//FuzzyHedgeVery
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeVery::getHedgeType()
	{
		return FuzzyHedgeType::Very;
	}

	std::double_t FuzzyHedgeVery::transform(std::double_t u)
	{
		return pow(u, std::double_t(2.0));
	}


	//FuzzyHedgeVeryVery
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeVeryVery::getHedgeType()
	{
		return FuzzyHedgeType::VeryVery;
	}

	std::double_t FuzzyHedgeVeryVery::transform(std::double_t u)
	{
		return pow(u, std::double_t(4.0));
	}

	//FuzzyHedgeIntensification
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeIntensification::getHedgeType()
	{
		return FuzzyHedgeType::Intensification;
	}

	std::double_t FuzzyHedgeIntensification::transform(std::double_t u)
	{
		if (u < 0.5)
		{
			return std::double_t(2.0 * pow(u, 2.0));
		}
		return std::double_t(1.0 - 2.0 * pow(1 - u, 2.0));
	}

	//FuzzyHedgeLess
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeLess::getHedgeType()
	{
		return FuzzyHedgeType::Less;
	}

	std::double_t FuzzyHedgeLess::transform(std::double_t u)
	{
		if (u > 0.0)
		{
			return pow(u, std::double_t(0.75));
		}
		return std::double_t(0.0);
	}

	//FuzzyHedgeMoreOrLess
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeMoreOrLess::getHedgeType()
	{
		return FuzzyHedgeType::MoreOrLess;
	}

	std::double_t FuzzyHedgeMoreOrLess::transform(std::double_t u)
	{
		if (u > 0.0)
		{
			return std::double_t(sqrt(u));
		}
		return std::double_t(0.0);
	}

	//FuzzyHedgeNot
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeNot::getHedgeType()
	{
		return FuzzyHedgeType::MoreOrLess;
	}

	std::double_t FuzzyHedgeNot::transform(std::double_t u)
	{
		if (u > 0.0)
		{
			return std::double_t(1.0 - u);
		}
		return std::double_t(0.0);
	}

	//FuzzyHedgeNotVery
	FuzzyHedge::FuzzyHedgeType FuzzyHedgeNotVery::getHedgeType()
	{
		return FuzzyHedgeType::NotVery;
	}

	std::double_t FuzzyHedgeNotVery::transform(std::double_t u)
	{
		return std::double_t(1.0 - u * u);
	}
}