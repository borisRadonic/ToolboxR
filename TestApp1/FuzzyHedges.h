#pragma once

#include <string>
#include <cmath>
#include <memory>

class FuzzyHedge
{
public:

	enum class FuzzyHedgeType
	{
		More = 0,	//more
		Very = 1,	//very
		VeryVery = 2,	//very very
		Intensification = 3,	//indeed
		Less = 4,	//less
		MoreOrLess = 5,	//more or less
		Is = 6,
		IsNot = 7,	//not
		NotVery = 8,		//not very
		NotExisting = 9
	};

	FuzzyHedge() = delete;

	FuzzyHedge(const std::string& name = "");

	virtual ~FuzzyHedge();

	virtual FuzzyHedgeType getHedgeType() = 0;

	static FuzzyHedgeType getFuzzyHedgeTypeFromString(const std::string& hedge);

	//input degree of membership 
	//returns transformed degree of membership  
	virtual std::double_t transform(std::double_t u) = 0;

	static std::unique_ptr<FuzzyHedge> create(FuzzyHedgeType type);
	
private:

	std::string m_HedgeName;
};

class FuzzyHedgeMore  final : public FuzzyHedge
{
public:

	FuzzyHedgeMore() :FuzzyHedge("more") {};

	virtual ~FuzzyHedgeMore() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};

class FuzzyHedgeVery  final : public FuzzyHedge
{
public:

	FuzzyHedgeVery() :FuzzyHedge("very") {};

	virtual ~FuzzyHedgeVery() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};


class FuzzyHedgeVeryVery  final : public FuzzyHedge
{
public:

	FuzzyHedgeVeryVery() :FuzzyHedge("very very") {};

	virtual ~FuzzyHedgeVeryVery() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};

class FuzzyHedgeIntensification  final : public FuzzyHedge
{
public:

	FuzzyHedgeIntensification() :FuzzyHedge("indeed") {};

	virtual ~FuzzyHedgeIntensification() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};

class FuzzyHedgeLess  final : public FuzzyHedge
{
public:

	FuzzyHedgeLess() :FuzzyHedge("less") {};

	virtual ~FuzzyHedgeLess() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};

class FuzzyHedgeMoreOrLess  final : public FuzzyHedge
{
public:

	FuzzyHedgeMoreOrLess() :FuzzyHedge("more or less") {};

	virtual ~FuzzyHedgeMoreOrLess() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};

class FuzzyHedgeNot  final : public FuzzyHedge
{
public:

	FuzzyHedgeNot() :FuzzyHedge("not") {};

	virtual ~FuzzyHedgeNot() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};


class FuzzyHedgeNotVery  final : public FuzzyHedge
{
public:

	FuzzyHedgeNotVery() :FuzzyHedge("not very") {};

	virtual ~FuzzyHedgeNotVery() {}

	virtual FuzzyHedgeType getHedgeType() final;

	virtual std::double_t transform(std::double_t u) final;
};


class FuzzyHedgeIs  final : public FuzzyHedge
{
public:

	FuzzyHedgeIs() :FuzzyHedge("is") {};

	virtual ~FuzzyHedgeIs() {}

	virtual FuzzyHedgeType getHedgeType() final
	{
		return FuzzyHedge::FuzzyHedgeType::Is;
	}

	virtual std::double_t transform(std::double_t u) final
	{
		return u;
	}
};
