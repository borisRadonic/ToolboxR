#pragma once

#include <string>
#include <vector>
#include <vector>
#include <map>
#include <memory>

#include "FuzzyHedges.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FuzzyTypes.h"


class Rule
{
public:

	Rule();

	Rule(const std::string& name, const std::string& definition);

	Rule(const Rule&) = delete;

	Rule& operator=(Rule&) = delete;

	~Rule();

	void addHedge(const std::string& inputName, std::unique_ptr<FuzzyHedge> hedge);

	void addInput(FuzzyInput* input);

	void addOutput(FuzzyOutput* output);
	
	const std::string getName();

	std::double_t process();

	void addInputTerm(const std::string& term);

	inline void setBooleanType(BooleanOperation bolianType)
	{
		_boleanType = bolianType;
	}

	inline BooleanOperation getBooleanType()
	{
		return _boleanType;
	}

	inline void setOutputTerm(const std::string& term)
	{
		_outputTerm = term;
	}

	inline const std::string& getOutputTerm() const
	{
		return _outputTerm;
	}

	inline FuzzyOutput* getFuzzyOutput() const
	{
		return _ptrOutput;
	}

	inline void setControllerType(FuzzyControllerType type)
	{
		_controllerType = type;
	}

	inline void setWeight(std::double_t weight)
	{
		_weight = weight;
	}

	inline std::double_t getWeight() const
	{
		return _weight;
	}
	
	void compile();

	void setCoefficientsC(const std::vector<std::double_t>& ccoeffs, std::double_t c0);

	std::double_t getCoefficient(const std::uint32_t i);
	
	inline std::size_t getnumberOfInputs() const
	{
		return _inputTerms.size();
	}

	std::double_t getInputValue(const std::uint32_t i);

	inline std::double_t getoffsetC0() const
	{
		return _offsetC0;
	}

private:

	std::string _name;
	std::string _definition;

	using MapHedgesPtr = std::map< std::string, std::vector<std::unique_ptr<FuzzyHedge>>>;

	using InputPtr = std::vector<FuzzyInput*>;

	MapHedgesPtr _hedges; //should be access from end to beginn

	InputPtr _ptrInputs;

	std::vector<std::string> _inputTerms;

	FuzzyOutput* _ptrOutput = nullptr;

	std::string _outputTerm;

	BooleanOperation _boleanType = BooleanOperation::AndMin;

	FuzzyControllerType _controllerType = FuzzyControllerType::Mamdani;
	
	std::vector<std::double_t> _ccoeffs; //used unly for Sugeno

	std::double_t _offsetC0 = 0.00; //used unly for Sugeno

	std::double_t _weight = 1.0;
		
};

