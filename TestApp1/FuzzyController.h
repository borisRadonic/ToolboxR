#pragma once
#include <vector>
#include <string>
#include <memory>
#include "FuzzyInput.h"
#include "Fuzzyoutput.h"
#include "FuzzyHedges.h"
#include "rule.h"
#include "Rules.h"
#include "FuzzyTypes.h"

class FuzzyController
{
public:

	FuzzyController();

	virtual ~FuzzyController();

	FuzzyController(const FuzzyControllerType& type, const std::string& name );

	void setControllerName(const std::string& name)
	{
		_controllerName = name;
	}

	const std::string& getControllerName()
	{
		return _controllerName;
	}

	const FuzzyControllerType& getControllerType()
	{
		return _controllerType;
	}

	/*returns Inputs identifier*/
	void addInput(std::unique_ptr<FuzzyInput> input);

	FuzzyInput* getInput(const std::string & inputName);

	bool setInputValue(const std::string & inputName, const std::double_t value);
	
	FuzzyOutput* getOutput();

	void addOutput(std::unique_ptr<FuzzyOutput> output);
	
	inline void setFuzzyAggregationMethod( const FuzzyAggregationMethod& method )
	{
		_agregationmethod = method;
	}

	inline FuzzyAggregationMethod getFuzzyAggregationMethod() const
	{
		 return _agregationmethod;
	}

	inline void setFuzzyImplicationMethod(const FuzzyImplicationMethod& method)
	{
		_implicationmethod = method;
	}

	inline FuzzyImplicationMethod getFuzzyImplicationMethod() const
	{
		return _implicationmethod;
	}

	const std::double_t getOutputValue(const std::string & outputName);

	virtual void addRule(	BooleanOperation booleanType,
							const std::string & name,
							const std::string & description, const std::vector<std::string>& inVariables,
							const std::vector<std::vector<std::string>>& hedges,
							const std::vector<std::string>& inLingValues,
							const std::string & outVariable,
							const std::string & outLingValue,
							const std::vector<double_t>& thenCoefsC, //THEN C1...Cr for Sugeno system
							const std::double_t & offset, //THEN C0 for Sugeno system  y = c0 + C1*X1 + c2*X2...Cn*Xn
							const std::double_t & weight) = 0; //Optional weight for Mamdani system
	

	inline void setResolution(std::uint32_t resolution)
	{
		_resolution = resolution;
	}

	inline std::uint32_t getResolution() const
	{
		return _resolution;
	}
	
	virtual void process();

	virtual void compile();

	inline void setDefuzzificationMethod(DefuzzificationMethod defuzzificationMethod)
	{
		_defuzzificationMethod = _defuzzificationMethod;
	}

	inline DefuzzificationMethod getDefuzzificationMethod() const
	{
		return _defuzzificationMethod;
	}
	
protected:

	using MapInputsPtr	= std::map<std::string, std::unique_ptr<FuzzyInput>>;
	using MapOutputPtr = std::unique_ptr<FuzzyOutput>;

	MapInputsPtr	_inputs;
	MapOutputPtr	_output;

	std::string _controllerName;

	FuzzyControllerType _controllerType = FuzzyControllerType::Mamdani;

	FuzzyAggregationMethod _agregationmethod = FuzzyAggregationMethod::Maximum;

	FuzzyImplicationMethod _implicationmethod = FuzzyImplicationMethod::Min;
	
	std::unique_ptr<Rules> _ptrRules;

	std::uint32_t _resolution = 512U;
	
	DefuzzificationMethod _defuzzificationMethod = DefuzzificationMethod::Centroid;
};

