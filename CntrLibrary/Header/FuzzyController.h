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

namespace CntrlLibrary
{
	class FuzzyController
	{
	public:

		using VecInputsPtr = std::vector<std::unique_ptr<FuzzyInput>>;
		using OutputPtr = std::unique_ptr<FuzzyOutput>;

		FuzzyController();

		virtual ~FuzzyController();

		FuzzyController(const FuzzyControllerType& type, const std::string& name);

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

		FuzzyInput* getInput(const std::string& inputName);

		bool setInputValue(const std::string& inputName, const std::double_t value);

		inline std::size_t getNumberOfInputs() const
		{
			return _inputs.size();
		}

		std::vector<std::string> getInputs();

		inline std::size_t getNumberOfOutputs() const
		{
			return 1U;
		}

		FuzzyOutput* getOutput();

		void addOutput(std::unique_ptr<FuzzyOutput> output);

		std::vector<std::string> getOutputs();

		inline void setFuzzyAggregationMethod(const FuzzyAggregationMethod& method)
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

		const std::double_t getOutputValue(const std::string& outputName);

		virtual void addRule(BooleanOperation booleanType,
			const std::string& name,
			const std::string& description, const std::vector<std::string>& inVariables,
			const std::vector<std::vector<std::string>>& hedges,
			const std::vector<std::string>& inLingValues,
			const std::string& outVariable,
			const std::string& outLingValue,
			const std::double_t& weight) = 0; //Optional weight for Mamdani system


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
			_defuzzificationMethod = defuzzificationMethod;
		}

		inline DefuzzificationMethod getDefuzzificationMethod() const
		{
			return _defuzzificationMethod;
		}

		inline size_t getNumberOfRules() const
		{
			return _ptrRules->getNumberOfRules();
		}

		Rules* getRules()
		{
			return _ptrRules.get();
		}

		inline void setBooleanOperationAnd(BooleanOperation andOperation)
		{
			_boolAndOperation = andOperation;
		}

		inline void setBooleanOperationOr(BooleanOperation orOperation)
		{
			_boolOrOperation = orOperation;
		}

		inline BooleanOperation getBooleanOperationAnd() const
		{
			return _boolAndOperation;
		}

		inline BooleanOperation getBooleanOperationOr() const
		{
			return _boolOrOperation;
		}



	protected:

		VecInputsPtr	_inputs;
		OutputPtr	_output;

		std::string _controllerName;

		FuzzyControllerType _controllerType = FuzzyControllerType::Mamdani;

		FuzzyAggregationMethod _agregationmethod = FuzzyAggregationMethod::Maximum;

		FuzzyImplicationMethod _implicationmethod = FuzzyImplicationMethod::Min;

		std::unique_ptr<Rules> _ptrRules;

		std::uint32_t _resolution = 512U;

		DefuzzificationMethod _defuzzificationMethod = DefuzzificationMethod::Centroid;

		BooleanOperation _boolOrOperation = BooleanOperation::OrMax;
		BooleanOperation _boolAndOperation = BooleanOperation::AndMin;

	};
};