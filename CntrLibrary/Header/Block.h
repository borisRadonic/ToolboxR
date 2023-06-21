#pragma once

#include "Signal.h"
#include "FuzzyInput.h"

#include <string>
#include <vector>
#include <memory>
#include <cmath>

namespace CntrlLibrary
{
	using SignalPtr	= std::vector<std::shared_ptr<BaseSignal>>;

	class Block
	{

	public:

		Block();

		Block(const std::string& name);

		virtual ~Block();

		inline void setName(const std::string& name)
		{
			_name = name;
		}

		inline std::string getName() const
		{
			return _name;
		}

		void addInput(std::shared_ptr<BaseSignal> input);

		void addOutput(std::shared_ptr<BaseSignal> output);

		BaseSignal* getInput(const std::uint32_t index);

		BaseSignal* getOutput(const std::uint32_t index);
							
		inline std::size_t getNumberOfInputs() const
		{
			return _inputs.size();
		}

		inline std::size_t getNumberOfOutputs() const
		{
			return _outputs.size();
		}

	protected:

		std::string _name;

		SignalPtr _inputs;

		SignalPtr _outputs;
		//std::unique_ptr<BaseSignal> ddd;
		//using VecInputsPtr = std::vector<std::unique_ptr<FuzzyInput>>;

	};

}