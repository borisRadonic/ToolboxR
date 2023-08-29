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

	/*! \brief Block is a base class for all blocks. Block is an atomic unit.
	 *         with parameters, inputs and outputs.
	 *
	 *  Blocks represent the basic building blocks for models.
	 */
	class Block
	{

	public:

		/*! \Constructor
		 */
		Block();

		Block(const std::string& name);

		/*! \Destructor
		*/
		virtual ~Block();

		/* !Sets the name of the block
		/*!
		  \param name a name of the block
		*/
		inline void setName(const std::string& name)
		{
			_name = name;
		}

		inline std::string getName() const
		{
			return _name;
		}

		std::shared_ptr<BaseSignal> getInputSignal(const std::uint32_t index);
		
		std::shared_ptr<BaseSignal> getOutputSignal(const std::uint32_t index);

		inline std::size_t getNumberOfInputs() const
		{
			return _inputs.size();
		}

		inline std::size_t getNumberOfOutputs() const
		{
			return _outputs.size();
		}

	protected:

		/* !Adds input signal. 
		/*!
		\param input a name of the block
		*/
		void addInput(std::shared_ptr<BaseSignal> input);

		void addOutput(std::shared_ptr<BaseSignal> output);

		

	protected:

		std::string _name;

		SignalPtr _inputs;

		SignalPtr _outputs;
		//std::unique_ptr<BaseSignal> ddd;
		//using VecInputsPtr = std::vector<std::unique_ptr<FuzzyInput>>;

	};

}