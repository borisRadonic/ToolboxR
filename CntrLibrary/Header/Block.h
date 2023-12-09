/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

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

	};

}