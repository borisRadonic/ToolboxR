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

#include "Block.h"

namespace CntrlLibrary
{
	Block::Block()
		:_name("")
	{
	}

	Block::Block(const std::string& name) :_name(name)
	{

	}

	Block::~Block()
	{
	}

	void Block::addInput(std::shared_ptr<BaseSignal> input)
	{
		if (nullptr != input)
		{
			_inputs.push_back(input);
		}
	}

	void Block::addOutput(std::shared_ptr<BaseSignal> output)
	{
		if (nullptr != output)
		{
			_outputs.push_back(output);
		}
	}

	std::shared_ptr<BaseSignal> Block::getInputSignal(const std::uint32_t index)
	{
		if (_inputs.size() > index)
		{
			return _inputs.at(index);
		}
		return nullptr;
	}
	
	std::shared_ptr<BaseSignal> Block::getOutputSignal(const std::uint32_t index)
	{
		if (_outputs.size() > index)
		{
			return _outputs.at(index);
		}
		return nullptr;
	}
}