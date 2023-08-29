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