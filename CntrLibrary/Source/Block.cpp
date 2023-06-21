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

	BaseSignal* Block::getInput(const std ::uint32_t index )
	{
		if (_inputs.size() > index)
		{
			return _inputs.at(index).get();
		}
		return nullptr;
	}

	BaseSignal* Block::getOutput(const std::uint32_t index)
	{
		if (_outputs.size() > index)
		{
			return _outputs.at(index).get();
		}
		return nullptr;
	}
}