#include "Block.h"

namespace DiscreteTime
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
}