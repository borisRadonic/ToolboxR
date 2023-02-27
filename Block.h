#pragma once
#include <string>

namespace DiscreteTime
{
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

	protected:
		std::string _name;
	};

}