#pragma once

#include "BaseSignal.h"
#include <memory>
#include <string>

namespace CntrlLibrary
{
	template <typename T>
	class Signal  final: public BaseSignal
	{
		using value_type = T;

	public:
	
		Signal() = delete;

		virtual ~Signal()
		{

		}

		Signal(const Signal&) = delete;
					
		Signal(const Signal&&) = delete;

		Signal& operator=(Signal&rhs)
		{
			this->_value = rhs._value;
		}

		Signal(const std::string& name, SignalType type ):BaseSignal(name, type)
		{
		}

		inline void set(const value_type value)
		{
			_value = value;
		}

		inline value_type get()
		{
			return _value;
		}
		
		struct Factory
		{
			static std::unique_ptr<Signal<T>> NewSignal(const std::string& name, SignalType type)
			{
				return std::make_unique <Signal<T>>(name, type);
			}
		};

	protected:
		value_type _value = 0;
	};
}