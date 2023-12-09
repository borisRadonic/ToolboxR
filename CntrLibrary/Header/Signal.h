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

#include "BaseSignal.h"
#include <memory>
#include <string>
#include <fstream>
#include <sstream>

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

		inline virtual std::string getValueAsString() const override
		{
			std::ostringstream oss;
			oss << _value;
			return oss.str();
		}
					
		struct Factory
		{
			static std::shared_ptr<Signal<T>> NewSignal(const std::string& name, SignalType type)
			{
				return std::make_shared <Signal<T>>(name, type);
			}
		};

	protected:
		value_type _value = 0;
	};
}