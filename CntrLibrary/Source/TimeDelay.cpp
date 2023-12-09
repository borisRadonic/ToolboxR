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

#include "TimeDelay.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{

		TimeDelay::TimeDelay() :Block()
		{
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}

		TimeDelay::~TimeDelay()
		{
		}

		void TimeDelay::setParameters(const std::uint32_t lenght)
		{
			_isParamsSet = true;

			_lenght = lenght;
			for (uint32_t i = 0; i < _lenght; i++)
			{
				_queue.push(0.00);
			}
		}

		double TimeDelay::process(const std::double_t u)
		{
			std::double_t ret = 0.00;
			if (_isParamsSet)
			{
				if (_queue.size() >= _lenght)
				{

					_queue.push(u);
					ret = _queue.front();
					_queue.pop();
				}
			}
			_ptrIn->set(u);
			_ptrOut->set(ret);
			return ret;
		}

		void TimeDelay::reset()
		{
			std::queue<std::double_t> empty_queue;
			for (uint32_t i = 0; i < _lenght; i++)
			{
				empty_queue.push(0.00);
			}
			_queue.swap(empty_queue);
		}
	}
}