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
			this->addInput(_ptrOut);
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