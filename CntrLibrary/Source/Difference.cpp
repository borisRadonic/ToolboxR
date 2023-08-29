#include "Difference.h"
#include <algorithm>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		
		Difference::Difference() :Block(), _old(0.00), _y(0.00)
		{
			/*create input and aouput*/
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}

		Difference::~Difference()
		{
		}

		double Difference::process(std::double_t u)
		{
			_y = u - _old;
			_old = u;

			_ptrIn->set(u);
			_ptrOut->set(_y);

			return _y;
		}
	}
}