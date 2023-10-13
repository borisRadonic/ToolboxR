#include "IIRFilterSO.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{

			IIRSecondOrderFilter::IIRSecondOrderFilter()
			{
				/*create input and aouput*/
				_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
				_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

				this->addInput(_ptrIn);
				this->addOutput(_ptrOut);
			}

			IIRSecondOrderFilter::~IIRSecondOrderFilter()
			{
			}

			void IIRSecondOrderFilter::setParameters(const std::double_t a1, const std::double_t a2, const std::double_t b0, const std::double_t b1, const std::double_t b2, const std::string& name)
			{
				_a1 = a1;
				_a2 = a2;
				_b0 = b0;
				_b1 = b1;
				_b2 = b2;
				setName(name);
				_isParamsSet = true;
			}

			double IIRSecondOrderFilter::process(std::double_t u)
			{
				if (_isParamsSet)
				{
					_x0 = u;
					_y0 = (_b0 * u) + (_b1 * _x1) + (_b2 * _x2) - (_a1 * _y1) - (_a2 * _y2);

					_y2 = _y1;
					_y1 = _y0;
					_x2 = _x1;
					_x1 = u;
				}
				_ptrIn->set(u);
				_ptrOut->set(_y0);
				return _y0;
			}

			void IIRSecondOrderFilter::reset()
			{
				_x0 = 0.00;
				_y0 = 0.00;
				_x1 = 0.00;
				_y1 = 0.00;
				_x2 = 0.00;
				_y2 = 0.00;
			}
		}
	}
}