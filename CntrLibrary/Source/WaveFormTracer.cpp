#include "WaveFormTracer.h"

//using namespace DiscreteTime;

namespace CntrlLibrary
{

	WaveFormTracer::WaveFormTracer(const std::string& filename, std::double_t ts)
		: _filename(filename), _ts(ts)
	{		
	}

	WaveFormTracer::~WaveFormTracer()
	{
	}

	bool WaveFormTracer::open()
	{
		_file.open(_filename, std::ios::trunc);
		return _file.is_open();		
	}

	void WaveFormTracer::writeHeader()
	{
		if (_file.is_open())
		{
			/*writes header to file*/
			_file << "Time ";
			for (const auto& signal : _signals)
			{
				_file << signal->getName() << " ";
			}
			_file << std::endl;
		}
	}

	void WaveFormTracer::trace()
	{
		counter++;
		_file << ((std::double_t)counter) * _ts << " ";
		for (const auto& signal : _signals)
		{
			_file << signal->getValueAsString() << " ";
		}
		_file << std::endl;
	}
}