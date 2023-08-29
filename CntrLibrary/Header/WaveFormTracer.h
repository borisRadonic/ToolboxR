#pragma once

#include "Signal.h"
#include "Block.h"

#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>

namespace CntrlLibrary
{
	class WaveFormTracer final : public Block
	{
	public:

		WaveFormTracer() = delete;

		explicit WaveFormTracer(const std::string& filename, std::double_t ts);

		~WaveFormTracer();

		bool open();
		
		void writeHeader();
				
		template <typename T>
		std::shared_ptr<Signal<T>> addSignal(const std::string& name, BaseSignal::SignalType type)
		{
			auto signal = Signal<T>::Factory::NewSignal(name, type);
			addBlockSignal(signal);
			return signal;
		}

		void addBlockSignal(std::shared_ptr<BaseSignal> signal)
		{
			_signals.push_back(signal);
		}

		// Writes the state of all signals to a given file.
		void trace();
		
	private:

		std::uint32_t counter = 0U;
		std::string _filename;
		std::double_t _ts;
		std::vector<std::shared_ptr<BaseSignal>> _signals;
		std::ofstream _file;

		std::string getSignalValueAsString(BaseSignal* signal);

	};
}

