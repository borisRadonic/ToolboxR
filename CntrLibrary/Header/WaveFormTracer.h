#pragma once

#include "Signal.h"

#include "Block.h"

#include <string>
#include <vector>
#include <cmath>

namespace CntrlLibrary
{
	class WaveFormTracer final : public Block
	{
	public:

		WaveFormTracer();

		~WaveFormTracer();
		
		//Ts - Sample time
		//void setParameters(std::double_t ts);

		//void addSignal( const std::string& name );

		//void push(const std::string& name);
		

	
	private:

//		std::vector<std::shared_ptr<Signal<std::double_t>>> _ptrInputs;
//		std::vector<std::shared_ptr<Signal<std::double_t>>> _ptrOutputs;
		
	};
}

