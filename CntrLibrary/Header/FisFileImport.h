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
#include "FuzzyController.h"

#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

namespace CntrlLibrary
{
	constexpr std::uint32_t maxInputs = 1000U;
	constexpr std::uint32_t maxOutputs = 1000U;
	constexpr std::uint32_t maxMSF = 100U;
	constexpr std::uint32_t maxRules = 1000U;

	struct FisRule
	{
		std::vector<std::int32_t> inputs;
		std::vector<std::int32_t> outputs;
		std::uint32_t			 logicOperation = 0U;
		std::double_t			 weight = 0U;
	};

	struct FisMF
	{
		std::uint32_t num;
		std::string mf;
		std::string mfName;
		std::string mfType;
		std::vector<std::double_t> mfParameters;
	};

	// Name     Parameters  Type							Supported
	// -------------------------------------------------------------- 
	// trimf	3 params	Triangular 							yes
	// trapmf	4 params	Trapezoidal 						yes
	// gbellmf	3 params	Generalized bell-shaped				No
	// gaussmf	2 params	Gaussian							Yes
	// gauss2mf 4 params	Gaussian combination				No
	// sigmf	2 params	Sigmoidal							Yes
	// dsigmf	4 params	Difference between two sigmoidal	No
	// psigmf	4 params	Product of two sigmoidals			No 
	// pimf		4 params	Pi-shaped membership function		Yes
	// smf		2 params	S-shaped membership function		Yes
	// zmf		2 params	Z-shaped membership function		Yes
	// constant	1 params	Sugeno								Yes? TODO
	// linear				Sugeno 								No
	// ...					Not supported: customs MATLAB...	No

	struct FisInputOutput
	{
		std::string								name;
		std::string								strRange;
		std::uint32_t							numMFs;
		std::pair<std::double_t, std::double_t> range;
		std::vector<FisMF>						msFuncs;
	};

	class FisFileImport
	{
	public:
		FisFileImport();

		FisFileImport(const std::string& filename);

		virtual ~FisFileImport();

		void setFileName(const std::string& filename);

		bool readFisFile(std::ostringstream& ossErrors);

		FuzzyController* toFuzzyController();

	protected:

		void readSectionSystem(std::map<std::string, std::string>& key);
		void readSectionInOut(std::map<std::string, std::string>& key, std::uint32_t inputOutput, bool isOutput);
		void fuzzyInputsToController(FuzzyController* pController);
		void fuzzyOutputsToController(FuzzyController* pController);

	private:

		std::string		_filename;
		std::string		_sysName;
		std::string		_sysType;
		std::double_t	_sysVersion = 0.00;
		std::uint32_t	_sysNumInputs = 0U;
		std::uint32_t	_sysNumOutputs = 0U;
		std::uint32_t	_sysNumRules = 0U;
		std::string		_sysAndMethod;
		std::string		_sysOrMethod;
		std::string		_sysImpMethod;
		std::string		_sysAggMethod;
		std::string		_sysDefuzzMethod;

		std::vector<FisInputOutput> _inputs;
		std::vector<FisInputOutput> _outputs;
		std::vector<FisRule>		_rules;
	};
};

