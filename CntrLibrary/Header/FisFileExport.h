#pragma once
#include "FuzzyController.h"

#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

namespace CntrlLibrary
{
	class FisFileExport
	{
	public:

		FisFileExport() = delete;

		FisFileExport(FuzzyController* fuzzyController, std::ofstream& file);

		virtual ~FisFileExport();

		bool exportToFIS();

	protected:

		bool writeSectionSystem();
		bool writeInputSections();
		bool writeOutputSections();
		bool writeRulesSection();

	private:

		std::ofstream& _file;
		FuzzyController* _fuzzyController;

	};
};
