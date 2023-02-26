#include "FisFileImport.h"
#include "StringUtil.h"
#include "FuzzyControllerFactory.h"
#include "TriangularFuzzySet.h"
#include "TrapezoidalFuzzySet.h"
#include "TrapezoidalIFuzzySetInfL.h"
#include "TrapezoidalIFuzzySetInfR.h"
#include "GaussianFuzzySet.h"
#include "SigmoidalFuzzySet.h"
#include "PiShapedFuzzySet.h"
#include "SShapedFuzzySet.h"
#include "ZShapedFuzzySet.h"
#include "SingletonFuzzySet.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FisFileImport.h"
#include "FuzzyTypes.h"

#include <memory>
#include <algorithm>
#include <sstream>
#include <iostream>

namespace CntrlLibrary
{
	FisFileImport::FisFileImport()
	{
	}

	FisFileImport::~FisFileImport()
	{
	}

	FisFileImport::FisFileImport(const std::string& filename) :_filename(filename)
	{
	}

	void FisFileImport::setFileName(const std::string& filename)
	{
		_filename = filename;
	}

	bool FisFileImport::readFisFile(std::ostringstream& ossErrors)
	{
		if (_filename == "")
		{
			ossErrors << "Empty file name" << std::endl;
			return false;
		}

		std::ifstream file(_filename);

		if (!file)
		{
			ossErrors << "Error opening file " << _filename << std::endl;
			return false;
		}
		std::string line;
		std::map<std::string, std::map<std::string, std::string>> iniData;
		std::string sectionName;
		std::vector < std::string> rules;
		while (std::getline(file, line))
		{
			if (line[0] == '[' && line[line.size() - 1] == ']')
			{
				sectionName = line.substr(1, line.size() - 2);
				std::transform(sectionName.begin(), sectionName.end(), sectionName.begin(),
					[](unsigned char c) { return std::tolower(c); });
				if (sectionName == "rules")
				{
					while (std::getline(file, line))
					{
						rules.push_back(line);
					}
				}
			}
			else if (!line.empty() && line.find('=') != std::string::npos)
			{
				auto delimiterPos = line.find('=');
				auto key = line.substr(0, delimiterPos);
				auto value = line.substr(delimiterPos + 1);
				iniData[sectionName][key] = value;
			}
		}

		for (auto& [section, key] : iniData)
		{
			if (section == "system")
			{
				readSectionSystem(key);
			}
		}

		if (_sysNumInputs > maxInputs)
		{
			ossErrors << "To many system inputs in " << _filename << std::endl;
			return false;
		}
		if (_sysNumInputs == 0U)
		{
			ossErrors << "Missing system inputs in " << _filename << std::endl;
			return false;
		}

		if (_sysNumOutputs > maxOutputs)
		{
			ossErrors << "To many system outputs in " << _filename << std::endl;
			return false;
		}
		if (_sysNumOutputs == 0U)
		{
			ossErrors << "Missing system outputs in " << _filename << std::endl;
			return false;
		}

		if (_sysNumRules > maxRules)
		{
			ossErrors << "To many rules defined in " << _filename << std::endl;
			return false;
		}
		if (_sysNumRules == 0U)
		{
			ossErrors << "Missing system rules in " << _filename << std::endl;
			return false;
		}

		std::vector<std::string> inputSections;
		std::string inputSec = "input";
		std::uint32_t count = 0;
		for (std::uint32_t i = 0U; i < _sysNumInputs; i++)
		{
			std::string in = inputSec + std::to_string(i + 1);
			inputSections.push_back(in);
		}

		for (auto& [section, key] : iniData)
		{
			count = 1;
			for (auto& sec : inputSections)
			{
				if (section == sec)
				{
					readSectionInOut(key, count, false);
				}
				count++;
			}
		}

		std::vector<std::string> outputSections;
		std::string outSection = "output";
		count = 0;
		for (std::uint32_t i = 0U; i < _sysNumOutputs; i++)
		{
			std::string out = outSection + std::to_string(i + 1);
			outputSections.push_back(out);
		}

		for (auto& [section, key] : iniData)
		{
			count = 1;
			for (auto& sec : outputSections)
			{
				if (section == sec)
				{
					readSectionInOut(key, count, true);
				}
				count++;
			}
		}

		if (rules.size() != _sysNumRules)
		{
			ossErrors << "Number of rules in files does not match  'NumRules'!" << std::endl;
			return false;
		}

		for (auto& r : rules)
		{
			FisRule rule;
			r = StringUtil::ltrim(r);
			std::vector<std::string> tokens = StringUtil::split(r, ":");
			if (tokens.size() != 2U)
			{
				ossErrors << "Sysntax error in rules. Rule " << r << std::endl;
				return false;
			}
			std::uint32_t logicOp = std::stoul(StringUtil::trim(tokens[1]));
			if (logicOp == 1U)
			{
				//AND
				rule.logicOperation = 1;
			}
			else if (logicOp == 2U)
			{
				//OR
				rule.logicOperation = 2;
			}
			else
			{
				ossErrors << "Sysntax error in rules. Unknown logic opearion in Rule " << r << std::endl;
				return false;
			}
			std::size_t pos1 = tokens[0].find("(");
			std::size_t pos2 = tokens[0].find(")");
			if ((pos1 >= tokens[0].size()) || (pos2 >= tokens[0].size()))
			{
				ossErrors << "Sysntax error in rules. Weight in Rule " << r << std::endl;
				return false;
			}

			std::string temp = tokens[0].substr(pos1 + 1, pos2 - pos1 - 1);
			std::double_t weight = std::stod(temp);
			rule.weight = weight;

			StringUtil::remove_substring(tokens[0], "(" + temp + ")");

			temp = tokens[0];

			std::vector<std::string> tokensRest = StringUtil::split(temp, ",");
			if (tokensRest.size() != 2U)
			{
				ossErrors << "Sysntax error in rules. Missing , Rule " << r << std::endl;
				return false;
			}
			std::vector<std::string> t1 = StringUtil::split(tokensRest[0], " ");
			std::vector<std::string> t2 = StringUtil::split(tokensRest[1], " ");

			for (auto& t : t1)
			{
				std::string s = StringUtil::trim(t);
				if (s.size() > 0U)
				{
					if (StringUtil::is_number(s) && std::stoul(s) != 0)
					{

						std::int32_t num = std::stoul(s);
						rule.inputs.push_back(num);
					}
					else if (StringUtil::is_number(s) && std::stoul(s) == 0)
					{
						//do nothing
					}
					else
					{
						ossErrors << "Sysntax error in rules. NaN or = Input in Rule " << r << std::endl;
						return false;
					}
				}
			}

			for (auto& t : t2)
			{
				std::string s = StringUtil::trim(t);
				if (s.size() > 0U)
				{
					if (StringUtil::is_number(s) && std::stoul(s) != 0)
					{

						std::int32_t num = std::stoul(s);
						rule.outputs.push_back(num);
					}
					else if (StringUtil::is_number(s) && std::stoul(s) == 0)
					{
						//do nothing
					}
					else
					{
						ossErrors << "Sysntax error in rules. NaN or = Outputs in Rule " << r << std::endl;
						return false;
					}
				}
			}
			_rules.push_back(rule);
		}
		return true;
	}

	void FisFileImport::readSectionInOut(std::map<std::string, std::string>& key, std::uint32_t inputOutput, bool isOutput)
	{
		FisInputOutput fisIO;
		if (key["Name"] != "")
		{

			std::string name = key["Name"];
			StringUtil::remove_substring(name, "'");
			fisIO.name = name;
			fisIO.strRange = key["Range"];

			std::istringstream iss0(fisIO.strRange);

			std::string mf_range_str;
			std::getline(iss0, mf_range_str, '[');
			std::getline(iss0, mf_range_str, ']');

			std::istringstream iss3(mf_range_str);

			std::double_t r1 = 0.0;
			std::double_t r2 = 0.0;
			if (std::getline(iss3, mf_range_str, ' '))
			{
				r1 = std::stod(mf_range_str);
				std::getline(iss3, mf_range_str, ' ');
				r2 = std::stod(mf_range_str);
				fisIO.range = std::make_pair(r1, r2);
			}

			fisIO.numMFs = std::stoul(key["NumMFs"]);
			if ((fisIO.numMFs > 0U) && (fisIO.numMFs < maxMSF))
			{
				//read MFs
				for (std::uint32_t i = 0; i < fisIO.numMFs; i++)
				{
					std::string ms = "MF" + std::to_string(i + 1);

					FisMF mf;
					if (key[ms].size() > 0U)
					{
						mf.mf = key[ms];
						std::istringstream iss1(mf.mf);
						std::getline(std::getline(iss1, mf.mfName, ':'), mf.mfType, ',');

						StringUtil::remove_substring(mf.mfName, "'");

						mf.num = i + 1;

						std::string mf_params_str;
						std::getline(iss1, mf_params_str, '[');
						std::getline(iss1, mf_params_str, ']');

						// Converting the parameter string into a vector of doubles
						std::istringstream iss2(mf_params_str);
						std::string mf_param_str;
						while (std::getline(iss2, mf_param_str, ' '))
						{
							mf.mfParameters.push_back(std::stod(mf_param_str));
						}
						fisIO.msFuncs.push_back(mf);
					}
				}
			}
			if (false == isOutput)
			{
				_inputs.push_back(fisIO);
			}
			else
			{
				_outputs.push_back(fisIO);
			}
		}
	}

	void FisFileImport::readSectionSystem(std::map<std::string, std::string>& key)
	{
		if (key["Name"] != "")
		{
			_sysName = key["Name"];
			StringUtil::remove_substring(_sysName, "'");
		}
		if (key["Type"] != "")
		{
			_sysType = key["Type"];
		}
		if (key["Version"] != "")
		{
			std::string sysVersion = key["Version"];
			_sysVersion = std::stod(sysVersion);
		}
		if (key["NumInputs"] != "")
		{
			std::string sysNumInputs = key["NumInputs"];
			_sysNumInputs = std::stoul(sysNumInputs);
		}
		if (key["NumOutputs"] != "")
		{
			std::string sysNumOutputs = key["NumOutputs"];
			_sysNumOutputs = std::stoul(sysNumOutputs);
		}
		if (key["NumRules"] != "")
		{
			std::string sysNumRules = key["NumRules"];
			_sysNumRules = std::stoul(sysNumRules);
		}
		if (key["AndMethod"] != "")
		{
			_sysAndMethod = key["AndMethod"];
		}
		if (key["OrMethod"] != "")
		{
			_sysOrMethod = key["OrMethod"];
		}
		if (key["ImpMethod"] != "")
		{
			_sysImpMethod = key["ImpMethod"];
		}
		if (key["AggMethod"] != "")
		{
			_sysAggMethod = key["AggMethod"];
		}
		if (key["DefuzzMethod"] != "")
		{
			_sysDefuzzMethod = key["DefuzzMethod"];
		}
	}

	void FisFileImport::fuzzyInputsToController(FuzzyController* pController)
	{
		for (auto& fIn : _inputs)
		{
			std::unique_ptr<FuzzyInput> input = std::make_unique<FuzzyInput>(fIn.range.first, fIn.range.second, fIn.name);
			for (auto& func : fIn.msFuncs)
			{
				if (func.mfType.find("trimf") < func.mfType.size())
				{
					// Triangular with 3 params
					if (func.mfParameters.size() != 3U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<TriangularFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else if (func.mfType.find("trapmf") < func.mfType.size())
				{
					// Trapezoidal with 4 params
					if (func.mfParameters.size() != 4U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<TrapezoidalFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfParameters[3], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else if (func.mfType.find("gbellmf") < func.mfType.size())
				{
					throw new std::exception("Not supported MF function.");
				}
				else if (func.mfType.find("gaussmf") < func.mfType.size())
				{
					//Gaussian with 2 params
					if (func.mfParameters.size() != 2U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<GaussianFuzzySet>(func.mfParameters[1], func.mfParameters[0], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else if (func.mfType.find("gauss2mf") < func.mfType.size())
				{
					//Gaussian combination with 4 params
					throw new std::exception("Not supported MF function.");
				}
				else if (func.mfType.find("sigmf") < func.mfType.size())
				{
					// Sigmoidal with 2 params
					if (func.mfParameters.size() != 2U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<SigmoidalFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else if (func.mfType.find("dsigmf") < func.mfType.size())
				{
					// Difference between two sigmoidal with 4 params
					throw new std::exception("Not supported MF function.");
				}
				else if (func.mfType.find("psigmf") < func.mfType.size())
				{
					// Product of two sigmoidals 4 params
					throw new std::exception("Not supported MF function.");

				}
				else if (func.mfType.find("pimf") < func.mfType.size())
				{
					// Pi-shaped with 4 params
					if (func.mfParameters.size() != 4U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<PiShapedFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfParameters[3], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else if (func.mfType.find("smf") < func.mfType.size())
				{
					// S-shaped with 2 params
					if (func.mfParameters.size() != 2U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<SShapedFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else if (func.mfType.find("zmf") < func.mfType.size())
				{
					// Z-shaped with 2 params
					if (func.mfParameters.size() != 2U)
					{
						throw new std::exception(" Wrong number of MS input parameters.");
					}
					std::unique_ptr<FuzzySet> fs = std::make_unique<ZShapedFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfName);
					input->addFuzzySet(std::move(fs));
				}
				else
				{
					throw new std::exception("Not supported MF function.");
				}
			}
			pController->addInput(std::move(input));
		}
	}

	void FisFileImport::fuzzyOutputsToController(FuzzyController* pController)
	{
		for (auto& fOut : _outputs)
		{
			std::unique_ptr<FuzzyOutput> output = std::make_unique<FuzzyOutput>(fOut.range.first, fOut.range.second, fOut.name);
			for (auto& func : fOut.msFuncs)
			{
				if (pController->getControllerType() == FuzzyControllerType::Mamdani)
				{
					if (func.mfType.find("trimf") < func.mfType.size())
					{
						// Triangular with 3 params
						if (func.mfParameters.size() != 3U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<TriangularFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("trapmf") < func.mfType.size())
					{
						// Trapezoidal with 4 params
						if (func.mfParameters.size() != 4U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<TrapezoidalFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfParameters[3], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("gbellmf") < func.mfType.size())
					{
						throw new std::exception("Not supported MF function.");
					}
					else if (func.mfType.find("gaussmf") < func.mfType.size())
					{
						//Gaussian with 2 params
						if (func.mfParameters.size() != 2U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<GaussianFuzzySet>(func.mfParameters[1], func.mfParameters[0], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("gauss2mf") < func.mfType.size())
					{
						//Gaussian combination with 4 params
						throw new std::exception("Not supported MF function.");
					}
					else if (func.mfType.find("sigmf") < func.mfType.size())
					{
						// Sigmoidal with 2 params
						if (func.mfParameters.size() != 2U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<SigmoidalFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("dsigmf") < func.mfType.size())
					{
						// Difference between two sigmoidal with 4 params
						throw new std::exception("Not supported MF function.");
					}
					else if (func.mfType.find("psigmf") < func.mfType.size())
					{
						// Product of two sigmoidals 4 params
						throw new std::exception("Not supported MF function.");

					}
					else if (func.mfType.find("pimf") < func.mfType.size())
					{
						// Pi-shaped with 4 params
						if (func.mfParameters.size() != 4U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<PiShapedFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfParameters[3], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("smf") < func.mfType.size())
					{
						// S-shaped with 2 params
						if (func.mfParameters.size() != 2U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<SShapedFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("zmf") < func.mfType.size())
					{
						// Z-shaped with 2 params
						if (func.mfParameters.size() != 2U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<ZShapedFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else
					{
						throw new std::exception("Not supported MF function.");
					}
				}
				else
				{
					//sugeno
					if (func.mfType.find("constant") < func.mfType.size())
					{
						// constant
						if (func.mfParameters.size() != 1U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}
						std::unique_ptr<FuzzySet> fs = std::make_unique<SingletonSugenoFuzzySet>(func.mfParameters[0], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
					else if (func.mfType.find("linear") < func.mfType.size())
					{
						// Clasic linear Sugeno fuzzy set z = a x + by + c
						if (func.mfParameters.size() != 3U)
						{
							throw new std::exception(" Wrong number of MS input parameters.");
						}

						std::unique_ptr<FuzzySet> fs = std::make_unique<LinearSugenoFuzzySet>(func.mfParameters[0], func.mfParameters[1], func.mfParameters[2], func.mfName);
						output->addFuzzySet(std::move(fs));
					}
				}
			}
			pController->addOutput(std::move(output));
		}
	}

	FuzzyController* FisFileImport::toFuzzyController()
	{
		FuzzyController* pController = nullptr;
		if (_sysType.find("mamdani") < _sysType.size())
		{
			pController = FuzzyControllerFactory::createController(FuzzyControllerType::Mamdani, _sysName);
		}
		else if (_sysType.find("sugeno") < _sysType.size())
		{
			pController = FuzzyControllerFactory::createController(FuzzyControllerType::Sugeno, _sysName);
		}
		else
		{
			return nullptr;
		}

		if (pController->getControllerType() == FuzzyControllerType::Mamdani)
		{
			if (_sysDefuzzMethod.find("centroid") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::Centroid);
			}
			else if (_sysDefuzzMethod.find("bisector") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::Bisector);
			}
			else if (_sysDefuzzMethod.find("mom") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::MiddleOfMaximum);
			}
			else if (_sysDefuzzMethod.find("lom") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::LargestOfMaximum);
			}
			else if (_sysDefuzzMethod.find("som") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::SmallestOfMaximum);
			}
		}
		else
		{					
			//sugeno controller
			if (_sysDefuzzMethod.find("wtaver") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::wtAver);
			}
			else if (_sysDefuzzMethod.find("wtsum") < _sysDefuzzMethod.size())
			{
				pController->setDefuzzificationMethod(DefuzzificationMethod::wtSum);
			}
		}
		if (pController->getControllerType() == FuzzyControllerType::Mamdani)
		{
			if (_sysAggMethod.find("max") < _sysAggMethod.size())
			{
				pController->setFuzzyAggregationMethod(FuzzyAggregationMethod::Maximum);
			}
			else if (_sysAggMethod.find("sum") < _sysAggMethod.size())
			{
				pController->setFuzzyAggregationMethod(FuzzyAggregationMethod::Sum);
			}
			else if (_sysAggMethod.find("probor") < _sysAggMethod.size())
			{
				pController->setFuzzyAggregationMethod(FuzzyAggregationMethod::Probor);
			}
			else
			{
				throw new std::exception("Not supported aggregation method.");
			}

		}
		else
		{
			//sugeno
			if (_sysAggMethod.find("max") < _sysAggMethod.size())
			{
				pController->setFuzzyAggregationMethod(FuzzyAggregationMethod::Maximum);
			}
			else if (_sysAggMethod.find("sum") < _sysAggMethod.size())
			{
				pController->setFuzzyAggregationMethod(FuzzyAggregationMethod::Sum);
			}
			else if (_sysAggMethod.find("probor") < _sysAggMethod.size())
			{
				pController->setFuzzyAggregationMethod(FuzzyAggregationMethod::Probor);
			}
			else
			{
				throw new std::exception("Not supported aggregation method.");
			}
		}


		if (pController->getControllerType() == FuzzyControllerType::Mamdani)
		{
			if (_sysImpMethod.find("min") < _sysImpMethod.size())
			{
				pController->setFuzzyImplicationMethod(FuzzyImplicationMethod::Min);
			}
			else if (_sysImpMethod.find("prod") < _sysImpMethod.size())
			{
				pController->setFuzzyImplicationMethod(FuzzyImplicationMethod::Prod);
			}
			else
			{
				throw new std::exception("Not supported implication method.");
			}
		}
		else
		{
			//sugeno
			if (_sysImpMethod.find("min") < _sysImpMethod.size())
			{
				pController->setFuzzyImplicationMethod(FuzzyImplicationMethod::Min);
			}
			else if (_sysImpMethod.find("prod") < _sysImpMethod.size())
			{
				pController->setFuzzyImplicationMethod(FuzzyImplicationMethod::Prod);
			}
			else
			{
				throw new std::exception("Not supported implication method.");
			}
		}

		BooleanOperation booleanTypeOr;

		
		if (_sysOrMethod.find("max") < _sysOrMethod.size())
		{
			booleanTypeOr = BooleanOperation::OrMax;
			pController->setBooleanOperationOr(BooleanOperation::OrMax);
		}
		else if (_sysOrMethod.find("probor") < _sysOrMethod.size())
		{
			booleanTypeOr = BooleanOperation::OrProbor;
			pController->setBooleanOperationOr(BooleanOperation::OrProbor);
		}
		else
		{
			throw new std::exception("Not supported system Or method.");
		}


		BooleanOperation booleanTypeAnd;

		if (_sysAndMethod.find("min") < _sysAndMethod.size())
		{
			booleanTypeAnd = BooleanOperation::AndMin;
			pController->setBooleanOperationAnd(BooleanOperation::AndMin);
		}
		else if (_sysAndMethod.find("prod") < _sysAndMethod.size())
		{
			booleanTypeAnd = BooleanOperation::AndProduct;
			pController->setBooleanOperationAnd(BooleanOperation::AndProduct);
		}
		else
		{
			throw new std::exception("Not supported system And method.");
		}

		fuzzyInputsToController(pController);

		fuzzyOutputsToController(pController);

		//add rules to controller
		//pController->addRule

		std::uint32_t rc = 1;

		for (auto& rule : _rules)
		{
			//rule.inputs
			//rule.weight
			//rule.logicOperation
			std::uint32_t count = 1;

			std::vector<std::string> inVariables;
			std::vector<std::string> linVariables;

			std::vector<std::string> vecHedges;

			std::vector<std::vector<std::string>> vecHedgesVec;

			for (auto& input : rule.inputs)
			{
				std::string in = _inputs[count - 1].name;


				FuzzyInput* fi = pController->getInput(in);
				if (nullptr == fi)
				{
					throw new std::exception("Can not find input!");
				}

				std::string inpName = "";
				for (auto& inp : _inputs)
				{
					if (inp.name == in)
					{
						for (auto& f : inp.msFuncs)
						{
							if (f.num == (abs(input)))
							{
								inpName = f.mfName;
							}
						}
					}
				}

				FuzzySet* fz = fi->getFuzzySet(inpName);
				if (nullptr == fz)
				{
					throw new std::exception("Can not find Fuzzy Set!");
				}

				inVariables.push_back(fi->getName());
				linVariables.push_back(fz->getName());

				if (input < 0)
				{
					vecHedges.push_back("is not");
					vecHedgesVec.push_back(vecHedges);
				}
				else
				{
					vecHedges.push_back("is");
					vecHedgesVec.push_back(vecHedges);
				}
				count++;
			}

			std::string outVariable;
			std::string outLingVariable;

			FuzzyOutput* fo = pController->getOutput();
			if (fo != nullptr)
			{
				std::string outpName = "";
				for (auto& outp : _outputs)
				{
					for (auto& f : outp.msFuncs)
					{
						if (f.num == (abs(rule.outputs[0])))
						{
							outpName = f.mfName;
							break;
						}
					}
					if (outpName.size() > 0U)
					{
						break;
					}
				}
				FuzzySet* fz = fo->getFuzzySet(outpName);
				if (fz != nullptr)
				{
					outVariable = fo->getName();
					outLingVariable = fz->getName();
				}
			}
			BooleanOperation bo = booleanTypeAnd;
			if (rule.logicOperation == 2)
			{
				bo = booleanTypeOr;
			}

			pController->addRule(bo, "rule" + std::to_string(rc), "", inVariables, vecHedgesVec, linVariables, outVariable, outLingVariable, rule.weight);

			rc++;

		}
		return pController;
	}
}