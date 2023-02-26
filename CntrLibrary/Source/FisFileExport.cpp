#include "FisFileExport.h"

#include <format>
#include <memory>
#include <algorithm>
#include <sstream>
#include <iostream>

namespace CntrlLibrary
{
    FisFileExport::FisFileExport(FuzzyController* fuzzyController, std::ofstream& file)
        :_fuzzyController(fuzzyController), _file(file)
    {
    }

    FisFileExport::~FisFileExport()
    {
    }

    bool FisFileExport::exportToFIS()
    {
        if (false == writeSectionSystem())
        {
            return false;
        }

        if (false == writeInputSections())
        {
            return false;
        }

        if (false == writeOutputSections())
        {
            return false;
        }

        if (false == writeRulesSection())
        {
            return false;
        }
        return true;
    }

    bool FisFileExport::writeSectionSystem()
    {
        _file << "[System]" << std::endl;
        _file << "Name=" << "'" << _fuzzyController->getControllerName() << "'" << std::endl;

        if (_fuzzyController->getControllerType() == FuzzyControllerType::Mamdani)
        {
            _file << "Type='mamdani'" << std::endl;
        }
        else if (_fuzzyController->getControllerType() == FuzzyControllerType::Sugeno)
        {
            _file << "Type='sugeno'" << std::endl;
        }
        else
        {
            _file << "Type='unknown'" << std::endl;
        }
        _file << "Version=2.0" << std::endl;
        _file << "NumInputs=" << _fuzzyController->getNumberOfInputs() << std::endl;
        _file << "NumOutputs=" << _fuzzyController->getNumberOfOutputs() << std::endl;
        _file << "NumRules=" << _fuzzyController->getNumberOfRules() << std::endl;

        if (_fuzzyController->getBooleanOperationAnd() == BooleanOperation::AndMin)
        {
            _file << "AndMethod='min'" << std::endl;
        }
        else
        {
            _file << "AndMethod='prod'" << std::endl;
        }

        if (_fuzzyController->getBooleanOperationOr() == BooleanOperation::OrMax)
        {
            _file << "OrMethod='max'" << std::endl;
        }
        else if (_fuzzyController->getBooleanOperationOr() == BooleanOperation::OrProbor)
        {
            _file << "OrMethod='probor'" << std::endl;
        }
        else
        {
            _file << "OrMethod='unsupported'" << std::endl;
            return false;
        }

        if (_fuzzyController->getFuzzyImplicationMethod() == FuzzyImplicationMethod::Min)
        {
            _file << "ImpMethod='min'" << std::endl;
        }
        else if (_fuzzyController->getFuzzyImplicationMethod() == FuzzyImplicationMethod::Prod)
        {
            _file << "ImpMethod='prod'" << std::endl;
        }
        else
        {
            _file << "ImpMethod='unsupported'" << std::endl;
            return false;
        }

        if (_fuzzyController->getFuzzyAggregationMethod() == FuzzyAggregationMethod::Maximum)
        {
            _file << "AggMethod='max'" << std::endl;
        }
        else if (_fuzzyController->getFuzzyAggregationMethod() == FuzzyAggregationMethod::Probor)
        {
            _file << "AggMethod='probor'" << std::endl;
        }
        else if (_fuzzyController->getFuzzyAggregationMethod() == FuzzyAggregationMethod::Sum)
        {
            _file << "AggMethod='sum'" << std::endl;
        }
        else
        {
            _file << "AggMethod='unknown'" << std::endl;
            return false;
        }

        if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::Centroid)
        {
            _file << "DefuzzMethod='centroid'" << std::endl;
        }
        else if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::Bisector)
        {
            _file << "DefuzzMethod='bisector'" << std::endl;
        }
        else if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::MiddleOfMaximum)
        {
            _file << "DefuzzMethod='mom'" << std::endl;
        }
        else if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::LargestOfMaximum)
        {
            _file << "DefuzzMethod='lom'" << std::endl;
        }
        else if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::SmallestOfMaximum)
        {
            _file << "DefuzzMethod='som'" << std::endl;
        }
        else if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::wtAver)
        {
            _file << "DefuzzMethod='wtaver'" << std::endl;
        }
        else if (_fuzzyController->getDefuzzificationMethod() == DefuzzificationMethod::wtSum)
        {
            _file << "DefuzzMethod='wtsum'" << std::endl;
        }
        else
        {
            _file << "DefuzzMethod='unsupported'" << std::endl;
            return false;
        }
        _file << std::endl;
        return true;
    }

    bool FisFileExport::writeInputSections()
    {
        std::uint32_t count = 1U;
        for (auto& in : _fuzzyController->getInputs())
        {
            FuzzyInput* input = _fuzzyController->getInput(in);
            if (input != nullptr)
            {
                _file << "[Input" << count << "]" << std::endl;
                count++;

                _file << "Name='" << input->getName() << "'" << std::endl;

                _file << "Range=[" << std::format("{}", input->getMinimum()) << " " << std::format("{}", input->getMaximum()) << "]" << std::endl;

                std::vector<std::string> vecLV = input->getLingvisticValues();

                _file << "NumMFs=" << vecLV.size() << std::endl;

                std::uint32_t countMF = 1U;
                for (auto& lv : vecLV)
                {
                    FuzzySet* fs = input->getFuzzySet(lv);
                    if (fs != nullptr)
                    {
                        _file << "MF" << countMF << "='" << fs->getName() << "'" << ":'" << fs->getMSFTypeNameFIS() << "',";
                        _file << fs->getMSFParamExportFISString() << std::endl;
                        countMF++;
                    }
                }
                _file << std::endl;
            }
        }
        return true;
    }

    bool FisFileExport::writeOutputSections()
    {
        std::uint32_t count = 1U;
        //for (auto& in : _fuzzyController->getOutputs())
        //{
        FuzzyOutput* output = _fuzzyController->getOutput();
        if (output != nullptr)
        {
            _file << "[Output" << count << "]" << std::endl;
            count++;

            _file << "Name='" << output->getName() << "'" << std::endl;

            _file << "Range=[" << output->getMinimum() << " " << output->getMaximum() << "]" << std::endl;

            std::vector<std::string> vecLV = output->getLingvisticValues();

            _file << "NumMFs=" << vecLV.size() << std::endl;

            std::uint32_t countMF = 1U;
            for (auto& lv : vecLV)
            {
                FuzzySet* fs = output->getFuzzySet(lv);
                if (fs != nullptr)
                {
                    _file << "MF" << countMF << "='" << fs->getName() << "'" << ":'" << fs->getMSFTypeNameFIS() << "',";
                    _file << fs->getMSFParamExportFISString() << std::endl;
                    countMF++;
                }
            }
        }
        _file << std::endl;
        //}
        return true;
    }

    bool FisFileExport::writeRulesSection()
    {
        Rules* rules = _fuzzyController->getRules();

        size_t numInputs = _fuzzyController->getNumberOfInputs();
        std::vector<std::string> rnames = rules->getNames();
        _file << "[Rules]" << std::endl;

        for (auto r : rnames)
        {
            Rule* rule = _fuzzyController->getRules()->getRule(r);
            if (rule != nullptr)
            {
                size_t numInputs = _fuzzyController->getNumberOfInputs();
                std::vector<std::string> terms = rule->getInputTerms();
                std::string name = rule->getName();
                std::string outTerm = rule->getOutputTerm();

                std::uint32_t count = 0;
                for (auto& in : _fuzzyController->getInputs())
                {
                    FuzzyInput* input = _fuzzyController->getInput(in);
                    if (input != nullptr)
                    {
                        bool bFound = false;
                        std::vector<std::string> lVec = input->getLingvisticValues();
                        if( count < terms.size() )                        
                        {
                            std::string term = terms[count];
                            for (size_t i = 0; i < lVec.size(); i++)
                            {
                                if (term == lVec[i])
                                {
                                    _file << std::to_string(i + 1);
                                    bFound = true;
                                    break;
                                }                                
                            }
                        }
                        if (false == bFound)
                        {
                            _file << "0";
                        }
                    }
                    count++;
                    if (count < numInputs)
                    {
                        _file << " ";
                    }
                    else
                    {
                        _file << ", ";
                    }
                }
                //write out term number
                std::vector<std::string> lvOutput = _fuzzyController->getOutput()->getLingvisticValues();
                bool bFound = false;
                count = 1;
                for (auto& lvo : lvOutput)
                {
                    if (outTerm == lvo)
                    {
                        _file << std::to_string(count) + " ";
                        bFound = true;
                        break;
                    }
                    count++;
                }
                if (false == bFound)
                {
                    _file << "0 ";
                }
                _file << "(" << std::format("{}", rule->getWeight()) << ") : ";
                BooleanOperation bo = rule->getBooleanType();
                if ((bo == BooleanOperation::AndMin) || (bo == BooleanOperation::AndProduct))
                {
                    _file << "1" << std::endl;
                }
                else
                {
                    _file << "2" << std::endl;
                }
            }
        }
        return true;
    }
}