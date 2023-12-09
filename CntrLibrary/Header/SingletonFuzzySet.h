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
#include "FuzzySet.h"

namespace CntrlLibrary
{
	/*
	SingletonSugenoFuzzySet Fuzzy set is used only for output and only with Sugeno  system
	*/
	class SingletonFuzzySet final : public FuzzySet
	{
	public:
		SingletonFuzzySet() = delete;

		SingletonFuzzySet(std::double_t c, const std::string& name = "") :m_c(c), FuzzySet(name) {}

		virtual ~SingletonFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;

		virtual std::double_t getMembership(std::double_t y) final;

	private:
		std::double_t m_c;
	};

	class SingletonSugenoFuzzySet final : public FuzzySet
	{
	public:
		SingletonSugenoFuzzySet() = delete;

		SingletonSugenoFuzzySet(std::double_t c, const std::string& name = "") :FuzzySet(name), m_c(c){}

		virtual ~SingletonSugenoFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;

		virtual std::double_t getMembership(std::double_t y) final;

		std::double_t getConstant() const
		{
			return m_c;
		}

	private:
		std::double_t m_c;
	
	};



	class LinearSugenoFuzzySet final : public FuzzySet
	{
	public:
		LinearSugenoFuzzySet() = delete;

		LinearSugenoFuzzySet(std::double_t a, std::double_t b, std::double_t c, const std::string& name = "") 
		:FuzzySet(name), m_a(a), m_b(b), m_c(c)
		{
		}

		virtual ~LinearSugenoFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;


		virtual std::double_t getMembership(std::double_t y) final;

		std::double_t get(std::double_t x, std::double_t y) const
		{
			return (m_a* x + m_b * y + m_c);
		}

	private:

		std::double_t m_a;
		std::double_t m_b;
		std::double_t m_c;

	};



};