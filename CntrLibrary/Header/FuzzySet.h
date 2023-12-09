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

#include <string>
#include "FuzzyTypes.h"
#include <cmath>

namespace CntrlLibrary
{

	//Operations between two fuzzy sets:
	//UNION, INTERSECTION, COMPLEMENT

	#define MIN_DOUBLE_DIFF 0.0000000000000001

/*FuzzySet represents a linguistic Value (for example cold,hot... Lingvistic variable contains one or more lingvistic values (Fuzzy sets)*/
	class FuzzySet
	{
	public:

		FuzzySet() = delete;

		FuzzySet(const std::string& name = "");

		virtual ~FuzzySet();

		virtual FuzzyMembershipFunctionType getMSFType() = 0;

		virtual std::string getMSFTypeNameFIS() = 0;

		virtual std::string getMSFParamExportFISString() = 0;

		//returns degree of membership of y
		virtual std::double_t getMembership(std::double_t y) = 0;

		const std::string& getName()
		{
			return m_FuzzySetName;
		}

	private:

		std::string m_FuzzySetName;

	};
}

