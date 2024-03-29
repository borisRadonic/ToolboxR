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

namespace CntrlLibrary
{

	enum class FuzzyMembershipFunctionType
	{
		Singleton = 0,
		Gaussian = 1,
		BellShaped = 2,
		Sigmoidal = 3,
		Triangular = 4,
		Trapezoidal = 5,
		PiShaped = 6,
		SShaped = 7,
		ZShaped = 8,
		TrapezoidalInfL = 9,
		TrapezoidalInfR = 10,
		FuzzySmall = 11,
		FuzzyLarge = 12,
		LinearSugeno = 13,
		SingletonSugeno = 14 //only for Sugeno systems (fuzzy set is not used for output)
	};

	enum class FuzzyControllerType
	{
		Mamdani = 0, //More interpretable rule based system
		Sugeno = 1 //Computationally efficient and works with linear techniques, such as PID control
	};

	enum class BooleanOperation
	{
		AndMin = 0,
		AndProduct = 1,
		OrMax = 2,
		OrProbor = 3
	};


	enum class FuzzyImplicationMethod
	{
		Min = 0,
		Prod = 1
	};

	//Aggregation is the process by which the fuzzy sets that represent the outputs of each rule are combined into a single fuzzy set.

	enum class FuzzyAggregationMethod
	{
		Maximum = 0,
		Probor = 1,
		Sum = 3
	};


	enum class DefuzzificationMethod
	{
		Centroid = 0,
		Bisector = 1,
		MiddleOfMaximum = 2,	//The average of the maximum value of the output set
		LargestOfMaximum = 3,
		SmallestOfMaximum = 4,
		wtAver = 5,				//Weighted average of all rule outputs
		wtSum = 6				//Weighted sum of all rule outputs
	};
}
 