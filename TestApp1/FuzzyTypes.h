#pragma once

enum class FuzzyMembershipFunctionType
{
	Singleton = 0,
	Gaussian = 1,
	BellShaped = 2,
	Triangular = 3,
	Trapezoidal = 4,
	TrapezoidalInfL = 5,
	TrapezoidalInfR = 6,
	FuzzySmall = 7,
	FuzzyLarge = 8,
	SingletonSugeno = 9 //only for Sugeno systems (fuzzy set is not used for output)
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
	MiddleOfMaximum = 2, //he average of the maximum value of the output set
	LargestOfMaximum = 3,
	SmallestOfMaximum = 4
};

 