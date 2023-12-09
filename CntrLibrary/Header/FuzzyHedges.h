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
#include <cmath>
#include <memory>

namespace CntrlLibrary
{
	class FuzzyHedge
	{
	public:

		enum class FuzzyHedgeType
		{
			More = 0,	//more
			Very = 1,	//very
			VeryVery = 2,	//very very
			Intensification = 3,	//indeed
			Less = 4,	//less
			MoreOrLess = 5,	//more or less
			Is = 6,
			IsNot = 7,	//not
			NotVery = 8,		//not very
			NotExisting = 9
		};

		FuzzyHedge() = delete;

		FuzzyHedge(const std::string& name = "");

		virtual ~FuzzyHedge();

		virtual FuzzyHedgeType getHedgeType() = 0;

		static FuzzyHedgeType getFuzzyHedgeTypeFromString(const std::string& hedge);

		//input degree of membership 
		//returns transformed degree of membership  
		virtual std::double_t transform(std::double_t u) = 0;

		static std::unique_ptr<FuzzyHedge> create(FuzzyHedgeType type);

	private:

		std::string m_HedgeName;
	};

	class FuzzyHedgeMore  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeMore() :FuzzyHedge("more") {};

		virtual ~FuzzyHedgeMore() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};

	class FuzzyHedgeVery  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeVery() :FuzzyHedge("very") {};

		virtual ~FuzzyHedgeVery() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};


	class FuzzyHedgeVeryVery  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeVeryVery() :FuzzyHedge("very very") {};

		virtual ~FuzzyHedgeVeryVery() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};

	class FuzzyHedgeIntensification  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeIntensification() :FuzzyHedge("indeed") {};

		virtual ~FuzzyHedgeIntensification() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};

	class FuzzyHedgeLess  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeLess() :FuzzyHedge("less") {};

		virtual ~FuzzyHedgeLess() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};

	class FuzzyHedgeMoreOrLess  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeMoreOrLess() :FuzzyHedge("more or less") {};

		virtual ~FuzzyHedgeMoreOrLess() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};

	class FuzzyHedgeNot  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeNot() :FuzzyHedge("not") {};

		virtual ~FuzzyHedgeNot() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};


	class FuzzyHedgeNotVery  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeNotVery() :FuzzyHedge("not very") {};

		virtual ~FuzzyHedgeNotVery() {}

		virtual FuzzyHedgeType getHedgeType() final;

		virtual std::double_t transform(std::double_t u) final;
	};


	class FuzzyHedgeIs  final : public FuzzyHedge
	{
	public:

		FuzzyHedgeIs() :FuzzyHedge("is") {};

		virtual ~FuzzyHedgeIs() {}

		virtual FuzzyHedgeType getHedgeType() final
		{
			return FuzzyHedge::FuzzyHedgeType::Is;
		}

		virtual std::double_t transform(std::double_t u) final
		{
			return u;
		}
	};
};