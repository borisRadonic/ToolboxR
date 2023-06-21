#pragma once

#include <memory>
#include <string>

namespace CntrlLibrary
{

	class BaseSignal
	{
	public:

		enum class SignalType
		{
			Boolean = 0,
			Int8 = 1,
			UInt8 = 2,
			Int16 = 3,
			UInt16 = 4,
			Int32 = 5,
			UInt32 = 6,
			Float = 7,
			Double = 8,
			EigenDoubleVector = 9,
			Max = 10
		};


		virtual ~BaseSignal()
		{

		}

		BaseSignal() :_type(SignalType::Double)
		{

		}

		BaseSignal(const BaseSignal&) = delete;

		BaseSignal(const BaseSignal&&) = delete;

		BaseSignal& operator=(BaseSignal& rhs) = delete;

		BaseSignal(const std::string& name, BaseSignal::SignalType type) :_signalName(name), _type(type)
		{
		}

		void setName(const std::string& name)
		{
			_signalName = name;
		}

		const std::string getName()
		{
			return _signalName;
		}

		const SignalType getType()
		{
			return _type;
		}

	protected:

		std::string _signalName;

		BaseSignal::SignalType _type;
	};
}