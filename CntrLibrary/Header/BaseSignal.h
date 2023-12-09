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

/*! \file BaseSignal.h
	\brief Defines Signal types and BaseSignal class that is  a base class for all signals.
*/

#pragma once

#include <memory>
#include <string>

namespace CntrlLibrary
{
	/*! \brief The base class for all signals.
	 *
	 */
	class BaseSignal
	{
	public:

		/*! \brief enum SignalType defines the data types of the input/output signals.
		*
		*/
		enum class SignalType
		{
			Boolean				= 0,	/**< Boolean type. Size: 1 byte*/
			Int8				= 1,	/**< 8 bit signed integer. Size: 1 byte*/
			UInt8				= 2,	/**< 8 bit unsigned integer. Size: 1 byte*/
			Int16				= 3,	/**< 16 bit signed integer. Size: 2 bytes*/
			UInt16				= 4,	/**< 16 bit unsigned integer. Size: 2 bytes*/
			Int32				= 5,	/**< 32 bit signed integer. Size: 4 bytes*/
			UInt32				= 6,	/**< 32 bit unsigned integer. Size: 4 bytes*/
			Int64				= 7,	/**< 64 bit signed integer. Size: 8 bytes*/
			UInt64				= 8,	/**< 64 bit unsigned integer. Size: 8 bytes*/
			Float				= 9,	/**< 32-bit IEEE 754 single precision floating point number. Size: 4 bytes*/
			Double				= 10,	/**< 64-bit IEEE 754 double precision floating point number. Size: 4 bytes*/
			EigenDoubleVector	= 11,	/**< Vector of 'double' types */ 
			Max = 10
		};

		/**
		* A destructor.
		*/
		virtual ~BaseSignal()
		{

		}


		/**
		 * A constructor.
		 * Constructs a sygnal of default type 'double'. The type can be changed after
		 * construction by calling 'setType' function
		 */
		BaseSignal() :_type(SignalType::Double)
		{

		}

		BaseSignal(const BaseSignal&) = delete;

		BaseSignal(const BaseSignal&&) = delete;

		BaseSignal& operator=(BaseSignal& rhs) = delete;

		/**
		 * A constructor.
		 * 
		 * @param name an string.
		 * @param type an SignalType.
		 */
		BaseSignal(const std::string& name, BaseSignal::SignalType type) :_signalName(name), _type(type)
		{
		}

		virtual std::string getValueAsString() const = 0;

		/**
		 * Sets the name of the signal.
		 *
		 * @param name an string.
		 */
		void setName(const std::string& name)
		{
			_signalName = name;
		}

		const std::string getName()
		{
			return _signalName;
		}

		/**
		 * Sets the type of the signal.
		 *
		 * @param type an SignalType.
		 */
		void setType(const SignalType type)
		{
			_type = type;
		}


		/**
		  * 
		  * @return the sygnal type.
		 */
		const SignalType getType()
		{
			return _type;
		}

		

	protected:

		std::string _signalName;

		BaseSignal::SignalType _type;
	};
}