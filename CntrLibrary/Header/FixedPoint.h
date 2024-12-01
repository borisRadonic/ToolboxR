
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
#include <cstdint>
#include <algorithm> // For std::clamp
#include <array>
#include <cmath>
#include <numbers>

namespace CntrlLibrary
{
	// Generic clamp function for saturation
	template <typename T>
	constexpr T saturate(T value, T min, T max)
    {
		return std::clamp(value, min, max);
	}

    template <int IntegerBits, int FractionalBits>
    class FixedPoint
    {
    
        static_assert(IntegerBits + FractionalBits <= 32, "Only up to 32-bit fixed-point numbers are supported.");
        
        static_assert(IntegerBits > 0 && FractionalBits > 0, "IntegerBits and FractionalBits must be greater than 0.");


    public:

        using UnderlyingType = int32_t; // Use 32-bit signed integer for storage
        
        // Maximum and minimum values for this fixed-point type
        static constexpr UnderlyingType MaxValue = (1 << (IntegerBits + FractionalBits - 1)) - 1;
        static constexpr UnderlyingType MinValue = -(1 << (IntegerBits + FractionalBits - 1));

        // Constructors
        constexpr FixedPoint() : value(0)
        {
        }

        constexpr explicit FixedPoint(float f) : value(static_cast<UnderlyingType>(f* (1 << FractionalBits)))
        {
        }
       

        constexpr explicit FixedPoint(UnderlyingType raw) : value(saturate(raw))
        {
        }

        // Conversion to float
        constexpr float toFloat() const
        {
            return static_cast<float>(value) / (1 << FractionalBits);
        }

        // Get raw value
        constexpr UnderlyingType raw() const
        {
            return value;
        }

        // Arithmetic operators
        constexpr FixedPoint operator+(const FixedPoint& other) const
        {
            return FixedPoint(saturate(value + other.value));
        }

        constexpr FixedPoint operator-(const FixedPoint& other) const
        {
            return FixedPoint(saturate(value - other.value));
        }

        constexpr FixedPoint operator*(const FixedPoint& other) const
        {
            int64_t temp = static_cast<int64_t>(value) * other.value;
            return FixedPoint(saturate(static_cast<UnderlyingType>(temp >> FractionalBits)));
        }

        constexpr FixedPoint operator/(const FixedPoint& other) const
        {
            int64_t temp = (static_cast<int64_t>(value) << FractionalBits) / other.value;
            return FixedPoint(saturate(static_cast<UnderlyingType>(temp)));
        }

        // Assignment operators
        constexpr FixedPoint& operator+=(const FixedPoint& other)
        {
            value = saturate(value + other.value);
            return *this;
        }

        constexpr FixedPoint& operator-=(const FixedPoint& other)
        {
            value = saturate(value - other.value);
            return *this;
        }

        constexpr FixedPoint& operator*=(const FixedPoint& other)
        {
            int64_t temp = static_cast<int64_t>(value) * other.value;
            value = saturate(static_cast<UnderlyingType>(temp >> FractionalBits));
            return *this;
        }

        constexpr FixedPoint& operator/=(const FixedPoint& other)
        {
            int64_t temp = (static_cast<int64_t>(value) << FractionalBits) / other.value;
            value = saturate(static_cast<UnderlyingType>(temp));
            return *this;
        }

        // Comparison operators
        constexpr bool operator==(const FixedPoint& other) const
        {
            return value == other.value;
        }

        constexpr bool operator!=(const FixedPoint& other) const
        {
            return value != other.value;
        }

        constexpr bool operator<(const FixedPoint& other) const
        {
            return value < other.value;
        }

        constexpr bool operator<=(const FixedPoint& other) const
        {
            return value <= other.value;
        }

        constexpr bool operator>(const FixedPoint& other) const
        {
            return value > other.value;
        }

        constexpr bool operator>=(const FixedPoint& other) const {
            return value >= other.value;
        }

        static constexpr UnderlyingType saturate(UnderlyingType rawValue)
        {
            return std::clamp(rawValue, MinValue, MaxValue);
        }

    private:
        
        UnderlyingType value{};       

    };

    // Define Q-format types
    using Q15 = FixedPoint<1, 15>; // q1.15 fixed-point format

    constexpr std::array<Q15, 256> generateSinTable()
    {
        std::array<Q15, 256> table{};
        for (int i = 0; i < 256; ++i)
        {
            table[i] = Q15 (static_cast<float>(std::sin((i * 2.0f * static_cast<float>( std::numbers::pi) / 256.0f)));
        }
        return table;
    }


}

