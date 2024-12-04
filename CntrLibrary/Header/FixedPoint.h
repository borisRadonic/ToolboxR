
/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2024 Boris Radonic

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
#include <limits>
#include <numbers>
#include <type_traits>

namespace CntrlLibrary
{

    template <typename T>
    class FixedPointUtils
    {
        static_assert(std::is_integral<T>::value, "FixedPointUtils can only be used with integral types.");

    public:
        // Adds two fixed-point values with saturation
      
        static T QADD(T x, T y)
        {
            static_assert(std::is_same<T, int32_t>::value, "QADD only supports 32-bit fixed-point arithmetic.");
            return static_cast<T>(clip<T>(static_cast<int64_t>(x) + static_cast<int64_t>(y), std::numeric_limits<T>::min(), std::numeric_limits<T>::max()));
        }

        static T QSUB( T x, T y)
        {
            static_assert(std::is_same<T, int32_t>::value, "QSUB only supports 32-bit fixed-point arithmetic.");
            return static_cast<T>(clip<T>(static_cast<int64_t>(x) - static_cast<int64_t>(y), std::numeric_limits<T>::min(), std::numeric_limits<T>::max()));
        }


        // Clips a value to the range of the given fixed-point type
        template <typename U>
        static U clip(U x, U minValue, U maxValue)
        {
            return (x > maxValue) ? maxValue : (x < minValue ? minValue : x);
        }

        // Clips from a higher type to the current fixed-point range
        static T clipFromHigherType(int64_t x)
        {
            return clip(x, std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
        }
    };


    using q31_t = int32_t;
    using q15_t = int16_t;
    using q7_t = int8_t;

    // Maximum and minimum values for Q31, Q15, and Q7
    constexpr q31_t Q31_MAX = static_cast<q31_t>(0x7FFFFFFF);
    constexpr q15_t Q15_MAX = static_cast<q15_t>(0x7FFF);
    constexpr q7_t Q7_MAX = static_cast<q7_t>(0x7F);

    constexpr q31_t Q31_MIN = static_cast<q31_t>(0x80000000);
    constexpr q15_t Q15_MIN = static_cast<q15_t>(0x8000);
    constexpr q7_t Q7_MIN = static_cast<q7_t>(0x80);

    // Absolute maximum and minimum values for Q31, Q15, and Q7
    constexpr q31_t Q31_ABSMAX = static_cast<q31_t>(0x7FFFFFFF);
    constexpr q15_t Q15_ABSMAX = static_cast<q15_t>(0x7FFF);
    constexpr q7_t Q7_ABSMAX = static_cast<q7_t>(0x7F);

    constexpr q31_t Q31_ABSMIN = static_cast<q31_t>(0); // Absolute minimum for Q31
    constexpr q15_t Q15_ABSMIN = static_cast<q15_t>(0); // Absolute minimum for Q15
    constexpr q7_t Q7_ABSMIN = static_cast<q7_t>(0);    // Absolute minimum for Q7


    // 8-bit fractional data type in 1.7 format
    using q7_t = int8_t;

    // 16-bit fractional data type in 1.15 format
    using q15_t = int16_t;

    // 32-bit fractional data type in 1.31 format
    using q31_t = int32_t;

    // 64-bit fractional data type in 1.63 format
    using q63_t = int64_t;

    // 32-bit floating-point type
    using float32_t = float;

    // 64-bit floating-point type
    using float64_t = double;


	// Generic clamp function for saturation
	template <typename T>
	constexpr T saturate(T value, T min, T max)
    {
		return std::clamp(value, min, max);
	}

    template <int IntegerBits, int FractionalBits, typename U, typename M>
    class FixedPoint
    {
    
        static_assert(IntegerBits + FractionalBits <= 32, "Only up to 32-bit fixed-point numbers are supported.");
        
        static_assert(IntegerBits > 0 && FractionalBits > 0, "IntegerBits and FractionalBits must be greater than 0.");


    public:

        using UnderlyingType = U; // Use n-bit signed integer for storage
        
        // Maximum and minimum values for this fixed-point type
        static constexpr UnderlyingType MaxValue = (1 << (IntegerBits + FractionalBits - 1)) - 1;
        static constexpr UnderlyingType MinValue = -(1 << (IntegerBits + FractionalBits - 1));

        // Constructors

        constexpr FixedPoint() : value(0)
        {
        }

        constexpr explicit FixedPoint(float fValue)
        {            
            if constexpr (std::is_same_v<U, std::int32_t>)
            {
                constexpr auto ScaleFactor = static_cast<std::uint32_t>(1U << FractionalBits);
                // Clamp the input value to the representable range
                if (fValue > (static_cast<float>(std::numeric_limits<UnderlyingType>::max()) / ScaleFactor))
                {
                    fValue = static_cast<float>(std::numeric_limits<UnderlyingType>::max()) / ScaleFactor;
                }
                else if (fValue < (static_cast<float>(std::numeric_limits<UnderlyingType>::min()) / ScaleFactor))
                {
                    fValue = static_cast<float>(std::numeric_limits<UnderlyingType>::min()) / ScaleFactor;
                }
                value = static_cast<UnderlyingType>(abs(fValue) * ScaleFactor);
                if (fValue < 0.0f)
                {
                    value = value * (-1);
                }
            }
            else if constexpr (std::is_same_v<U, std::int64_t>)
            {
                constexpr std::uint64_t ScaleFactor = 1ULL << FractionalBits;

                // Similar logic as above for 64-bit
                if (fValue > (static_cast<float>(std::numeric_limits<UnderlyingType>::max()) / ScaleFactor))
                {
                    fValue = static_cast<float>(std::numeric_limits<UnderlyingType>::max()) / ScaleFactor;
                }
                else if (fValue < (static_cast<float>(std::numeric_limits<UnderlyingType>::min()) / ScaleFactor))
                {
                    fValue = static_cast<float>(std::numeric_limits<UnderlyingType>::min()) / ScaleFactor;
                }

                value = static_cast<UnderlyingType>(fValue * ScaleFactor);
                if (fValue > 0.0f)
                {
                   // value = value * (-1);
                }
            }
            else
            {
                static_assert(std::is_same_v<U, std::int32_t> || std::is_same_v<U, std::int64_t>, "U must be either std::int32_t or std::int64_t");
            }
        }
               
        constexpr explicit FixedPoint(UnderlyingType raw) : value(raw)
        {
        }

        // Conversion to float
        constexpr float toFloat() const
        {           
            if constexpr (std::is_same_v<U, std::int32_t>)
            {
                float v{ 0.0f };
                constexpr std::uint32_t ScaleFactor = 1U << FractionalBits;
                v = static_cast<float>(value) / ScaleFactor;
                return v;
            }
            else if constexpr (std::is_same_v<U, std::int64_t>)
            {
                float v{ 0.0f };
                constexpr std::uint64_t ScaleFactor = 1ULL << FractionalBits;
                v = static_cast<float>(value) / ScaleFactor;
                return v;

            }
            else
            {
                static_assert(std::is_same_v<U, std::int32_t> || std::is_same_v<U, std::int64_t>, "U must be either std::int32_t or std::int64_t");
            }
            return 0.00;
        }

        // Get raw value
        constexpr UnderlyingType raw() const
        {
            return value;
        }

        constexpr FixedPoint saturateResult() const
        {
            return FixedPoint(saturate(value));
        }

        // Arithmetic operators
        constexpr FixedPoint operator + (const FixedPoint& other) const
        {
            return FixedPoint(value + other.value);
        }

        constexpr FixedPoint operator - (const FixedPoint& other) const
        {
            return FixedPoint(value - other.value);
        }

        constexpr FixedPoint operator * (const FixedPoint& other) const
        {
            M temp = static_cast<M>(value) * other.value;
            return FixedPoint(static_cast<UnderlyingType>(temp >> FractionalBits));
        }

        constexpr FixedPoint operator / (const FixedPoint& other) const
        {
            M temp = (static_cast<M>(value) << FractionalBits) / other.value;
            return FixedPoint(static_cast<UnderlyingType>(temp));
        }

        // Assignment operators
        constexpr FixedPoint& operator += (const FixedPoint& other)
        {
            value = value + other.value;
            return *this;
        }

        constexpr FixedPoint& operator -= (const FixedPoint& other)
        {
            value = value - other.value;
            return *this;
        }

        constexpr FixedPoint& operator *= (const FixedPoint& other)
        {
            M temp = static_cast<M>(value) * other.value;
            value = static_cast<UnderlyingType>(temp >> FractionalBits);
            return *this;
        }

        constexpr FixedPoint& operator/=(const FixedPoint& other)
        {
            M temp = (static_cast<M>(value) << FractionalBits) / other.value;
            value = static_cast<UnderlyingType>(temp);
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

        static constexpr FixedPoint fromRaw(U raw)
        {
            return FixedPoint(raw);
        }

        static constexpr FixedPoint fromFloat(float fValue)
        {
            return FixedPoint(fValue);
        }

        static constexpr float toFloat(U rawValue)
        {
            return static_cast<float>(rawValue) / (1 << FractionalBits);
        }

    private:
        
        UnderlyingType value{};

    };

    namespace FixedPointOps
    {
        template <int IntBitsA, int FracBitsA, typename UA, typename MA,
            int IntBitsB, int FracBitsB, typename UB, typename MB,
            typename Op>
        constexpr auto performOperation(const FixedPoint<IntBitsA, FracBitsA, UA, MA>& a,
            const FixedPoint<IntBitsB, FracBitsB, UB, MB>& b,
            Op operation)
        {
            // Determine common fractional bits
            constexpr int CommonFractionalBits = (FracBitsA > FracBitsB) ? FracBitsA : FracBitsB;
            constexpr int CommonIntegerBits = (IntBitsA > IntBitsB) ? IntBitsA : IntBitsB;

            using CommonUnderlyingType = std::common_type_t<UA, UB>;
            using CommonMultiplierType = std::common_type_t<MA, MB>;

            // Align 'a' to the common fractional bits
            CommonUnderlyingType aAligned = (FracBitsA < CommonFractionalBits)
                ? static_cast<CommonUnderlyingType>(a.raw()) << (CommonFractionalBits - FracBitsA)
                : static_cast<CommonUnderlyingType>(a.raw()) >> (FracBitsA - CommonFractionalBits);

            // Align 'b' to the common fractional bits
            CommonUnderlyingType bAligned = (FracBitsB < CommonFractionalBits)
                ? static_cast<CommonUnderlyingType>(b.raw()) << (CommonFractionalBits - FracBitsB)
                : static_cast<CommonUnderlyingType>(b.raw()) >> (FracBitsB - CommonFractionalBits);

            // Perform the operation
            CommonUnderlyingType resultRaw = operation(aAligned, bAligned);

            // Saturate the result to avoid overflow
            constexpr CommonUnderlyingType MaxValue = (1ULL << (CommonIntegerBits + CommonFractionalBits - 1)) - 1;
            constexpr CommonUnderlyingType MinValue = -(1ULL << (CommonIntegerBits + CommonFractionalBits - 1));

            if (resultRaw > MaxValue) resultRaw = MaxValue;
            if (resultRaw < MinValue) resultRaw = MinValue;

            // Return the result as a FixedPoint with the common fractional bits
            return FixedPoint<CommonIntegerBits, CommonFractionalBits, CommonUnderlyingType, CommonMultiplierType>::fromRaw(resultRaw);
        }

        // Addition function
        template <int IntBitsA, int FracBitsA, typename UA, typename MA,
            int IntBitsB, int FracBitsB, typename UB, typename MB>
        constexpr auto add(const FixedPoint<IntBitsA, FracBitsA, UA, MA>& a,
            const FixedPoint<IntBitsB, FracBitsB, UB, MB>& b)
        {
            return performOperation(a, b, [](auto aAligned, auto bAligned) {
                return aAligned + bAligned;
                });
        }

        // Subtraction function
        template <int IntBitsA, int FracBitsA, typename UA, typename MA,
            int IntBitsB, int FracBitsB, typename UB, typename MB>
        constexpr auto sub(const FixedPoint<IntBitsA, FracBitsA, UA, MA>& a,
            const FixedPoint<IntBitsB, FracBitsB, UB, MB>& b)
        {
            return performOperation(a, b, [](auto aAligned, auto bAligned) {
                return aAligned - bAligned;
                });
        }
    }

    // Define Q-format types
    using Q15 = FixedPoint<1, 15, int32_t, int32_t>; // q1.15 fixed-point format
    using Q31 = FixedPoint<1, 31, int64_t, int64_t>; // q1.31 fixed-point format

    constexpr std::array<Q15, 256> generateSinTable()
    {
        std::array<Q15, 256> table{};
        for (int i = 0; i < 256; ++i)
        {
            float angle = static_cast<float>(i) * (2.0f * std::numbers::pi_v<float> / 256.0f);
            table[i] = Q15(std::sin(angle));
        }
        return table;
    }
}