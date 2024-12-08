
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


#ifndef CIRCLE_LIMITATION_H
#define CIRCLE_LIMITATION_H

#include <cstdint>
#include "ParkClarke.h"
#include "TrigFunctions.h"
#include "MathFunctions.h"
#include "FixedPoint.h"

#ifndef HAS_ST32_CORDIC 
#include <cmath>  // For sqrt
#endif

namespace CntrlLibrary
{
    template <typename T>
    class CircleLimitation
    {
        static_assert(std::is_same_v<T, Q15>, "For CircleLimitation, T must be in Q15 format!");

    public:
        // Constructor
        CircleLimitation(T maxModuleValue, T maxVdValue)
            : maxModule(maxModuleValue), maxVd(maxVdValue)
        {
            maxModuleSquared = maxModuleValue.raw() * maxModuleValue.raw();
            maxVdSquared = maxVd.raw() * maxVd.raw();
        }

        // Method to calculate the saturated values
        QD_t<T> calculateSaturation( QD_t<T> vqd) const
        {
            QD_t<T> result = vqd;           
            std::int64_t squareQ = vqd.q.raw() * vqd.q.raw();
            std::int64_t squareD = vqd.d.raw() * vqd.d.raw();
            std::int64_t squareSum = squareQ + squareD;

            if (squareSum > maxModuleSquared)
            {
                if (squareD <= maxVdSquared)
                {
                    typename T::UnderlyingType temp = maxModuleSquared - squareD;
                    T a = T(MathFunctions::sqrt(temp));
                    if (sign(vqd.q) < 0)
                    {
                        result.q = T(0) - a;
                        result.d = vqd.d;
                    }
                    else
                    {
                        result.q = a;
                    }
                   
                }
                else
                {
                    if (sign(vqd.d) < 0)
                    {
                        result.d = T(0) - T(maxVd.raw());
                    }
                    else
                    {
                        result.d = T(maxVd.raw());
                    }
                    std::int32_t temp = maxModuleSquared - maxVdSquared;
                    if (sign(vqd.q) < 0)
                    {
                        result.q = T(0) - T(MathFunctions::sqrt(temp));

                    }
                    else
                    {
                        result.q = T(MathFunctions::sqrt(temp));
                    }
                }
            }
            return result;
        }

        T getMaxModule() const
        {
            return maxModule;
        }

        T getMaxVd() const
        {
            return maxVd;
        }

        void setMaxModule(T maxMdlValue)
        {
            maxModule = maxMdlValue;
        }
        
        void setMaxVd(T maxVdValue)
        {
            maxVd = maxVdValue;
        }

    private:

        T maxModule{};
        std::int64_t maxModuleSquared{};

        T maxVd{};
        std::int64_t maxVdSquared{};

        static int sign(T value)
        {
            return (value.raw() < 0) ? -1 : 1;
        }
    };
}
#endif // CIRCLE_LIMITATION_H
