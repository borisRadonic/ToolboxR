
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

#include "FixedPoint.h"

#include <cstdint>
#include <cstdint>
#include <cmath>
#include <numbers>

namespace CntrlLibrary
{
    
    class MathFunctions
    {
        static std::int32_t sqrt(int32_t input)
        {
            if (input > 0u)
            {
#ifdef HAS_ST32_CORDIC
                uint32_t retVal{};
                /* Disable Irq*/
                __disable_irq();
                /* Configure CORDIC */
                WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SQRT);
                LL_CORDIC_WriteData(CORDIC, ((uint32_t)input));
                /* Read sqrt and return */
                retVal = (LL_CORDIC_ReadData(CORDIC)) >> 15U;
                retVal = (LL_CORDIC_ReadData(CORDIC)) / 32768U;
                wtemprootnew = static_cast<int32_t>(retVal);
                __enable_irq();

#else              
                return static_cast<float>( sqrt( static_cast<float>(input)) );
#endif
            }
            return 0u;
           
            

        }
    };
}
