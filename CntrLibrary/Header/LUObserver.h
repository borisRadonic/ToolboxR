
/******************************************************************************
The MIT License(MIT)

Derived from ToolboxR Control Library
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
#include <array>
#include <cmath>
#include "FixedPoint.h"
#include "TrigFunctions.h"


namespace CntrlLibrary
{
    constexpr int32_t C6_COMP_CONST1 = 1043038;
    constexpr int32_t C6_COMP_CONST2 = 10430;

    // Helper constants
    constexpr std::size_t SpeedBufferSize = 16;

    template <typename T>
    struct MotorParams
    {
        T R;        // Stator resistance (Ohms)
        T invL;     // 1/Stator inductance (Henry)
        Q15 Ke;     // Back-EMF constant
    };

    template <typename VoltageT, typename CurrentT>
    struct LUStateObserverInputs
    {
        VoltageT Valpha{0.0f}; // Applied voltage alpha
        VoltageT Vbeta{0.0f};  // Applied voltage beta
        CurrentT Ialpha{0.0f}; // Measured current alpha
        CurrentT Ibeta{0.0f};  // Measured current beta
    };

    template <typename VoltageT, typename CurrentT, typename SpeedT>
    struct ObserverState
    {
        VoltageT BemfAlpha{ 0.0f };
        VoltageT BemfBeta{ 0.0f };
        CurrentT IalphaEst{ 0.0f };
        CurrentT IbetaEst{ 0.0f };
        Q15 RotorAngle{ 0.0f }; // Angle in Q1.15 format normalized to the range -p to p radians.
        SpeedT RotorSpeed{ 0.0f };
    };


    template <typename ParamT, typename VoltageT, typename CurrentT, typename SpeedT>
    class LUStateObserver
    {
    public:

        static constexpr CurrentT currTresholdConvPos = CurrentT(0.01f);
        static constexpr CurrentT currTresholdConvNeg = CurrentT(-0.01f);

        LUStateObserver() = default;

        void initialize( const float R,
                         const float invL,
                         const float Ke,
                         const float dcBus,
                         float maxFSpeed,
                         float maxFCurrent,
                         float maxFVoltage,
                         const ParamT samplingPeriod,
                         const ParamT externalHc3,
                         const ParamT externalHc4,
                         const float treshold )
        {
            params.R = R;
            params.invL = invL;
            params.Ke = Ke;
            invDcBusVoltage = 1.0f/dcBus;
            maxSpeed = maxFSpeed;
            maxCurrent = maxFCurrent;
            maxVoltage = maxFVoltage;
            Ts = samplingPeriod;
            hC3 = externalHc3;
            hC4 = externalHc4;
            Fts = static_cast<std::uint32_t>(1.0f/Ts.toFloat());
            Threshold = treshold;
            clearState();
        }

       void update(const LUStateObserverInputs<VoltageT, CurrentT>& inputs)
        {
            // Compute current resistive voltage drops
            VoltageT res1, res2;
            FixedPointOps::convert(FixedPointOps::mul(params.R, state.IalphaEst), res1);
            FixedPointOps::convert(FixedPointOps::mul(params.R, state.IbetaEst), res2);

            // Compute current derivatives
            CurrentT dIalpha, dIbeta;
            FixedPointOps::convert(FixedPointOps::mul((inputs.Valpha - res1 - state.BemfAlpha), params.invL), dIalpha);
            FixedPointOps::convert(FixedPointOps::mul((inputs.Vbeta - res2 - state.BemfBeta), params.invL), dIbeta);

            // Update estimated currents using Euler integration
            CurrentT ialphaEst = state.IalphaEst;
            CurrentT ibetaEst = state.IbetaEst;

            CurrentT deltaIalpha, deltaIbeta;
            FixedPointOps::convert(FixedPointOps::mul(dIalpha, Ts), deltaIalpha);
            FixedPointOps::convert(FixedPointOps::mul(dIbeta, Ts), deltaIbeta);

            ialphaEst += deltaIalpha;
            ibetaEst += deltaIbeta;

            // Clamp estimated currents to prevent overflow
            ialphaEst = std::clamp(ialphaEst, CurrentT(0.0f)-maxCurrent, maxCurrent);
            ibetaEst = std::clamp(ibetaEst, CurrentT(0.0f)-maxCurrent, maxCurrent);

            state.IalphaEst = ialphaEst;
            state.IbetaEst = ibetaEst;

            // Compute back-EMF
            TrigComponents trigComp = TrigFunctions::TrigFuncs(state.RotorAngle);
            VoltageT bemf;
            FixedPointOps::convert(FixedPointOps::mul(params.Ke, state.RotorSpeed), bemf);

            VoltageT bemfAlpha, bemfBeta;
            FixedPointOps::convert(FixedPointOps::mul(bemf, trigComp.hCos), bemfAlpha);
            FixedPointOps::convert(FixedPointOps::mul(bemf, trigComp.hSin), bemfBeta);

            // Clamp back-EMF to prevent overflow
            state.BemfAlpha = std::clamp(bemfAlpha, VoltageT(0.0f)-maxVoltage, maxVoltage);
            state.BemfBeta = std::clamp(bemfBeta, VoltageT(0.0f)-maxVoltage, maxVoltage);

            // Calculate current errors
            CurrentT iAlphaError = state.IalphaEst - inputs.Ialpha;
            CurrentT iBetaError = state.IbetaEst - inputs.Ibeta;

            // Calculate voltage corrections based on errors and observer gains
            VoltageT h3Voltage, h4AlphaVoltage, h4BetaVoltage;
            FixedPointOps::convert(FixedPointOps::mul(hC3, state.RotorSpeed), h3Voltage);
            FixedPointOps::convert(FixedPointOps::mul(hC4, iAlphaError), h4AlphaVoltage);
            FixedPointOps::convert(FixedPointOps::mul(hC4, iBetaError), h4BetaVoltage);


            // Update back-EMF with corrections
            state.BemfAlpha += h4AlphaVoltage + h3Voltage;
            state.BemfBeta += h4BetaVoltage - h3Voltage;

            // Compute rotor angle and speed
            Q15 alpha, beta;
            FixedPointOps::convert(FixedPointOps::mul(invDcBusVoltage, state.BemfAlpha), alpha);
            FixedPointOps::convert(FixedPointOps::mul(invDcBusVoltage, state.BemfBeta), beta);

            state.RotorAngle = calculateAngle(alpha, beta);
            state.RotorSpeed = calculateSpeed(state.RotorAngle);

            // Ensure rotor speed is within physical limits
            state.RotorSpeed = std::clamp(state.RotorSpeed, SpeedT(0.0f)-maxSpeed, maxSpeed);

            // Check convergence of the observer
            updateConvergence(inputs, iAlphaError, iBetaError);
}

        ObserverState<VoltageT, CurrentT, SpeedT> getState() const
        {
            return state;
        }

        [[nodiscard]] bool getIsConverged() const
        {
            return isConverged;
        }

        void clearState()
        {
            state.BemfAlpha = VoltageT(0.0f);
            state.BemfBeta = VoltageT(0.0f);
            state.IalphaEst = CurrentT(0.0f);
            state.IbetaEst = CurrentT(0.0f);
            state.RotorAngle = Q15(0.0f);
            state.RotorSpeed = SpeedT(0.0f);
            bufferIndex = 0;
            consistencyCounter = 0;
            reliabilityCounter = 0;
            speedBuffer.fill(Q15(0.0f));
        }

    private:

        ParamT F1{ 0.0f }, F2{ 0.0f };
        ParamT hC3{ 0.0f }, hC4{ 0.0f };

        ObserverState<VoltageT, CurrentT, SpeedT> state{};

        std::array<Q15, SpeedBufferSize> speedBuffer{0.0f};
        std::size_t bufferIndex{ 0 };

        std::uint8_t consistencyCounter {0};
        std::uint8_t reliabilityCounter {0};

        bool isConverged{false};

        VoltageT Threshold{ 100.00f };

        MotorParams<ParamT> params{};

        Q15 invDcBusVoltage{1.0f};
        
        Q15 Ts{0.0001f};

        std::uint32_t Fts{10000};

        SpeedT maxSpeed{};
        CurrentT maxCurrent{};
        VoltageT maxVoltage{};


        [[nodiscard]] Q15 calculateAngle(const Q15 alpha, const Q15 beta) const
        {
            return TrigFunctions::Atan2(alpha, beta );
        }

        SpeedT calculateSpeed(const Q15 rotorAngle)
        {
            // Retrieve the previous angle from the buffer
            Q15 previousAngle = speedBuffer[bufferIndex];

            // Compute the raw angle difference
            Q15 deltaAngle = rotorAngle - previousAngle;

            // Normalize the angle difference to handle wraparound
            if (deltaAngle > Q15(1.0f))
            {
                deltaAngle -= Q15(2.0f); // Correct for jump from -1 to 1
            }
            else if (deltaAngle < Q15(-1.0f))
            {
                deltaAngle += Q15(2.0f); // Correct for jump from 1 to -1
            }

            // Update the buffer with the current rotor angle
            speedBuffer[bufferIndex] = rotorAngle;

            // Increment and wrap the buffer index
            bufferIndex = (bufferIndex + 1) % SpeedBufferSize;

            SpeedT speed;


            if( deltaAngle < Q15(0.0f))
            {
                Q15 val = Q15(0.0f) - deltaAngle;
                std::int32_t vel = val.raw() * this->Fts;
                speed = SpeedT::fromRaw(vel);
            }
            else
            {
                std::int32_t vel = deltaAngle.raw() * this->Fts;
                speed = SpeedT::fromRaw(vel);
            }

            // Return the calculated angle difference (simplified; scale as needed for speed computation)
            return speed;
        }

        void updateConvergence(const LUStateObserverInputs<VoltageT, CurrentT>& inputs, CurrentT iAlphaError, CurrentT iBetaError)
        {
            // Calculate back-EMF consistency
            VoltageT bemfMagnitude = (state.BemfAlpha * state.BemfAlpha) + (state.BemfBeta * state.BemfBeta);

            bool condition1(false);
            bool condition2(false);
            if( iAlphaError <= CurrentT(0.0f) )
            {
                if( iAlphaError > this->currTresholdConvNeg)
                {
                    condition1 = true;
                }
            }
            else
            {
                if( iAlphaError < this->currTresholdConvPos)
                {
                    condition1 = true;
                }
            }

            if( iBetaError <= CurrentT(0.0f) )
            {
                if( iBetaError > this->currTresholdConvNeg)
                {
                    condition2 = true;
                }
            }
            else
            {
                if( iBetaError < this->currTresholdConvPos)
                {
                    condition2 = true;
                }
            }

            if ( (bemfMagnitude > Threshold) && condition1 && condition2)
            {
                consistencyCounter++;
                if (consistencyCounter >= 10)
                {
                    // Require 10 consistent updates
                    isConverged = true;
                }
            }
            else
            {
                consistencyCounter = 0;
                isConverged = false;
            }
        }
    };
}