#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include "FixedPoint.h"



namespace CntrlLibrary
{
    constexpr int32_t C6_COMP_CONST1 = 1043038;
    constexpr int32_t C6_COMP_CONST2 = 10430;

    // Helper constants
    constexpr std::size_t SpeedBufferSize = 16;

    struct MotorParams
    {
        Q15 R;   // Stator resistance (Ohms)
        Q15 L;   // Stator inductance (Henry)
        Q15 Ke;  // Back-EMF constant
    };

    struct Inputs
    {
        Q15 Valpha; // Applied voltage alpha
        Q15 Vbeta;  // Applied voltage beta
        Q15 Ialpha; // Measured current alpha
        Q15 Ibeta;  // Measured current beta
    };

    struct ObserverState
    {
        Q15 BemfAlpha{ 0.0f };
        Q15 BemfBeta{ 0.0f };
        Q15 IalphaEst{ 0.0f };
        Q15 IbetaEst{ 0.0f };
        Q15 RotorAngle{ 0.0f };
        Q15 RotorSpeed{ 0.0f };
    };

    class LUStateObserver
    {
    public:
        
        LUStateObserver(const MotorParams& motorParams, Q15 samplingPeriod)
            : consistencyCounter(0)
            , reliabilityCounter(0)
            , isConverged(false)
            , params(motorParams)
            , Ts(samplingPeriod)
        {
            speedBuffer.fill(Q15(0.0f));
        }

        void initialize(const Q15& hF1, const Q15& hF2, Q15 minSpeed )
        {
            F1 = hF1;
            F2 = hF2;

            // Initialize gain factors
            hC1 = F1;
            hC2 = Q15(C6_COMP_CONST1 / F2.raw());
            hC3 = F1 * Q15(2.0f);
            hC4 = Q15(C6_COMP_CONST2 / F2.raw());

            speedThreshold = minSpeed;

            clearState();
        }

        void update(const Inputs& inputs)
        {
            // Compute current derivatives
            Q15 dIalpha = (inputs.Valpha - params.R * state.IalphaEst - state.BemfAlpha) / params.L;
            Q15 dIbeta = (inputs.Vbeta - params.R * state.IbetaEst - state.BemfBeta) / params.L;

            // Update estimated currents using Euler integration
            state.IalphaEst += dIalpha * Ts;
            state.IbetaEst += dIbeta * Ts;

            // Compute back-EMF
            // TODO use cordic
            state.BemfAlpha = params.Ke * state.RotorSpeed * Q15(std::cos(state.RotorAngle.toFloat()));
            state.BemfBeta = params.Ke * state.RotorSpeed * Q15(std::sin(state.RotorAngle.toFloat()));

            // Calculate errors
            Q15 iAlphaError = state.IalphaEst - inputs.Ialpha;
            Q15 iBetaError = state.IbetaEst - inputs.Ibeta;

            // Back-EMF estimation
            auto newBemfAlpha = state.BemfAlpha + hC4 * iAlphaError + hC3 * state.RotorSpeed;
            auto newBemfBeta = state.BemfBeta + hC4 * iBetaError - hC3 * state.RotorSpeed;

            // Update state
            state.BemfAlpha = newBemfAlpha;
            state.BemfBeta = newBemfBeta;

            // Compute rotor angle and speed
            state.RotorAngle = calculateAngle(state.BemfAlpha, state.BemfBeta);
            state.RotorSpeed = calculateSpeed(state.RotorAngle);

            // Check convergence
            updateConvergence(inputs, iAlphaError, iBetaError);
        }

        ObserverState getState() const
        {
            return state;
        }

        bool getIsConverged() const
        {
            return isConverged;
        }

    private:

        Q15 F1{ 0.0f }, F2{ 0.0f };
        Q15 hC1{ 0.0f }, hC2{ 0.0f }, hC3{ 0.0f }, hC4{ 0.0f };

        ObserverState state;

        std::array<Q15, SpeedBufferSize> speedBuffer;
        std::size_t bufferIndex{ 0 };

        std::uint8_t consistencyCounter = 0;
        std::uint8_t reliabilityCounter = 0;

        bool isConverged{false};

        Q15 speedThreshold{ 100.00f };


        MotorParams params;
        
        Q15 Ts;




        void clearState()
        {
            state.BemfAlpha = Q15(0.0f);
            state.BemfBeta = Q15(0.0f);
            state.IalphaEst = Q15(0.0f);
            state.IbetaEst = Q15(0.0f);
            state.RotorAngle = Q15(0.0f);
            state.RotorSpeed = Q15(0.0f);

            bufferIndex = 0;
            consistencyCounter = 0;
            reliabilityCounter = 0;
            speedBuffer.fill(Q15(0.0f));
        }

        Q15 calculateAngle(Q15 alpha, Q15 beta) const
        {
            return Q15(0.0f);
            //return Q15(std::atan2(beta.toDouble(), alpha.toDouble()));
        }

        Q15 calculateSpeed(Q15 rotorAngle)
        {
            auto deltaAngle = rotorAngle - speedBuffer[bufferIndex];
            speedBuffer[bufferIndex] = rotorAngle;
            bufferIndex = (bufferIndex + 1) % SpeedBufferSize;
            return deltaAngle; // Simplified, refine as needed
        }


        void updateConvergence(const Inputs& inputs, Q15 iAlphaError, Q15 iBetaError)
        {
            // Calculate back-EMF consistency
            auto bemfMagnitude = state.BemfAlpha * state.BemfAlpha + state.BemfBeta * state.BemfBeta;

            if (bemfMagnitude > speedThreshold && std::abs(iAlphaError.toFloat()) < 0.01 && std::abs(iBetaError.toFloat()) < 0.01)
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

        void normalizeAngle(Q15& angle) const
        {
            while (angle.toFloat() >= 2.0 * static_cast<float>(std::numbers::pi))
            {
                angle -= Q15(2.0f * static_cast<float>(std::numbers::pi));
            }
            while (angle.toFloat() < 0.0)
            {
                angle += Q15( 2.0f * static_cast<float>(std::numbers::pi));
            }
        }

    };
}