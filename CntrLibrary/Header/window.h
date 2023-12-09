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
#include <cmath>
#include <vector>
#include <memory>

namespace CntrlLibrary
{
    namespace DiscreteTime
    {
        namespace Filters
        {
            class BaseWindow
            {
            public:

                BaseWindow() = delete;

                enum Window
                {
                    Rectangular = 0,
                    FlatTop = 1,
                    Hamming = 2,
                    Hann = 3,
                    BartlettHann = 4,
                    Blackman = 5,
                    BlackmanHarris = 6,
                    Nuttall = 7,
                    BlackmanNuttall = 8,
                    Parzen = 9,
                    Triangular = 10,
                    Tukey = 11,
                    Welch = 12,
                    Lanczos = 13,
                    Chebyshev = 14,
                    Kaiser = 15,
                };

                BaseWindow(int, double) {}

                virtual ~BaseWindow() {}

                inline size_t length()
                {
                    return _w.size();
                }

                double w(int i);

            protected:
                std::vector<std::double_t> _w;

            };

            class WindowFactory
            {
            public:

                static std::unique_ptr<BaseWindow> create(BaseWindow::Window window, int size, double attenuation = 0.0);

            private:

                WindowFactory() {}
            };

            class WindowRectangular : public BaseWindow
            {
            public:
                WindowRectangular(int size);
                virtual ~WindowRectangular() {}
            };

            class WindowChebyshev : public BaseWindow
            {
            public:
                WindowChebyshev(int size, double attenuation);
                virtual ~WindowChebyshev() {}
            };

            class WindowFlatTop : public BaseWindow
            {
            public:
                WindowFlatTop(int size);
                virtual ~WindowFlatTop() {}
            };

            class WindowHamming : public BaseWindow
            {
            public:
                WindowHamming(int size);
                virtual ~WindowHamming() {}
            };

            class WindowHann : public BaseWindow
            {
            public:
                WindowHann(int size);
                virtual ~WindowHann() {}
            };

            class WindowKaiser : public BaseWindow
            {
            public:
                WindowKaiser(int size, double attenuation);
                virtual ~WindowKaiser() {}
            };

            class WindowBartlettHann : public BaseWindow
            {
            public:
                WindowBartlettHann(int size);
                virtual ~WindowBartlettHann() {}
            };

            class WindowBlackman : public BaseWindow
            {
            public:
                WindowBlackman(int size);
                virtual ~WindowBlackman() {}
            };

            class WindowBlackmanHarris : public BaseWindow
            {
            public:
                WindowBlackmanHarris(int size);
                virtual ~WindowBlackmanHarris() {}
            };

            class WindowNuttall : public BaseWindow
            {
            public:
                WindowNuttall(int size);
                virtual ~WindowNuttall() {}
            };

            class WindowBlackmanNuttall : public BaseWindow
            {
            public:
                WindowBlackmanNuttall(int size);
                virtual ~WindowBlackmanNuttall() {}
            };

            class WindowParzen : public BaseWindow
            {
            public:
                WindowParzen(int size);
                virtual ~WindowParzen() {}
            };

            class WindowTriangular : public BaseWindow
            {
            public:
                WindowTriangular(int size);
                virtual ~WindowTriangular() {}
            };

            class WindowTukey : public BaseWindow
            {
            public:
                WindowTukey(int size);
                virtual ~WindowTukey() {}
            };

            class WindowWelch : public BaseWindow
            {
            public:
                WindowWelch(int size);
                virtual ~WindowWelch() {}
            };
        }
    }
}

