#include "window.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
    namespace DiscreteTime
    {
        namespace Filters
        {
            double BaseWindow::w(int i)
            {
                if (i < (_w.size()-1))
                {
                    return _w[i];
                }
                return 0;
            }

            std::unique_ptr<BaseWindow> WindowFactory::create(BaseWindow::Window window, int size, double attenuation)
            {
                switch (window)
                {
                case BaseWindow::Rectangular:
                {
                    return std::make_unique<WindowRectangular>(size);
                }
                case BaseWindow::Chebyshev:
                {
                    return std::make_unique<WindowChebyshev>(size, attenuation);
                }
                case BaseWindow::FlatTop:
                {
                    return std::make_unique<WindowFlatTop>(size);
                }
                case BaseWindow::Hamming:
                {
                    return std::make_unique<WindowHamming>(size);
                }
                case BaseWindow::Hann:
                {
                    return std::make_unique <WindowHann>(size);
                }
                case BaseWindow::Kaiser:
                {
                    return std::make_unique <WindowKaiser>(size, attenuation);
                }
                case BaseWindow::BartlettHann:
                {
                    return std::make_unique<WindowBartlettHann>(size);
                }
                case BaseWindow::Blackman:
                {
                    return std::make_unique<WindowBlackman>(size);
                }
                case BaseWindow::BlackmanHarris:
                {
                    return std::make_unique<WindowBlackmanHarris>(size);
                }
                case BaseWindow::Nuttall:
                {
                    return std::make_unique <WindowNuttall>(size);
                }
                case BaseWindow::BlackmanNuttall:
                {
                    return std::make_unique<WindowBlackmanNuttall>(size);
                }
                case BaseWindow::Parzen:
                {
                    return std::make_unique<WindowParzen>(size);
                }
                case BaseWindow::Triangular:
                {
                    return std::make_unique<WindowTriangular>(size);
                }
                case BaseWindow::Tukey:
                {
                    return std::make_unique<WindowTukey>(size);
                }
                case BaseWindow::Welch:
                {
                    return std::make_unique<WindowWelch>(size);
                }              
                }
                return nullptr;
            }

            WindowRectangular::WindowRectangular(int size) :BaseWindow(size, 0.0)
            {
                for (int i = 0; i < size; i++)
                {
                    _w.push_back(1.0);
                }
            }

            double acosh(double x)
            {
                return 2 * log(sqrt((x + 1) / 2.0) + sqrt((x - 1) / 2.0));
            }

            double beta(int n, double alpha)
            {
                return cosh(acosh(pow(10, alpha)) / (n - 1));
            }

            double T(double n, double x)
            {
                if (fabs(x) <= 1)
                {
                    return cos(n * acos(x));
                }
                else
                {
                    return cosh(n * acosh(x));
                }
            }

            WindowChebyshev::WindowChebyshev(int size, double attenuation) :BaseWindow(size, 0.0)
            {
                attenuation = attenuation < 0 ? -attenuation : attenuation;

                int k(0);
                double a = attenuation / 20;
                int M = size / 2;
                int N = M * 2;
                double b = beta(N, a);

                for (int i = 0; i < size; i++)
                {
                    double sum = 0;
                    for (k = 0; k < M; k++)
                    {
                        sum += (k & 1 ? -1 : 1) * T(N, b * cos(M_PI * k / N)) * cos(2 * i * k * M_PI / N);
                    }
                    sum /= T(N, b);
                    sum -= .5;
                    _w.push_back(sum);
                }
            }

            WindowFlatTop::WindowFlatTop(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (1.000
                        - 1.930 * cos(2.0 * M_PI * (double)i / (double)(size - 1))
                        + 1.290 * cos(4.0 * M_PI * (double)i / (double)(size - 1))
                        - 0.388 * cos(6.0 * M_PI * (double)i / (double)(size - 1))
                        + 0.028 * cos(8.0 * M_PI * (double)i / (double)(size - 1)));
                    _w.push_back(w);
                }
            }

            WindowHamming::WindowHamming(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.54 - 0.46 * cos(2.0 * M_PI * (double)i / (double)(size - 1)));
                    _w.push_back(w);
                }
            }

            WindowHann::WindowHann(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.5 * (1.0 - cos(2.0 * M_PI * (double)i / (double)(size - 1))));
                    _w.push_back(w);
                }
            }

            WindowKaiser::WindowKaiser(int size, double attenuation) :BaseWindow(size, 0.0)
            {
            }

            WindowBartlettHann::WindowBartlettHann(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.62
                        - 0.48 * abs(((double)i / (double)(size - 1)) - 0.5)
                        - 0.38 * cos(2.0 * M_PI * (double)i / (double)(size - 1)));

                    _w.push_back(w);
                }
            }

            WindowBlackman::WindowBlackman(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.42 - 0.5 * cos(2.0 * M_PI * (double)i / (double)(size - 1))
                        + 0.08 * cos(4.0 * M_PI * (double)i / (double)(size - 1)));
                    //qDebug() << w;
                    _w.push_back(w);
                }
            }

            WindowBlackmanHarris::WindowBlackmanHarris(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.358750
                        - 0.488290 * cos(2.0 * M_PI * (double)i / (double)(size - 1))
                        + 0.141280 * cos(4.0 * M_PI * (double)i / (double)(size - 1))
                        - 0.001168 * cos(6.0 * M_PI * (double)i / (double)(size - 1)));
                    //qDebug() << w;
                    _w.push_back(w);
                }
            }

            WindowNuttall::WindowNuttall(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.355768
                        - 0.487396 * cos(2.0 * M_PI * (double)i / (double)(size - 1))
                        + 0.144232 * cos(4.0 * M_PI * (double)i / (double)(size - 1))
                        - 0.012604 * cos(6.0 * M_PI * (double)i / (double)(size - 1)));
                    _w.push_back(w);
                }
            }

            WindowBlackmanNuttall::WindowBlackmanNuttall(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (0.3635819
                        - 0.4891775 * cos(2.0 * M_PI * (double)i / (double)(size - 1))
                        + 0.1365995 * cos(4.0 * M_PI * (double)i / (double)(size - 1))
                        - 0.0106411 * cos(6.0 * M_PI * (double)i / (double)(size - 1)));
                    _w.push_back(w);
                }
            }

            WindowParzen::WindowParzen(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (1.0 - fabs(((double)i - 0.5 * (double)(size - 1)) / (0.5 * (double)(size + 1))));
                       _w.push_back(w);
                }
            }

            WindowTriangular::WindowTriangular(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                double nominator(0.0);
                double denominator(0.0);
                for (int i = 0; i < size; i++)
                {
                    nominator = (double)i - (double)(size - 1) * 0.5;
                    denominator = (double)size * 0.5;
                    w = 1 - abs(nominator / denominator);
                    _w.push_back(w);
                }
            }

            WindowTukey::WindowTukey(int size) :BaseWindow(size, 0.0)
            {

            }

            WindowWelch::WindowWelch(int size) :BaseWindow(size, 0.0)
            {
                double w(0.0);
                for (int i = 0; i < size; i++)
                {
                    w = (1.0 - (((double)i - 0.5 * (double)(size - 1)) / (0.5 * (double)(size + 1)))
                        * (((double)i - 0.5 * (double)(size - 1)) / (0.5 * (double)(size + 1))));
                    _w.push_back(w);
                }
            }
        }
    }
}