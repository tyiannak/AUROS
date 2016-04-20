#include <algorithm>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "window.hpp"

namespace constant = boost::math::double_constants;

Window::Window(settings_t & settings)
    : size(settings.get<unsigned>("frame_size"))
    , coefs(build_win(settings))
{
    output.resize(size);
}

bool Window::bang(const sequence_vec_t & input)
{
    for (unsigned i=0; i<size; ++i)
        output[i] = input[i]*coefs[i];

    return true;
}

std::vector<double> Window::build_win(settings_t & settings)
{
    std::vector<double> w(settings.get<unsigned>("frame_size"));

    switch(Type(settings.get<int>("window_type")))
    {
    case Type::Rectangle:
        std::fill(w.begin(), w.end(), 1.0);
        break;
    case Type::Hamming:
        for (unsigned i=0; i<size; ++i)
            w[i] = 0.54 - 0.46*cos(constant::two_pi*i / (size-1));
        break;
    case Type::Hanning:
        for (unsigned i=0; i<size; ++i)
            w[i] = 0.5*(1.0 - cos(constant::two_pi*(i+1) / (size+1)));
        break;
    default:
        std::fill(w.begin(), w.end(), 1.0);
        break;
    }

    return w;
}
