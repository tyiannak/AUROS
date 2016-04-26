#include <algorithm>
#include <cmath>
#include <iostream>

#include <boost/math/constants/constants.hpp>

#include "stft.hpp"

namespace constants = boost::math::double_constants;

STFT::STFT(settings_t & settings)
    : fft_size(settings.get<unsigned>("fft_size"))
    , num_freq_bins(fft_size/2 + 1)
    , freq_bins(num_freq_bins)
    , fftw_in_(fft_size, 0.0)
    , fftw_out_(fft_size)
    , fft_(fftw_plan_r2r_1d(fft_size, &fftw_in_[0], &fftw_out_[0], FFTW_R2HC, FFTW_ESTIMATE))
{
    output.resize(num_freq_bins);
}


void STFT::show_info()
{
    std::cout << "FFT size: " << fft_size << std::endl;
    std::cout << "num. frequency bins:" << num_freq_bins << std::endl;
    std::cout << "[ ";
    for (auto x : freq_bins)
        std::cout << x << " ";
    std::cout << "]\n" << std::endl;
}

bool STFT::bang(const sequence_vec_t & input)
{
    std::copy(input.begin(), input.end(), fftw_in_.begin());
    fftw_execute(fft_);

    output[0] = sqrt(fftw_out_[0]*fftw_out_[0]);
    for (unsigned i=1; i<num_freq_bins; ++i)
        output[i] = sqrt(fftw_out_[i]*fftw_out_[i] + fftw_out_[fft_size-i]*fftw_out_[fft_size-i]);
    if (fft_size % 2 == 0)
        output[fft_size/2] /= constants::root_two;

    return true;
}

STFT::~STFT()
{
    fftw_destroy_plan(fft_);
}
