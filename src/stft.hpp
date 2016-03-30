#ifndef STFT_HPP
#define STFT_HPP

#include <fftw3.h>

#include "processor.hpp"
#include "short_term_feature.hpp"

class STFT : public Processor<sequence_vec_t, sequence_vec_t>
{
public:
    STFT(settings_t & settings);
    ~STFT();

    bool bang(const sequence_vec_t & input);
    void show_info();

    const int fft_size;
    const int num_freq_bins;
    const std::vector<double> freq_bins;

private:
    std::vector<double> frequency_scale(unsigned len, double samplerate);

    std::vector<double> fftw_in_;
    std::vector<double> fftw_out_;

    fftw_plan fft_;
};

#endif // STFT_HPP
