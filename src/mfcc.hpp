#ifndef MFCC_HPP
#define MFCC_HPP

#include <fftw3.h>

#include "processor.hpp"
#include "short_term_feature.hpp"

//! @class MFCC
//! @brief Computes the Mel-Frequency Cepstral Coefficients of a speech frame.
//!
//! As a Processor, MFCC takes the mel-spectrum as input and outputs the MFCCs.
class MFCC : public Processor<sequence_vec_t,  numeric_vec_t>
{
public:
    MFCC (settings_t & settings);
    ~MFCC();

    bool bang(const sequence_vec_t & input);
    const unsigned num_coefs;

private:
    std::vector<double> fftw_in_;
    std::vector<double> fftw_out_;
    fftw_plan dct_;
};

#endif // MFCC_HPP
