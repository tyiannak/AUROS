#include "mfcc.hpp"

MFCC::MFCC(settings_t & settings)
    : num_coefs(settings.get<unsigned>("mfcc.num_coefs"))
    , fftw_in_(settings.get<unsigned>("mfcc.mel_filters"))
    , fftw_out_(settings.get<unsigned>("mfcc.mel_filters"))
    , dct_(fftw_plan_r2r_1d(fftw_in_.size(), &fftw_in_[0], &fftw_out_[0], FFTW_REDFT10, FFTW_ESTIMATE ))
{
    output = Eigen::VectorXd::Zero(num_coefs);
}

#include <iostream>
bool MFCC::bang(const sequence_vec_t & input)
{
    std::copy(input.begin(), input.end(), fftw_in_.begin());

    fftw_execute(dct_);

    for (unsigned i=0; i<num_coefs; ++i)
        output(i) = fftw_out_[i];

    return true;
}

MFCC::~MFCC()
{
    fftw_destroy_plan(dct_);
}
