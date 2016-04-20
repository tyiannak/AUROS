#ifndef MEL_FILTERBANK_HPP
#define MEL_FILTERBANK_HPP

#include <cmath>
#include <utility>
#include <boost/property_tree/ptree.hpp>

#include "processor.hpp"
#include "short_term_feature.hpp"


inline double hz_to_mel(double f)
{
    return 2595.0 * log10(f/700.0 + 1.0);
}

inline double mel_to_hz(double m)
{
    return 700.0*(pow(10.0, m/2595.0) - 1.0);
}

//! @class MelFilterbank
//! @brief A mel-scale, equal-height, 50% overlapped filterbank
//!
//! Used as a Processor, it accepts a sequence representing the DFT
//! of a speech frame and outputs a sequence representing the
//! corresponding mel-spectrum.
class MelFilterbank : public Processor<sequence_vec_t, sequence_vec_t>
{
    //! A mel-filter is defined as a sequence of pairs of
    //! a DFT sequence index and its corresponding weight.
    typedef std::vector<std::pair<unsigned, double>> filter_t;

public:
    MelFilterbank(settings_t & settings);

    bool bang(const sequence_vec_t & input);

    const double f_min;
    const double f_max;
    const unsigned num_filters;

    const std::vector<filter_t> filters;

private:
    std::vector<filter_t> build_filters(settings_t & settings);
};


#endif // MEL_FILTERBANK_HPP
