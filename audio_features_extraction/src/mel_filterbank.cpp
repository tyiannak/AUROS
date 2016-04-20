#include "mel_filterbank.hpp"

MelFilterbank::MelFilterbank(settings_t &settings)
    : f_min(settings.get<double>("mfcc.f_min"))
    , f_max(settings.get<double>("mfcc.f_max"))
    , num_filters(settings.get<unsigned>("mfcc.mel_filters"))
    , filters(build_filters(settings))
{
    output.resize(num_filters);
}

bool MelFilterbank::bang(const sequence_vec_t & input)
{
    for (unsigned i=0; i<num_filters; ++i)
    {
        output[i] = 0.0;
        for (auto elem : filters[i])
            output[i] += input[elem.first]*elem.second;
        output[i] = log10(output[i] + 1e-20);
    }

    return true;
}

std::vector<MelFilterbank::filter_t> MelFilterbank::build_filters(settings_t & settings)
{
    std::vector<filter_t> result(num_filters);

    double samplerate = settings.get<double>("samplerate");
    double frame_size = settings.get<double>("frame_size");
    unsigned fft_size = settings.get<unsigned>("fft_size");

    std::vector<double> hz_bins(fft_size/2 + 1);
    for (unsigned i=0; i<hz_bins.size(); ++i)
        hz_bins[i] = i * samplerate/fft_size;

    double m_min = hz_to_mel(f_min);
    double m_max = hz_to_mel(f_max);

    double overlap = settings.get<double>("mfcc.mel_filter_overlap");

    double mel_length = (m_max - m_min)/(1.0 + (num_filters-1)*(1.0-overlap));
    double mel_step   = (1.0-overlap)*mel_length;

    for (unsigned i=0; i<num_filters; ++i)
    {
        double mel_low  = m_min + i*mel_step;
        double mel_high = mel_low + mel_length;
        double mel_mid  = 0.5*(mel_low + mel_high);

        unsigned hz_min_index =
                std::distance(hz_bins.begin(), std::find_if(hz_bins.begin(), hz_bins.end(),
                                [&](double f){ return hz_to_mel(f) >= mel_low; }));

        unsigned hz_max_index =
                std::distance(hz_bins.begin(), std::find_if(hz_bins.begin(), hz_bins.end(),
                                [&](double f){ return hz_to_mel(f) > mel_high; }));

        for (unsigned j=hz_min_index; j<hz_max_index; ++j)
        {
            double w = 1.0 - 2.0*fabs(hz_to_mel(hz_bins[j]) - mel_mid) / (mel_high - mel_low);
            result[i].push_back(std::make_pair(j, w));
        }
    }

    return result;
}
