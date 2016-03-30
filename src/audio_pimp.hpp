#ifndef AUDIO_PIMP_HPP
#define AUDIO_PIMP_HPP

#include <memory>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <sndfile.h>


//! @class AudioPimp
//! @brief A libsndfile-based pimp. You ask for a frame, you 've got it!
//!
//! As a Processor, AudioPimp receives an integer representing the audio
//! frame wanted and outputs it as a sequence vector.
class AudioPimp
{
public:
    AudioPimp(std::string filename, boost::property_tree::ptree &settings);

    void show_info();
    bool get_samples(std::vector<float> & target);

private:
    SNDFILE* load_file(std::string filename_);
    SF_INFO sf_info_;
    std::shared_ptr<SNDFILE> audio_file_;

public:
    const std::string filename;
    const int frame_size;
    const int overlap_size;
    const int samplerate;
    const unsigned msec_per_frame;
    const float duration;
};

#endif // AUDIO_PIMP_HPP
