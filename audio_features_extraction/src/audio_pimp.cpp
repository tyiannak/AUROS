#include <stdexcept>
#include <iostream>

#include "audio_pimp.hpp"

AudioPimp::AudioPimp(std::string filename, boost::property_tree::ptree & settings)
    : audio_file_  (load_file(filename), &sf_close)
    , filename     (filename)
    , frame_size   (settings.get<double>("frame_duration")*sf_info_.samplerate/1000)
    , overlap_size (settings.get<double>("frame_overlap")*frame_size)
    , samplerate   (sf_info_.samplerate)
    , msec_per_frame ( 1000.0*(frame_size - overlap_size) / samplerate )
    , duration(float(sf_info_.frames) / sf_info_.samplerate)
{}


void AudioPimp::show_info()
{
    std::cout << "#       file: " << filename << "\n";
    std::cout << "#   duration: " << static_cast<double>(sf_info_.frames)/samplerate << " secs.\n";
    std::cout << "# samplerate: " << samplerate << "\n";
    std::cout << "# frame size: " << frame_size*1000.0/samplerate << " msec (" << frame_size << " samples)\n";
    std::cout << "#    overlap: " << overlap_size*100.0/frame_size << "% (" << overlap_size << " samples)\n";
    std::cout << std::endl;
}


bool AudioPimp::get_samples(std::vector<float> & output)
{
    int read_count = sf_read_float(audio_file_.get(), &output[0], frame_size);
    sf_seek(audio_file_.get(), -overlap_size, SEEK_CUR);

    if (read_count != frame_size) {
        sf_seek(audio_file_.get(), 0, SEEK_SET);
        return false;
    }

    return true;
}

SNDFILE* AudioPimp::load_file(std::string filename_)
{
    SNDFILE* s = sf_open(filename_.c_str(), SFM_READ, &sf_info_);
    if (!s) throw std::runtime_error("No such file.");
    return s;
}
