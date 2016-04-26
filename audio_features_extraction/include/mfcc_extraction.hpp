#ifndef DIARX_HPP
#define DIARX_HPP

#include "audio_pimp.hpp"
#include "mel_filterbank.hpp"
#include "mfcc.hpp"
#include "stft.hpp"
#include "window.hpp"

inline
settings_t reformat_settings(settings_t & settings, int samplerate)
{
    settings_t my_settings = settings;

    int frame_size = settings.get<int>("frame_duration")*samplerate/1000.0;
    if (frame_size <= 0)
        throw std::runtime_error("[Error] frame duration must be positive.\n");

    int fft_size = settings.get<int>("fft_size");
    if (fft_size == -1)
        fft_size = pow(2, ceil(log(frame_size)/log(2)));
    else if (fft_size == 0)
        fft_size =  frame_size;

    if (fft_size < frame_size)
        throw std::runtime_error("[Error] FFT size must be >= frame size.\n");

    double f_min = settings.get<double>("mfcc.f_min");
    f_min = f_min < 0.0 ? 0.0 : f_min;
    double f_max = settings.get<double>("mfcc.f_max");
    f_max = f_max < 0.0 ? samplerate/2.0 : f_max;

    if (f_min < 0.0 || f_max < 0.0 || f_min > samplerate/2.0 || f_max > samplerate/2.0)
        throw std::runtime_error("[Error] f_min, f_max must be in range [0, samplerate/2].\n");
    if (f_min >= f_max)
        throw std::runtime_error("[Error] f_min must be < f_max.\n");

    double overlap = settings.get<double>("frame_overlap");
    if (overlap < 0.0 || overlap >= 1.0)
        throw std::runtime_error("[Error] frame overlap must be in [0, 1).\n");

    // duration exclusive to each frame
//    double frame_time_span = (1.0-overlap)*settings.get<double>("frame_duration");

    // convert texture window durations to number of short-time frames
//    unsigned texture_win_size = settings.get<unsigned>("flsd.texture_window_size");
//    unsigned texture_win_step = settings.get<unsigned>("flsd.texture_window_step");

//    texture_win_size = ceil(texture_win_size/frame_time_span);
//    texture_win_step = ceil(texture_win_step/frame_time_span);




    my_settings.put("samplerate", samplerate);
    my_settings.put("frame_size", frame_size);
    my_settings.put("fft_size", fft_size);
    my_settings.put("mfcc.f_min", f_min);
    my_settings.put("mfcc.f_max", f_max);
//    my_settings.put("flsd.texture_window_size", texture_win_size);
//    my_settings.put("flsd.texture_window_step", texture_win_step);

    return my_settings;
}

#endif // DIARX_HPP
