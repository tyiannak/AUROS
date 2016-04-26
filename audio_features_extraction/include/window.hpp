#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <vector>

#include "processor.hpp"
#include "short_term_feature.hpp"

class Window : public Processor<sequence_vec_t, sequence_vec_t>
{
public:
    enum class Type : int
    {
        Rectangle = 0,
        Hamming   = 1,
        Hanning = 2,
        Bartlett = 3,
        Blackman = 4,
        Gaussian = 5
    };

    Window(settings_t & settings);

    bool bang(const sequence_vec_t & input);

    const unsigned size;
    const std::vector<double> coefs;

private:
    std::vector<double> build_win(settings_t & settings);

};

#endif // WINDOW_HPP
