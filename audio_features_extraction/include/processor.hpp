#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

#include <boost/property_tree/ptree.hpp>

typedef boost::property_tree::ptree settings_t;

template<class InType, class OutType>
struct Processor
{
    virtual bool bang(const InType & input) = 0;

    OutType output;
};

#endif // PROCESSOR_HPP
