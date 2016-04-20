#include <iostream>
#include <functional>
#include <vector>
#include <memory>
#include <sndfile.hh>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <portaudiocpp/PortAudioCpp.hxx>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/circular_buffer.hpp>
#include "audio_pimp.hpp"
#include "mfcc_extraction.hpp"


#include <deque>
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <ctime>
#include "paexample/featMsg.h"


namespace pa = portaudio;

//////////////////////////////////////////
// AUDIO FEED - RELATED FUNCTIONALITIES //
//////////////////////////////////////////


class RollingStats
{
public:
    RollingStats(std::size_t ndim, std::size_t size)
        : nm1(1.0 / (size - 1.0))
        , nnm1(1.0 / (size * (size - 1.0)))
        , buffer_(size)
        , sum_(Eigen::VectorXd::Zero(ndim))
        , sum_sq_(Eigen::VectorXd::Zero(ndim))
        , aux_(ndim)
    {}

    Eigen::VectorXd mean() { return sum_ * (1.0 / buffer_.size()); }
    Eigen::VectorXd std()  { return (nm1 * sum_sq_ - nnm1 * sum_.cwiseProduct(sum_)).array().sqrt(); }

    std::vector<float> meanf() { auto m = mean(); std::copy(m.data(), m.data() + m.size(), aux_.begin()); return aux_; }
    std::vector<float> stdf() { auto s = std(); std::copy(s.data(), s.data() + s.size(), aux_.begin()); return aux_; }    
    void push(const Eigen::VectorXd & val)
    {        
        for (unsigned int i=0; i<val.size(); i++)
            if (val(i) != val(i))                
            {
                std::cout << val(i) << " " <<  i << std::endl;
                //val(i) = 0;
            }

        sum_ += val;
        sum_sq_ += val.cwiseProduct(val);

        if (buffer_.full()) {
            sum_ -= buffer_[0];
            sum_sq_ -= buffer_[0].cwiseProduct( buffer_[0] );
        }

        buffer_.push_back(val);
    }

    bool ready() { return buffer_.full(); }

    const double nm1, nnm1;

private:
    Eigen::VectorXd sum_, sum_sq_;
    boost::circular_buffer<Eigen::VectorXd> buffer_;
    std::vector<float> aux_;

};






class AudioFeed
{
public:
    AudioFeed(pa::System & instance, int device_id, int frame_size_msec);
    bool fetch(std::vector<float> & output);

    pa::System & instance_;
    pa::Device & device_;

    pa::DirectionSpecificStreamParameters input_params_;
    pa::StreamParameters stream_params_;
    std::unique_ptr<pa::BlockingStream> stream_;
    const int block_size;
};

AudioFeed::AudioFeed(pa::System &instance, int device_id, int frame_size_msec)
    : instance_(instance)
    , device_(instance_.deviceByIndex(device_id))
    , input_params_(device_, 1, pa::FLOAT32, true, device_.defaultHighInputLatency(), nullptr)
    , stream_params_(input_params_, pa::DirectionSpecificStreamParameters::null(), device_.defaultSampleRate(), 0, paClipOff)
    , block_size(frame_size_msec * stream_params_.sampleRate() * input_params_.numChannels() / 1000)
    , stream_(new pa::BlockingStream(stream_params_))
{}


bool AudioFeed::fetch(std::vector<float> & output)
{
    stream_->read(&output[0], block_size);
    return true;
}

void list_input_devices(pa::System & instance)
{
    std::cout << "\n|Available Recording Devices\n";
    std::cout << "*---*---------------------------------------------------*---------*------------*\n";
    std::cout << "| # | Name                                              | Inputs  | Samplerate |\n";
    std::cout << "*---*---------------------------------------------------*---------*------------*\n";

    auto entry_line = boost::format("|%3i| %|-50|| %|8|| %|=11||");
    for (auto device = instance.devicesBegin(); device != instance.devicesEnd(); ++device) {
        if (device->maxInputChannels() < 1)
            continue;
        std::cout << entry_line % device->index() % device->name() % device->maxInputChannels() % device->defaultSampleRate() << "\n";
    }

    std::cout << "*---*---------------------------------------------------*---------*------------*\n";
}

//////////////////////////////
//        FEATURES          //
//////////////////////////////

double energy(std::vector<float> A)
{
    double E = 0.0;
    for (unsigned int i=0; i<A.size(); i++)
        E += ((double)A[i] * (double)A[i]);
    return E / double(A.size());
}

double zcr(std::vector<float> A)
{
    int countZCR = 0;
    for (unsigned int i=1; i<A.size(); i++)
        if ( ( (  A[i]  >0 ) && (  A[i-1]  <0 ) ) || ( ( A[i]  <0 ) && ( A[i-1] >0 ) ) )
            countZCR++;
    return ( double ) countZCR / ( double ) A.size();
}

double energyEntropy(std::vector<float> A, double Energy)
{
    int M = A.size();

    int numOfEnergyEntropyBins = 10;                         // number of energy entropy sub-frames (bins)
    int energyEntropyBinStep = M / numOfEnergyEntropyBins;  // number of samples in each bin
    int countBins = 0;
    int countBinSamples = 0;                                 // (starts from 1)

    std::vector<double> subEnergies ( numOfEnergyEntropyBins );
    subEnergies[0] = 0.0;

    for (unsigned int i=0; i<A.size(); i++) // for each sample:
    {
        countBinSamples++;

        // Compute current sub-energy:
        if ( countBinSamples==energyEntropyBinStep ) // end of bin-->new bin:
        {
            countBinSamples = 0;
            countBins++;
            if (countBins<numOfEnergyEntropyBins)
                subEnergies[countBins] = 0;
        }
        if (countBins<numOfEnergyEntropyBins)
            subEnergies[countBins] += ( A[i] ) * ( A[i] );
    }

    // Initialize energy entropy:
    double H = 0.0;

    // Normalize sub-energies and compute Entropy:
    for ( int i=0; i<numOfEnergyEntropyBins; i++ )
    {
        subEnergies[i] /= double ( M );
        subEnergies[i] /= (Energy+0.000000001);
        H += subEnergies[i] * log2 ( subEnergies[i] + 0.0001 );
    }
    H *= (-1);
    H /= ( double ) numOfEnergyEntropyBins;
    return H;
}

double spectralCentroid(Eigen::VectorXd FFT, int Fs)
{
    double Centroid = 0.0;
    double Sum = 0.0;
    int M = FFT.size();
    for ( int i = 0; i<M; i++ ) // for each fft sample:
    {
        Centroid += FFT[i] * ( double ) i;        // Spectral Centroid Numerator
        Sum += FFT[i];                            // Total spectral energy
    }

    Centroid /= (Sum+0.00000001);                 // Compute spectral centroid

    return Centroid / ((double)Fs / 2.0);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "paexample");
    ros::NodeHandle n;
    bool write_wav, write_features;
    int stWin, ltWin1, ltWin2, window_type, window_param, fft_size, mel_filters, f_min, f_max, num_coefs;
    double frame_overlap, preemphasis_coeff, mel_filter_overlap;
    std::string featuresTopic, exec_mode, wav_filename, features_filename;
    n.param("paexample/mode", exec_mode, std::string("device-input"));
    n.param("paexample/write_wav", write_wav, false);
    n.param("paexample/wav_filename", wav_filename, std::string("output.wav"));
    n.param("paexample/write_features", write_features, false);
    n.param("paexample/features_filename", features_filename, std::string("output_features"));

    n.param("paexample/stWin", stWin, 20);
    n.param("paexample/ltWin1", ltWin1, 50);
    n.param("paexample/ltWin2", ltWin2, 250);

    n.param("paexample/features_topic", featuresTopic, std::string("features_topic"));

    n.param("paexample/frame_overlap", frame_overlap, 0.0);
    n.param("paexample/preemphasis_coeff", preemphasis_coeff, 0.97);
    n.param("paexample/window_type", window_type, 1);
    n.param("paexample/window_param", window_param, -1);
    n.param("paexample/fft_size", fft_size, -1);
    n.param("paexample/mel_filters", mel_filters, 100);
    n.param("paexample/mel_filter_overlap", mel_filter_overlap, 0.5);
    n.param("paexample/f_min", f_min, -1);
    n.param("paexample/f_max", f_max, -1);
    n.param("paexample/num_coefs", num_coefs, 13);
    ros::Publisher pub = n.advertise<paexample::featMsg>(featuresTopic, 1000);

    //Specify loop rate if you want
    //ros::Rate loop_rate(10);

    pa::System::initialize(); // ************ Initialize Portaudio

    settings_t settings;
    settings.put("frame_duration", stWin);
    settings.put("frame_overlap", frame_overlap);
    settings.put("preemphasis_coeff", preemphasis_coeff);
    settings.put("window_type", window_type);
    settings.put("window_param", window_param);
    settings.put("fft_size", fft_size);
    settings.put("mfcc.mel_filters", mel_filters);
    settings.put("mfcc.mel_filter_overlap", mel_filter_overlap);
    settings.put("mfcc.f_min", f_min);
    settings.put("mfcc.f_max", f_max);
    settings.put("mfcc.num_coefs", num_coefs);

    std::cout << "Using " << pa::System::versionText() << "\n";

    if (exec_mode.compare("list-devices") == 0) {
        list_input_devices(pa::System::instance());
        return 1;
    }

    bool use_ltWin2 = false;
    bool reachedMaxSize = false;

    if(ltWin2 == -1){
        use_ltWin2 = false;
    }

    if(stWin % 10 != 0){
        ROS_WARN("stWin is not multiple of 10. Using the default value of 20 instead.");
        stWin = 20;
    }

    if(ltWin1 < 2){
        ROS_WARN("ltWin1 is less than 2. Using the default value of 50 instead.");
        ltWin1 = 50;
    }

    if(ltWin2 < 2 && use_ltWin2){
        ROS_WARN("ltWin2 is less than 2. Using the default value of 250 instead.");
        ltWin2 = 250;
    }

    if(ltWin1 > 10000){
        ROS_WARN("ltWin1 exceeds the value of 10000. Using the default value of 50 instead.");
        ltWin1 = 50;
    }

    if(ltWin2 > 10000){
        ROS_WARN("ltWin2 exceeds the value of 10000. Using the default value of 250 instead.");
        ltWin2 = 250;
    }

    if(ltWin1 >= ltWin2 && use_ltWin2){
        ROS_WARN("ltWin1 is not smaller than ltWin2. Using the default values of 50 and 250 respectively.");
        ltWin1 = 50;
        ltWin2 = 250;
    }

    int Fs;

    // Where does data come from?

    std::function< bool(std::vector<float>&) > get_samples;

    std::unique_ptr<AudioFeed> feed;
    std::unique_ptr<AudioPimp> pimp;

    if (exec_mode.compare("device-input") == 0) {
        //Get default mic ID here
        int default_id = 0;
        for (auto device = pa::System::instance().devicesBegin(); device != pa::System::instance().devicesEnd(); ++device) {
            if (device->maxInputChannels() < 1){
                continue;
            }
            else if(std::string(device->name()).compare("default") == 0){
                default_id = device->index();
            }
        }
        feed.reset(new AudioFeed(pa::System::instance(), default_id, settings.get<int>("frame_duration")));
        feed->stream_->start();
        get_samples = [&](std::vector<float>& out) { return feed->fetch(out); };
        Fs = feed->stream_params_.sampleRate();
    } else if (exec_mode.compare("file-input") == 0) {
        pimp.reset(new AudioPimp(wav_filename, settings));
        pimp->show_info();
        Fs = pimp->samplerate;
        get_samples = [&](std::vector<float>& out) { return pimp->get_samples(out); };
    } else {
        throw std::runtime_error("No input specified. Use one of the list-devices, device-input or file-input options in the parameters.yaml file.");
    }

    settings = reformat_settings(settings, Fs);

    Window win(settings);
    STFT stft(settings);
    MelFilterbank mfb(settings);
    MFCC mfcc(settings);

    std::vector<float> s(win.size);
    std::vector<double> scopy(win.size);

    Eigen::VectorXd featuresAll(4 + mfcc.num_coefs);
    Eigen::VectorXd FFT;

    bool WAVWRITE = false;
    bool FEATUREWRITE = false;

    const int format=SF_FORMAT_WAV | SF_FORMAT_PCM_16;
    const int channels=1;
    SndfileHandle wavFile;
    std::ofstream file;

    if (write_wav) {
        wavFile = SndfileHandle(wav_filename, SFM_WRITE, format, channels, Fs);
        WAVWRITE = true;
    }

    if (write_features) {
        file.open(features_filename);
        FEATUREWRITE = true;
    }

    //std::deque<Eigen::VectorXd> features_collection;
    int currWin = 0;
    ros::Time s_ros = ros::Time::now();


    RollingStats ltWin1_stats(4 + mfcc.num_coefs, ltWin1);
    RollingStats ltWin2_stats(4 + mfcc.num_coefs, ltWin2);
    std::vector<float> featVect(4 + mfcc.num_coefs);
    
    while(ros::ok() && get_samples(s)){
        float max = -100;
        for (unsigned j=0; j<scopy.size(); ++j)
        {
            scopy[j] = s[j];
            if (max < scopy[j])
                max = scopy[j];
        }                                
        //std::cout << max << std::endl ;

        double E = energy(s);
        double Z = zcr(s);
        double EE = energyEntropy(s, E);
        double SC = spectralCentroid(FFT, Fs);
        if (WAVWRITE)
            wavFile.write(s.data(), s.size());

        win.bang(scopy);                                        // apply window
        stft.bang(win.output);                                    // apply fft
        FFT = Eigen::VectorXd::Map(stft.output.data(),stft.output.size());    // fft to eigen array
        FFT = FFT / FFT.size();                                 // FFT normalization
        mfb.bang(stft.output);                                    // mfcc filter bank initialization
        mfcc.bang(mfb.output);                                    // mfcc calculation

        featuresAll << E, Z, EE, SC, mfcc.output;
        ltWin1_stats.push(featuresAll);
        ltWin2_stats.push(featuresAll);

        /*
        std::vector<float> featVect;
        std::vector<float> ltWin1mean, ltWin1deviation, ltWin2mean, ltWin2deviation;

        for (unsigned j=0; j<featuresAll.size(); ++j){
            featVect.push_back(featuresAll[j]);
        }

        if(use_ltWin2){
            if(features_collection.size() == ltWin2){
                features_collection.pop_front();
                reachedMaxSize = true;
            }
            features_collection.push_back(featuresAll);
        }
        else{
            if(features_collection.size() == ltWin1){
                features_collection.pop_front();
                reachedMaxSize = true;
            }
            features_collection.push_back(featuresAll);
        }

        for(int j=0;j<featuresAll.size();j++){
            ltWin1mean.push_back(0);
            ltWin1deviation.push_back(0);
            if(use_ltWin2){
                ltWin2mean.push_back(0);
                ltWin2deviation.push_back(0);
            }
        }

        if(reachedMaxSize){
            if(use_ltWin2){
                for(int i=features_collection.size()-1;i>=0;i--){
                    for(int j=0;j<featuresAll.size();j++){
                        if(features_collection.size() - 1 - ltWin1 >= i){//check if we still need to update for ltWin1
                            ltWin1mean[j] += features_collection[i][j]/features_collection.size();
                        }
                        ltWin2mean[j] += features_collection[i][j]/features_collection.size();
                    }
                }
                for(int i=features_collection.size()-1;i>=0;i--){
                    for(int j=0;j<featuresAll.size();j++){
                        if(features_collection.size() - 1 - ltWin1 >= i){//check if we still need to update for ltWin1
                            ltWin1deviation[j] += pow(features_collection[i][j]-ltWin1mean[j],2)/(features_collection.size()-1);//we first calculate the variance
                        }
                        ltWin2deviation[j] += pow(features_collection[i][j]-ltWin2mean[j],2)/(features_collection.size()-1);//we first calculate the variance
                    }
                }
                for(int i=0;i<ltWin2deviation.size();i++){
                    ltWin2deviation[i] = sqrt(ltWin2deviation[i]);
                }
            }
            else{
                for(int i=0;i<features_collection.size();i++){
                    for(int j=0;j<featuresAll.size();j++){
                        ltWin1mean[j] += features_collection[i][j]/features_collection.size();
                    }
                }
                for(int i=0;i<features_collection.size();i++){
                    for(int j=0;j<featuresAll.size();j++){
                        ltWin1deviation[j] += pow(features_collection[i][j]-ltWin1mean[j],2)/(features_collection.size()-1);//we first calculate the variance
                    }
                }
            }
            //no matter if we have an ltWin2 we will compute this without any difference
            for(int i=0;i<ltWin1deviation.size();i++){
                ltWin1deviation[i] = sqrt(ltWin1deviation[i]);
            }
        }
        */

        paexample::featMsg feat_msg;
        
        //feat_msg.time = ((time_t)clock()/(double)CLOCKS_PER_SEC)*1000;//cpu time in ms
        ros::Time e_ros = ros::Time::now();
        double dur_ros = (e_ros-s_ros).toNSec() * 1e-9;
        feat_msg.time = dur_ros;
        
        std::copy(featuresAll.data(), featuresAll.data() + featuresAll.size(), featVect.begin());
        feat_msg.features = featVect;
        feat_msg.ltWin1mean = ltWin1_stats.meanf();//ltWin1mean;
        feat_msg.ltWin1deviation = ltWin1_stats.stdf(); //ltWin1deviation;
        feat_msg.ltWin2mean = ltWin2_stats.meanf();//ltWin2mean;
        feat_msg.ltWin2deviation = ltWin2_stats.stdf();//ltWin2deviation;

        pub.publish(feat_msg);

        //No need to print anything now
        /*for (unsigned j=0; j<featuresAll.size(); ++j)
            std::cout << featuresAll[j] << "\t";
        std::cout << "\n";*/

        /*for (unsigned j=0; j<featVect.size(); ++j)
            std::cout << featVect[j] << "\t";
        std::cout << "\n";*/

        if (FEATUREWRITE) {
            for (unsigned j=0; j<featuresAll.size(); ++j)
                file << featuresAll[j] << "\t";
            file << "\n";
            file.flush();
        }
    }

    if (FEATUREWRITE)
        file.close();

    pa::System::terminate(); // **************** Close PortAudio

    return 0;
}

