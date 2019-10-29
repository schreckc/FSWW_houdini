#ifndef INPUTPOINT_HPP
#define INPUTPOINT_HPP

#include <list>
#include <vector>
#include "definitions.hpp"
#include "FFT.hpp"
#include <fstream>

/*
 * 0: Rectangular (no window)
 * 1: Bartlett    (triangular)
 * 2: Hamming
 * 3: Hanning
 * 4: Blackman
 * 5: Blackman-Harris
 * 6: Welch
 * 7: Gaussian(a=2.5)
 * 8: Gaussian(a=3.5)
 * 9: Gaussian(a=4.5)
 */
#define WINDOW_FUNC 9

class InputPoint {
private:
  VEC2 pos;
  
  uint window_size;
  FLOAT dt;
  FLOAT sample_rate;
  uint nb_frequencies;
  FLOAT frequency_step;
  
  std::list<FLOAT> samples;
  std::string name;
  
public:
  FLOAT *spectrum_re;
  FLOAT *spectrum_im;

  std::list<std::vector<FLOAT> > spectrogram;
  std::list<std::vector<FLOAT> > spectrogram_im;
  std::list<std::vector<FLOAT> > spectrogram_re;

  uint t;

  bool rec;
    
public:
  InputPoint() {
    pos = VEC2(0, 0);
    window_size = 0;
    dt = 0;
    nb_frequencies = 0;
    frequency_step = 0;
    
    spectrum_re = NULL;
    spectrum_im = NULL;
    
    t = 0;
    name = "test";
    rec = false;
  }
  InputPoint(uint nb_samples, FLOAT time_step) {
    pos = VEC2(0, 0);
    window_size = nb_samples;
    dt = time_step;
    sample_rate = 1.0/dt;
    frequency_step = 1/(dt*window_size);
    nb_frequencies = window_size;

    //  std::cout<<"freq step "<<frequency_step<<std::endl;
  
    for (uint i  = 0; i < window_size; ++i) {
      samples.push_back(0.0);
    }
    spectrum_re = new FLOAT[nb_frequencies];
    spectrum_im = new FLOAT[nb_frequencies];
    for (uint i = 0; i < nb_frequencies; ++i) {
      spectrum_re[i] = 0;
      spectrum_im[i] = 0;
    }
    t = 0;
    name = "test";
    rec = false;
  }
    
  ~InputPoint() {
     // if (spectrum_re != NULL) {
     //   delete[]spectrum_re;
     //   spectrum_re = NULL;
     // }
     // if (spectrum_im != NULL) {
     //   delete[]spectrum_im;
     //   spectrum_im = NULL;
     // }
    // plotSpectrum();
    // plotSpectrogram();
    // plotSamples();
  }
  
  void setPos(VEC2 p) {
    pos = p;
  }
  void setPos(FLOAT x, FLOAT y) {
    pos = VEC2(x, y);
  }

  VEC2 getPos() const {
    return pos;
  }

  void setName(std::string n) {
    name = n;
  }
  
  void update(FLOAT next_sample) {
     samples.push_front(next_sample);
     //  if (!rec) {
       samples.pop_back();
       // }
    if (t == 1/*window_size / 16*/) {
      computeSpectrum();
      t = 0;
    }
    ++t;
    //   std::cout<<next_sample<<std::endl;   
    
  }
  
  void computeSpectrum()  {
    // std::cout<<"compute spectrum "<<spectrogram.size() <<std::endl;
    FLOAT *in_re = new FLOAT[window_size];
    FLOAT *in_im = new FLOAT[window_size];

    std::list<FLOAT>::iterator it;
    uint i = 0;
    for (it = samples.begin(); /*it != samples.end()*/ i < window_size; ++it, ++i) {
      in_re[i] = (*it);
      in_im[i] = 0.0;
    }
    WindowFunc(WINDOW_FUNC, window_size, in_re);
    FFT(window_size, false, in_re, in_im, spectrum_re, spectrum_im);
    //RealFFT(window_size, in_re, spectrum_re, spectrum_im);
    //FFT(window_size, true, spectrum_re, spectrum_im, in_re, in_im);
  
    std::vector<FLOAT> power_spectrum(nb_frequencies);
    std::vector<FLOAT> spec_re(nb_frequencies);
    std::vector<FLOAT> spec_im(nb_frequencies);
    FLOAT epsilon_db = 1e-18;
    for (uint i = 0; i < nb_frequencies; ++i) {
      FLOAT e = powf(spectrum_re[i]/window_size, 2) + powf(spectrum_im[i]/window_size, 2);
    
      FLOAT e_db = - 10*log10(e+epsilon_db);
      power_spectrum[i] = e;
      spec_re[i] = spectrum_re[i]/window_size;
      spec_im[i] = spectrum_im[i]/window_size;
      // std::cout<<"s "<<i<<" "<<spectrum_re[i]<<" "<<spectrum_im[i]<<std::endl;
      // spectrum_re[i] = in_re[i];
      // spectrum_im[i] = 0;
    }
    spectrogram.push_back(power_spectrum);
    spectrogram_re.push_back(spec_re);
    spectrogram_im.push_back(spec_im);
    if (/*!rec &&*/ spectrogram.size() > 256) {
      // plotSpectrum();
      // plotSpectrogram();
      // plotSamples();
      //    exit(0);
      spectrogram.pop_front();
    }
  
    delete[] in_re;
    delete[] in_im;
  }

  COMPLEX amplitude(FLOAT frequency) const  {
    uint index = frequency/frequency_step; //TODO interpolation ?
    COMPLEX out(spectrum_re[index], spectrum_im[index]);
    return out;
  }

  void plotSpectrum() const {
    std::stringstream ss;
    ss <<name<<"_spectrum.txt";
    std::string str(ss.str());
    //    std::cout<<"Plotting "<<str<<std::endl;
    std::ofstream  out_file;
    out_file.open(str.c_str());

    FLOAT epsilon_db = 1e-18; 
  
    for (uint i = 0; i < nb_frequencies/2; ++i) {
      FLOAT e = powf(spectrum_re[i]/window_size, 2) + powf(spectrum_im[i]/window_size, 2);    
      FLOAT e_db = - 10*log10(e  + epsilon_db);
      out_file << i*frequency_step << " " << e <<"\n";
    }
    out_file.close();

    std::stringstream ss2;
    ss2 <<name<<"_spectrum_imag.txt";
    std::string str2(ss2.str());
    // std::cout<<str<<std::endl;
    std::ofstream  out_file2;
    out_file2.open(str2.c_str());
  
    for (uint i = 0; i < nb_frequencies/2; ++i) {
      out_file2 << i*frequency_step << " " << spectrum_re[i]/window_size <<" "<<spectrum_im[i]/window_size <<"\n";
    }
    out_file2.close();

    
  }

  void plotSpectrogram() const {
    std::stringstream ss;
    ss <<name<<"_spectrogram.txt";
    std::string str(ss.str());
    //std::cout<<"Plotting "<<str<<std::endl;
    std::ofstream  out_file;
    out_file.open(str.c_str());

    std::list<std::vector<FLOAT> >::const_iterator it;
    std::list<std::vector<FLOAT> >::const_iterator it_re = spectrogram_re.begin();
    std::list<std::vector<FLOAT> >::const_iterator it_im = spectrogram_im.begin();
    uint i = 0;
    for (it = spectrogram.begin(); it != spectrogram.end(); ++it, ++i, ++it_re, ++it_im) {
      for (uint j = 0; j < nb_frequencies/2; ++j) {
	out_file << i*dt<<" "<<j*frequency_step << " " << (*it)[j] <<" "<<(*it_re)[j]<<" "<<(*it_im)[j]<<"\n";
      }
      //      std::cout<<i<<" ";
      out_file <<"\n";
    }
    //    std::cout<<" "<<std::endl;
    out_file.close();
  }
  void plotSamples() const {
    std::stringstream ss;
    ss <<name<<"_samples.txt";
    std::string str(ss.str());
    std::ofstream  out_file;
    out_file.open(str.c_str());
  
    std::list<FLOAT>::const_iterator it;
    uint i = 0;
    for (it = samples.begin(); it != samples.end(); ++it, ++i) {
      out_file << i*dt<<" "<<(*it) <<"\n";
    }

    out_file.close();
  }

  void move(VEC2 trans) {
    pos += trans;
  }

  std::ofstream & record_samples(std::ofstream & file) {
    file << samples.back() <<" ";
    return file;
  }

};



#endif




  // //FFT
  // #include "FFT.h"
  //   float *out = new float[size_slide];
  //   float *out2 = new float[size_slide];
  //   float *in = new float[size_slide];
  //   float *in2 = new float[size_slide];

  //    for (int i = 0; i < size_slide; ++i) {
  //     float f = i * freq_step;
  //     double s = /*(1 + 1*sp.speed * exp(0.001*i)) */ sqrt(pow(r_spectrum[i], 2) + pow(i_spectrum[i], 2));
  //     double phase = 2*M_PI*(rand() / (RAND_MAX + 1.0) - 0.5);
  //     in[i] = cos(phase)*s;
  //     in2[i] = sin(phase)*s;
  //     // if (in[i] != 0 || in2[i] != 0) {
  //     //   INFO("freq reel im", f<<" "<<in[i]<<" "<<in2[i]);
  //     // }
  //   }
  //   FFT(size_slide, true, in, in2, out, out2);

