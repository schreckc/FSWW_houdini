#include "definitions.hpp"
#include "InputPoint.hpp"
#include <iostream>
#include <math.h>

int main(int argc, char **argv) {
  InputPoint ip(128, 0.1);
  for (int i = 0; i < 128; ++i) {
    ip.update(sin(0.05*0.1*i*(2.0*M_PI)));
  }
  ip.plotSpectrum();
  ip.plotSpectrogram();
  ip.plotSamples();
  return 0;
}

