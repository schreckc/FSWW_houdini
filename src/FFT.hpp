/**********************************************************************

  FFT.h

  Dominic Mazzoni

  September 2000

  This file contains a few FFT routines, including a real-FFT
  routine that is almost twice as fast as a normal complex FFT,
  and a power spectrum routine which is more convenient when
  you know you don't care about phase information.  It now also
  contains a few basic windowing functions.

  Some of this code was based on a free implementation of an FFT
  by Don Cross, available on the web at:

    http://www.intersrv.com/~dcross/fft.html

  The basic algorithm for his code was based on Numerical Recipes
  in Fortran.  I optimized his code further by reducing array
  accesses, caching the bit reversal table, and eliminating
  float-to-double conversions, and I added the routines to
  calculate a real FFT and a real power spectrum.

  Note: all of these routines use single-precision floats.
  I have found that in practice, floats work well until you
  get above 8192 samples.  If you need to do a larger FFT,
  you need to use doubles.

**********************************************************************/
/*
  Salvo Ventura - November 2006
  Added more window functions:
    * 4: Blackman
    * 5: Blackman-Harris
    * 6: Welch
    * 7: Gaussian(a=2.5)
    * 8: Gaussian(a=3.5)
    * 9: Gaussian(a=4.5)
*/

#ifndef M_PI
#define	M_PI		3.14159265358979323846  /* pi */
#endif

static int **gFFTBitTable = NULL;
static const int MaxFastBits = 16;

/* Declare Static functions */
static int IsPowerOfTwo(int x);
static int NumberOfBitsNeeded(int PowerOfTwo);
static int ReverseBits(int index, int NumBits);
static void InitFFT();

int IsPowerOfTwo(int x)
{
   if (x < 2)
      return false;

   if (x & (x - 1))             /* Thanks to 'byang' for this cute trick! */
      return false;

   return true;
}

int NumberOfBitsNeeded(int PowerOfTwo)
{
   int i;

   if (PowerOfTwo < 2) {
      fprintf(stderr, "Error: FFT called with size %d\n", PowerOfTwo);
      exit(1);
   }

   for (i = 0;; i++)
      if (PowerOfTwo & (1 << i))
         return i;
}

int ReverseBits(int index, int NumBits)
{
   int i, rev;

   for (i = rev = 0; i < NumBits; i++) {
      rev = (rev << 1) | (index & 1);
      index >>= 1;
   }

   return rev;
}

void InitFFT()
{
   gFFTBitTable = new int *[MaxFastBits];

   int len = 2;
   for (int b = 1; b <= MaxFastBits; b++) {

      gFFTBitTable[b - 1] = new int[len];

      for (int i = 0; i < len; i++)
         gFFTBitTable[b - 1][i] = ReverseBits(i, b);

      len <<= 1;
   }
}

inline int FastReverseBits(int i, int NumBits)
{
   if (NumBits <= MaxFastBits)
      return gFFTBitTable[NumBits - 1][i];
   else
      return ReverseBits(i, NumBits);
}

/*
 * Computes a FFT of complex input and returns complex output.
 * Currently this is the only function here that supports the
 * inverse transform as well.
 */

void FFT(int NumSamples,
         bool InverseTransform,
         float *RealIn, float *ImagIn, float *RealOut, float *ImagOut) {

   int NumBits;                 /* Number of bits needed to store indices */
   int i, j, k, n;
   int BlockSize, BlockEnd;

   double angle_numerator = 2.0 * M_PI;
   double tr, ti;                /* temp real, temp imaginary */

   if (!IsPowerOfTwo(NumSamples)) {
      fprintf(stderr, "%d is not a power of two\n", NumSamples);
      std::exit(1);
   }

   if (!gFFTBitTable)
      InitFFT();

   if (!InverseTransform)
      angle_numerator = -angle_numerator;

   NumBits = NumberOfBitsNeeded(NumSamples);

   /*
    **   Do simultaneous data copy and bit-reversal ordering into outputs...
    */

   for (i = 0; i < NumSamples; i++) {
      j = FastReverseBits(i, NumBits);
      RealOut[j] = RealIn[i];
      ImagOut[j] = (ImagIn == NULL) ? 0.0 : ImagIn[i];
   }

   /*
    **   Do the FFT itself...
    */

   BlockEnd = 1;
   for (BlockSize = 2; BlockSize <= NumSamples; BlockSize <<= 1) {

      double delta_angle = angle_numerator / (double) BlockSize;

      double sm2 = sin(-2 * delta_angle);
      double sm1 = sin(-delta_angle);
      double cm2 = cos(-2 * delta_angle);
      double cm1 = cos(-delta_angle);
      double w = 2 * cm1;
      double ar0, ar1, ar2, ai0, ai1, ai2;

      for (i = 0; i < NumSamples; i += BlockSize) {
         ar2 = cm2;
         ar1 = cm1;

         ai2 = sm2;
         ai1 = sm1;

         for (j = i, n = 0; n < BlockEnd; j++, n++) {
            ar0 = w * ar1 - ar2;
            ar2 = ar1;
            ar1 = ar0;

            ai0 = w * ai1 - ai2;
            ai2 = ai1;
            ai1 = ai0;

            k = j + BlockEnd;
            tr = ar0 * RealOut[k] - ai0 * ImagOut[k];
            ti = ar0 * ImagOut[k] + ai0 * RealOut[k];

            RealOut[k] = RealOut[j] - tr;
            ImagOut[k] = ImagOut[j] - ti;

            RealOut[j] += tr;
            ImagOut[j] += ti;
         }
      }

      BlockEnd = BlockSize;
   }

   /*
      **   Need to normalize if inverse transform...
    */

   if (InverseTransform) {
      float denom = (float) NumSamples;

      for (i = 0; i < NumSamples; i++) {
         RealOut[i] /= denom;
         ImagOut[i] /= denom;
      }
   }

 }




/*
 * This is the function you will use the most often.
 * Given an array of floats, this will compute the power
 * spectrum by doing a Real FFT and then computing the
 * sum of the squares of the real and imaginary parts.
 * Note that the output array is half the length of the
 * input array, and that NumSamples must be a power of two.
 */

// void PowerSpectrum(int NumSamples, float *In, float *Out) {


// }

/*
 * Computes an FFT when the input data is real but you still
 * want complex data as output.  The output arrays are the
 * same length as the input, but will be conjugate-symmetric
 * NumSamples must be a power of two.
 */

void RealFFT(int NumSamples,
             float *RealIn, float *RealOut, float *ImagOut) {
#ifdef EXPERIMENTAL_USE_REALFFTF
   // Remap to RealFFTf() function
   int i;
   HFFT hFFT = GetFFT(NumSamples);
   float *pFFT = new float[NumSamples];
   // Copy the data into the processing buffer
   for(i=0; i<NumSamples; i++)
      pFFT[i] = RealIn[i];

   // Perform the FFT
   RealFFTf(pFFT, hFFT);

   // Copy the data into the real and imaginary outputs
   for(i=1;i<(NumSamples/2);i++) {
      RealOut[i]=pFFT[hFFT->BitReversed[i]  ];
      ImagOut[i]=pFFT[hFFT->BitReversed[i]+1];
   }
   // Handle the (real-only) DC and Fs/2 bins
   RealOut[0] = pFFT[0];
   RealOut[i] = pFFT[1];
   ImagOut[0] = ImagOut[i] = 0;
   // Fill in the upper half using symmetry properties
   for(i++ ; i<NumSamples; i++) {
      RealOut[i] =  RealOut[NumSamples-i];
      ImagOut[i] = -ImagOut[NumSamples-i];
   }
   delete [] pFFT;
   ReleaseFFT(hFFT);

#else

   int Half = NumSamples / 2;
   int i;

   float theta = M_PI / Half;

   float *tmpReal = new float[Half];
   float *tmpImag = new float[Half];

   for (i = 0; i < Half; i++) {
      tmpReal[i] = RealIn[2 * i];
      tmpImag[i] = RealIn[2 * i + 1];
   }

   FFT(Half, 0, tmpReal, tmpImag, RealOut, ImagOut);

   float wtemp = float (sin(0.5 * theta));

   float wpr = -2.0 * wtemp * wtemp;
   float wpi = -1.0 * float (sin(theta));
   float wr = 1.0 + wpr;
   float wi = wpi;

   int i3;

   float h1r, h1i, h2r, h2i;

   for (i = 1; i < Half / 2; i++) {

      i3 = Half - i;

      h1r = 0.5 * (RealOut[i] + RealOut[i3]);
      h1i = 0.5 * (ImagOut[i] - ImagOut[i3]);
      h2r = 0.5 * (ImagOut[i] + ImagOut[i3]);
      h2i = -0.5 * (RealOut[i] - RealOut[i3]);

      RealOut[i] = h1r + wr * h2r - wi * h2i;
      ImagOut[i] = h1i + wr * h2i + wi * h2r;
      RealOut[i3] = h1r - wr * h2r + wi * h2i;
      ImagOut[i3] = -h1i + wr * h2i + wi * h2r;

      wr = (wtemp = wr) * wpr - wi * wpi + wr;
      wi = wi * wpr + wtemp * wpi + wi;
   }

   RealOut[0] = (h1r = RealOut[0]) + ImagOut[0];
   ImagOut[0] = h1r - ImagOut[0];

   delete[]tmpReal;
   delete[]tmpImag;
#endif //EXPERIMENTAL_USE_REALFFTF
}
  

/*
 * Computes an Inverse FFT when the input data is conjugate symmetric
 * so the output is purely real.  NumSamples must be a power of
 * two.
 * Requires: EXPERIMENTAL_USE_REALFFTF
 */
/* #include "Experimental.h" */
/* #ifdef EXPERIMENTAL_USE_REALFFTF */
/* void InverseRealFFT(int NumSamples, */
/*              float *RealIn, float *ImagIn, float *RealOut); */
/* #endif */



/*
 * Applies a windowing function to the data in place
 *
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

void WindowFunc(int whichFunction, int NumSamples, float *in) {
   int i;
   double A;

   switch( whichFunction )
   {
   case 1:
      // Bartlett (triangular) window
      for (i = 0; i < NumSamples / 2; i++) {
         in[i] *= (i / (float) (NumSamples / 2));
         in[i + (NumSamples / 2)] *=
             (1.0 - (i / (float) (NumSamples / 2)));
      }
      break;
   case 2:
      // Hamming
      for (i = 0; i < NumSamples; i++)
         in[i] *= 0.54 - 0.46 * cos(2 * M_PI * i / (NumSamples - 1));
      break;
   case 3:
      // Hanning
      for (i = 0; i < NumSamples; i++)
         in[i] *= 0.50 - 0.50 * cos(2 * M_PI * i / (NumSamples - 1));
      break;
   case 4:
      // Blackman
      for (i = 0; i < NumSamples; i++) {
          in[i] *= 0.42 - 0.5 * cos (2 * M_PI * i / (NumSamples - 1)) + 0.08 * cos (4 * M_PI * i / (NumSamples - 1));
      }
      break;
   case 5:
      // Blackman-Harris
      for (i = 0; i < NumSamples; i++) {
          in[i] *= 0.35875 - 0.48829 * cos(2 * M_PI * i /(NumSamples-1)) + 0.14128 * cos(4 * M_PI * i/(NumSamples-1)) - 0.01168 * cos(6 * M_PI * i/(NumSamples-1));
      }
      break;
   case 6:
      // Welch
      for (i = 0; i < NumSamples; i++) {
          in[i] *= 4*i/(float)NumSamples*(1-(i/(float)NumSamples));
      }
      break;
   case 7:
      // Gaussian (a=2.5)
      // Precalculate some values, and simplify the fmla to try and reduce overhead
      A=-2*2.5*2.5;

      for (i = 0; i < NumSamples; i++) {
          // full
          // in[i] *= exp(-0.5*(A*((i-NumSamples/2)/NumSamples/2))*(A*((i-NumSamples/2)/NumSamples/2)));
          // reduced
          in[i] *= exp(A*(0.25 + ((i/(float)NumSamples)*(i/(float)NumSamples)) - (i/(float)NumSamples)));
      }
      break;
   case 8:
      // Gaussian (a=3.5)
      A=-2*3.5*3.5;
      for (i = 0; i < NumSamples; i++) {
          // reduced
          in[i] *= exp(A*(0.25 + ((i/(float)NumSamples)*(i/(float)NumSamples)) - (i/(float)NumSamples)));
      }
      break;
   case 9:
      // Gaussian (a=4.5)
      A=-2*4.5*4.5;

      for (i = 0; i < NumSamples; i++) {
          // reduced
          in[i] *= exp(A*(0.25 + ((i/(float)NumSamples)*(i/(float)NumSamples)) - (i/(float)NumSamples)));
      }
      break;
   default:
      fprintf(stderr,"FFT::WindowFunc - Invalid window function: %d\n",whichFunction);
   }
}

/*
 * Returns the name of the windowing function (for UI display)
 */

//const wxChar *WindowFuncName(int whichFunction);

/*
 * Returns the number of windowing functions supported
 */

int NumWindowFuncs() {
   return 10;
}

//void DeinitFFT();
