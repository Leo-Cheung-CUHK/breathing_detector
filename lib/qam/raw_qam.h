/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 */

#ifndef INCLUDED_RAW_QAM_H
#define INCLUDED_RAW_QAM_H

#include <rawofdm_api.h>

#include <gnuradio/sync_decimator.h>
#include <gnuradio/sync_interpolator.h>
#include <vector>

class raw_qam_enc;
typedef boost::shared_ptr<raw_qam_enc> raw_qam_enc_sptr;

RAWOFDM_API raw_qam_enc_sptr
 raw_make_qam_enc (int nbits);

/*!
 * \brief Takes bits and converts them to amplitude
 * Uses Gray coding.
 *
 * nbits = number of bits per symbol (1,2,4,6)
 *
 * input[0]: bits
 * output[0]: modulated complex symbols (normalized to unit power)
 */
class raw_qam_enc : public gr::sync_decimator
{
  friend RAWOFDM_API raw_qam_enc_sptr raw_make_qam_enc (int nbits);

  raw_qam_enc (int nbits);

  int d_nbits;

 public:
  int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

class raw_qam_dec;
typedef boost::shared_ptr<raw_qam_dec> raw_qam_dec_sptr;

RAWOFDM_API raw_qam_dec_sptr
 raw_make_qam_dec (int nbits);

/*!
 * \brief Takes modulated symbols and outputs soft values.
 * Uses Gray coding.
 *
 * nbits = number of bits per symbol (1,2,4,6)
 *
 * input[0]: modulated complex symbols (normalized to unit power)
 * output[0]: soft values (confidence, 0..255)
 */
class raw_qam_dec : public gr::sync_interpolator
{
  friend RAWOFDM_API raw_qam_dec_sptr raw_make_qam_dec (int nbits);

  raw_qam_dec (int nbits);

  int d_nbits;

 public:
  int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

// return +1 or -1
static inline int sign(int v) {
  return +1 | (v >> (sizeof(int) * CHAR_BIT - 1));
}

// saturate between 0 and 255
static inline int clamp(int i) {
  return i < 0 ? 0 : (i > 255 ? 255 : i);
}

/**
 * fast QAM (uses at most 4x imull to decode)
 * Tested on 7600 bogomips yields 600-1200Mbps encoding and 300Mbps decoding
 * Compile with -O3
 */
template<int NumBits>
class QAM {
  int d_gain;
  float d_scale_e;
  float d_scale_d;
public:
  /**
   * power -- desired symbol power
   * gain  -- gain on the decoded confidence (power of 2)
   */
  QAM (float power, int gain = 0) {
    d_gain = gain + CHAR_BIT - NumBits;
    const int nn = (1<<(NumBits-1));
    // sum((2k+1)^2,k=0..n-1)
#if 0
    int sum2 = 0;
    for (int i = 0; i < nn; ++i) {
      sum2+= (2*i + 1)*(2*i + 1);
    }
#else
    int sum2 = (4*nn*nn*nn-nn)/3;
#endif
    float sf = sqrt(power * float(nn) / float(sum2));
    d_scale_e = sf;
    d_scale_d = (1 << d_gain) / sf;
  }

  /**
   * We encode recursively to match the decoding process.
   *
   * This could have been implemented by a gray + multiply.
   * gray(i) = (i>>1)^i
   * see: http://www.dspguru.com/dsp/tricks/gray-code-conversion
   */
  inline void encode (const char* bits, float *sym) {
    int pt = 0; // constellation point
    int flip = 1; // +1 or -1 -- for gray coding
    // unrolled with -O3
    for (int i = 0; i < NumBits; ++i) {
      int bit = *bits * 2 - 1; // +1 or -1
      pt = bit * flip + pt * 2;
      flip *= -bit;
      ++bits;
    }
    *sym = pt * d_scale_e;
  }

  /**
   * We decode recursively because we want meaningful confidences.
   * The alternative would be to simply divide and round, which only yields
   * the smallest per-bit confidence.
   *
   * output bit confidence is between 0 and 255
   */
  inline void decode (float sym, unsigned char *bits) {
    int pt = sym * d_scale_d;
    int flip = 1; // +1 or -1 -- for gray coding
    int amp = (1 << (NumBits-1)) << d_gain;
    // unrolled with -O3
    for (int i = 0; i < NumBits; ++i) {
      *bits = clamp(flip * pt + 128);
      int bit = sign(pt);
      pt -= bit * amp;
      flip = -bit;
      amp /= 2;
      ++bits;
    }
  }
};

#endif
