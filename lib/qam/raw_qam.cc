/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_qam.h>
#include <gnuradio/io_signature.h>
#include <limits.h>

raw_qam_enc_sptr
 raw_make_qam_enc (int nbits) {
   return raw_qam_enc_sptr (new raw_qam_enc(nbits));
}

raw_qam_enc::raw_qam_enc (int nbits)
  : gr::sync_decimator ("qam_enc",
                       gr::io_signature::make (1, 1, sizeof(char)),
                       gr::io_signature::make (1, 1, sizeof(gr_complex)),
                       nbits),
    d_nbits(nbits)
{
}

int raw_qam_enc::work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items)
{
  const char *in = (const char *)input_items[0];
  float *out = (float *)output_items[0];

  switch(d_nbits) {
  case 1: { // BPSK
    QAM<1> q(1.0f);
    for (int i = 0; i < noutput_items; ++i) {
      q.encode(in, out); in+= 1; out+= 2;
    }
  } break;
  case 2: { // QPSK
    QAM<1> q(0.5f);
    for (int i = 0; i < noutput_items; ++i) {
      q.encode(in, out); in+= 1; ++out;
      q.encode(in, out); in+= 1; ++out;
    }
  } break;
  case 4: { // QAM16
    QAM<2> q(0.5f);
    for (int i = 0; i < noutput_items; ++i) {
      q.encode(in, out); in+= 2; ++out;
      q.encode(in, out); in+= 2; ++out;
    }
  } break;
  case 6: { // QAM64
    QAM<3> q(0.5f);
    for (int i = 0; i < noutput_items; ++i) {
      q.encode(in, out); in+= 3; ++out;
      q.encode(in, out); in+= 3; ++out;
    }
  } break;
  }
  return noutput_items;
}

raw_qam_dec_sptr
 raw_make_qam_dec (int nbits) {
   return raw_qam_dec_sptr (new raw_qam_dec(nbits));
}

raw_qam_dec::raw_qam_dec (int nbits)
  : gr::sync_interpolator ("qam_dec",
                       gr::io_signature::make (1, 1, sizeof(gr_complex)),
                       gr::io_signature::make (1, 1, sizeof(unsigned char)),
                       nbits),
    d_nbits(nbits)
{
}

int raw_qam_dec::work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items)
{
  const float *in = (const float *)input_items[0];
  unsigned char *out = (unsigned char *)output_items[0];

  switch(d_nbits) {
  case 1: { // BPSK
    QAM<1> q(1.0f);
    for (int i = 0; i < noutput_items; ++i) {
      q.decode(*in, out); in+= 2; out+= 1;
    }
  } break;
  case 2: { // QPSK
    QAM<1> q(0.5f);
    for (int i = 0; i < noutput_items/2; ++i) {
      q.decode(*in, out); in+= 1; out+= 1;
      q.decode(*in, out); in+= 1; out+= 1;
    }
  } break;
  case 4: { // QAM16
    QAM<2> q(0.5f);
    for (int i = 0; i < noutput_items/4; ++i) {
      q.decode(*in, out); in+= 1; out+= 2;
      q.decode(*in, out); in+= 1; out+= 2;
    }
  } break;
  case 6: { // QAM64
    QAM<3> q(0.5f);
    for (int i = 0; i < noutput_items/6; ++i) {
      q.decode(*in, out); in+= 1; out+= 3;
      q.decode(*in, out); in+= 1; out+= 3;
    }
  } break;
  }
  return noutput_items;
}


