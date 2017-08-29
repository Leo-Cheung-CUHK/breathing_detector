/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_peak_detector_fb.h>
#include <gnuradio/io_signature.h>
#include <string.h>
#include <stdio.h>

// lzyou: FIXME: AUTO_CORR_RANGE depends on signal pattern
static const int AUTO_CORR_RANGE = 1.5;

raw_peak_detector_fb_sptr
raw_make_peak_detector_fb (float threshold_factor_rise,
                    float threshold_factor_fall,
                    int look_ahead, float alpha)
{
  return raw_peak_detector_fb_sptr (new raw_peak_detector_fb (threshold_factor_rise,
                                  threshold_factor_fall,
                                  look_ahead, alpha));
}

raw_peak_detector_fb::raw_peak_detector_fb (float threshold_factor_rise,
                float threshold_factor_fall,
                int look_ahead, float alpha)
  : gr::sync_block ("peak_detector_fb",
                  gr::io_signature::make (1, 1, sizeof (float)),
                  gr::io_signature::make (1, 1, sizeof (char))),
    d_threshold_factor_rise(threshold_factor_rise),
    d_threshold_factor_fall(threshold_factor_fall),
    d_look_ahead(look_ahead), d_avg_alpha(alpha), d_avg(0)
{
}

static const int LOOKAHEAD = 1000;
void
raw_peak_detector_fb::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = LOOKAHEAD; // NOTE: so that our sync method is efficient
}

int
raw_peak_detector_fb::work (int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
{
  float *iptr = (float *) input_items[0];
  char *optr = (char *) output_items[0];

  memset(optr, 0, noutput_items*sizeof(char));

  float peak_val = -(float)INFINITY; // szym: FIX
  int peak_ind = 0;
  unsigned char state = 0; // szym: FIX
  int i = 0;
  bool mark_peak = false;

  float avg = -1.0f;

  const uint64_t nread = this->nitems_read(0);
  while(i < noutput_items) {
    // lzyou: avoid +-NaN only at the beginning
    if( iptr[i] != iptr[i] ) {
      i++;
      continue;
    }

    // lzyou: avoid outputing 1 at the end of a packet
    if(iptr[i] > 0 || iptr[i] < -1*AUTO_CORR_RANGE) {
      i++;
      continue;
    }

    //printf(" |PEAK|DEBUG| state = %d | in[%d] = %f | peak = %d:%f | out_items = %d \n", state, nread+i, iptr[i], peak_ind, peak_val, noutput_items);

    if(state == 0) {  // below threshold
      if(iptr[i] > avg*d_threshold_factor_rise) {
        state = 1;
      } else {
        i++;
      }
    } else if(state == 1) {  // above threshold, have not found peak
      if(iptr[i] > peak_val) {
        peak_val = iptr[i];
        peak_ind = i;
        i++;
      } else if (iptr[i] > avg*d_threshold_factor_fall) {
        i++;
      } else {
        //printf(" |PEAK|DEBUG| state = %d | i=%d | in[%d] = %f | peak = %d:%f | out_items = %d \n", state, i, nread+i, iptr[i], peak_ind, peak_val, noutput_items);
        mark_peak = true;
        optr[peak_ind] = 1;
        state = 0;
        peak_val = -(float)INFINITY;
      }
    }
  }

  if(noutput_items < LOOKAHEAD)
    printf(">>> FATAL ERROR | Please check raw_peak_detector_fb block \n");

  // lzyou: ignore the "struck" point caused by GNURadio scheduling
  // the struck point means it can not reach the low_threshold, but it is larger than high_threshold
  // ie., it can not be the peak we want to find given than ninput > 4 ofdm symbols
  if( (peak_ind==0) && !mark_peak && noutput_items > 80*4) {
    //printf(" |WARNING| No Fall | s=%d | in[%d] = %f | peak = %d:%f | nin=nout=%d \n",
    //  state, nread+i, iptr[i], peak_ind, peak_val, noutput_items);
    return noutput_items;
  }

  //if(state==1)
  //  printf(" |PEAK|DEBUG| state = %d | in[%d] = %f | peak = %d:%f | out_items = %d:%d | d_count = %d:%d \n",
  //    state, nread+i, iptr[i], peak_ind, peak_val, i, noutput_items, peak_ind, d_count);

  if(state == 0) {
    return noutput_items;
  } else {   // only return up to passing the threshold
    return peak_ind; // NOTE: peak itself must remain unconsumed
  }
}
