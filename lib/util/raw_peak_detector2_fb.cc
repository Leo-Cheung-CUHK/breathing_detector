/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_peak_detector2_fb.h>
#include <gnuradio/io_signature.h>
#include <string.h>
#include <stdio.h>
#include <pnc_tags.h>
// lzyou: FIXME: AUTO_CORR_RANGE depends on signal pattern
static const int AUTO_CORR_RANGE = 1.5;

#define DETECTOR_DEBUG  0

enum {
  STATE_NO_SIG,
  STATE_ENERGY_VALID,
  STATE_VALLEY,
  STATE_PLATEAU,
};

#define PWR_THRESHOLD     1e-6
#define SAMPLE_CNT_UP     3
#define SAMPLE_CNT_DW     10
#define AUTO_CORR_VAL_UP  1.2
#define AUTO_CORR_VAL_DW  0.8
#define PLATEAU_CNT       100

#define THRESHOLD_VALLEY     0.5

#define PLATEAU_CNT_MAX       (160+80)


raw_peak_detector2_fb_sptr
raw_make_peak_detector2_fb (float threshold)
{
  return raw_peak_detector2_fb_sptr (new raw_peak_detector2_fb (threshold));
}

raw_peak_detector2_fb::raw_peak_detector2_fb (float threshold)
  : gr::sync_block ("peak_detector2_fb",
                  gr::io_signature::make2 (2, 2, sizeof(gr_complex), sizeof (float)),
                  gr::io_signature::make2 (1, 2, sizeof(char), sizeof(float))),
    d_threshold(threshold), d_iir(0.01)
{
    d_state = STATE_NO_SIG;
    d_energydn_sample_cnt = 0;
    d_energyup_sample_cnt = 0;
    d_plateau = 0;
    d_peak_set = true;

    d_num_auto_peak = 0;

    d_pos_last_peak = 0;

    set_tag_propagation_policy(TPP_DONT);  /* TPP_DONT TPP_ALL_TO_ALL TPP_ONE_TO_ONE */
}

void
raw_peak_detector2_fb::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = noutput_items; // NOTE: so that our sync method is efficient
}

int
raw_peak_detector2_fb::work (int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
{
  gr_complex *iptr = (gr_complex *) input_items[0];
  const float *iauto = (float *) input_items[1];
  char *optr = (char *) output_items[0];
  float *opwr = (output_items.size()>=2) ? ((float *)output_items[1]) : NULL;

  //memset(optr, 0, noutput_items*sizeof(char));

#if DETECTOR_DEBUG
  int port = 0;
  // Use the stream tags to get USRP *INIT* timestamp
  std::vector<gr::tag_t> rx_time_tags;
  const uint64_t nread_n = this->nitems_read(port); //number of items read on port 0

  this->get_tags_in_range(rx_time_tags, port, nread_n, nread_n+noutput_items, TIME_KEY);

  // See if there is a RX timestamp (only on first block or after underrun)
  if(rx_time_tags.size()>0) {
    size_t t = rx_time_tags.size()-1;

    // Take the last timestamp
    const uint64_t sample_offset = rx_time_tags[t].offset;  // distance from sample to timestamp in samples
    const pmt::pmt_t &value = rx_time_tags[t].value;

    // Now, compute the actual time in seconds and fractional seconds of the preamble
    double d_init_hw_frac = pmt::to_double(tuple_ref(value,1));
    uint64_t  d_init_hw_secs = pmt::to_uint64(tuple_ref(value,0));

    printf(" [DETECTOR2] USRP HW init timestamp = (usrp)%.06f offset=%lu\n", d_init_hw_frac+d_init_hw_secs, sample_offset);  
  }
#endif

  int nn = 0;
  while(nn < noutput_items) {
    float pwr = d_iir.filter(std::norm(iptr[nn]));

    if(opwr) {
      opwr[nn] = pwr;
    }

    // default is 0, indicates no pkt
    optr[nn] = 0;

    // if energy is low, force to STATE_NO_SIG immediately
    if(pwr <= PWR_THRESHOLD*2) {
      d_energydn_sample_cnt++;

      if(d_energydn_sample_cnt >= SAMPLE_CNT_DW) {
        d_state = STATE_NO_SIG;
        d_energydn_sample_cnt = 0;
      }
    }
    else {
        d_energydn_sample_cnt = 0;
    }

    // under the condition that energy is valid, going to state machine
    switch(d_state) {
      case STATE_NO_SIG:
        if (pwr > PWR_THRESHOLD*2) {
          d_energyup_sample_cnt++;

          if(d_energyup_sample_cnt == SAMPLE_CNT_UP) {
            d_state = STATE_ENERGY_VALID;
            d_energyup_sample_cnt = 0;
          }
        }
        else {
          d_energyup_sample_cnt = 0;
        }
      break;

     case STATE_ENERGY_VALID:
      if(iauto[nn] < THRESHOLD_VALLEY) {
        d_state = STATE_VALLEY;
      }
      break;

    case STATE_VALLEY:
      if (iauto[nn] > d_threshold && iauto[nn] < AUTO_CORR_VAL_UP) {
        d_state = STATE_PLATEAU;
        d_peak_set = false;
        d_plateau = 0;
      }
      break;

    case STATE_PLATEAU:
      if (iauto[nn] > d_threshold && iauto[nn] < AUTO_CORR_VAL_UP) {
        d_plateau++;

        if (!d_peak_set && d_plateau >= PLATEAU_CNT) {
          d_peak_set = true;
          d_plateau = 0;

          uint64_t npos_cur = nitems_read(0) + nn;
          if(npos_cur - d_pos_last_peak > PLATEAU_CNT_MAX) {
            optr[nn] = 1;

            #if DETECTOR_DEBUG
            printf(" ==================== To STATE CROSS at pos=%lu d_num_auto_peak=%d \n", npos_cur, d_num_auto_peak);
            //printf(" --------------------%lu %lu\n", d_pos_last_peak, npos_cur);
            #endif
          }
          d_pos_last_peak = npos_cur;
          
          d_state = STATE_ENERGY_VALID;
          d_num_auto_peak++;
        }
      }
      else {
        //printf("PLATEAU RESET to 0 at pos=%d \n", nread+i);
        d_plateau = 0;
      }



      break;
    }

    nn++;
  }

  return nn;

}
