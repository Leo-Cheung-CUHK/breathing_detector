/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 * Copyright 2014 Lizhao You
 */

#ifndef INCLUDED_RAW_PEAK_DETECTOR2_FB_H
#define INCLUDED_RAW_PEAK_DETECTOR2_FB_H

#include <rawofdm_api.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/filter/single_pole_iir.h>

class raw_peak_detector2_fb;
typedef boost::shared_ptr<raw_peak_detector2_fb> raw_peak_detector2_fb_sptr;

RAWOFDM_API raw_peak_detector2_fb_sptr
 raw_make_peak_detector2_fb (float threshold = 0.7);


class RAWOFDM_API raw_peak_detector2_fb : public gr::sync_block
{
  friend RAWOFDM_API raw_peak_detector2_fb_sptr
    raw_make_peak_detector_fb (float threshold);

public:
  raw_peak_detector2_fb (float threshold);

 private:
  void forecast(int noutput_items, gr_vector_int &ninput_items_required);

  float           d_threshold;
  unsigned char   d_state;
  int             d_energyup_sample_cnt;
  int             d_energydn_sample_cnt;
  int             d_plateau;
  bool            d_peak_set;
  int             d_num_auto_peak;

  float           d_peak_val;
  int             d_peak_cnt;

  int             d_peak_outputed;

  size_t          d_pkt_cnt;

  uint64_t        d_pos_last_peak;

  gr::filter::single_pole_iir<double,double,double> d_iir;

 public:
  int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif
