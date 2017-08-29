/* -*- c++ -*- */

%include "gnuradio.i"                  // the common stuff

%{
#include "raw_peak_detector_fb.h"
#include "raw_peak_detector2_fb.h"
#include "raw_message_sink.h"
#include "raw_message.h"
#include "raw_msg_queue.h"
#include "raw_message_source.h"
#include "raw_regenerate_peak2.h"
#include "raw_regenerate_peak3.h"
#include "raw_pnc_frequency_modulator_fc.h"
#include "raw_divide_ff.h"
%}

%include "raw_message.i"
%include "raw_msg_queue.i"

GR_SWIG_BLOCK_MAGIC(raw,peak_detector_fb)
raw_peak_detector_fb_sptr raw_make_peak_detector_fb (float threshold_factor_rise = 0.25,
                                                     float threshold_factor_fall = 0.40,
                                                     int look_ahead = 10,
                                                     float alpha=0.001);
class raw_peak_detector_fb : public gr::sync_block {};

GR_SWIG_BLOCK_MAGIC(raw,peak_detector2_fb)
raw_peak_detector2_fb_sptr raw_make_peak_detector2_fb (float threshold=0.7);
class raw_peak_detector2_fb : public gr::sync_block {};

GR_SWIG_BLOCK_MAGIC(raw,message_sink);
raw_message_sink_sptr raw_make_message_sink (size_t itemsize, size_t num_symbol,
                                             raw_msg_queue_sptr msgq,
                                             bool dont_block);
class raw_message_sink : public gr::sync_block {};

GR_SWIG_BLOCK_MAGIC(raw,message_source);
raw_message_source_sptr raw_make_message_source (size_t itemsize, int msgq_limit=0, bool flag=false);
raw_message_source_sptr raw_make_message_source (size_t itemsize, raw_msg_queue_sptr msgq);
class raw_message_source : public gr::sync_block {
 public:
  raw_msg_queue_sptr msgq() const { return d_msgq; }
};

GR_SWIG_BLOCK_MAGIC(raw,regenerate_peak2)
raw_regenerate_peak2_sptr
raw_make_regenerate_peak2(unsigned int fft_length,
                         unsigned int symbol_length,
                         unsigned int cplen,
                         const std::vector<std::vector<gr_complex> > &preamble,
                         bool debug=false);
class raw_regenerate_peak2 : public gr::sync_block {
 public:
  void set_file_mode(int mode);
};

GR_SWIG_BLOCK_MAGIC(raw,regenerate_peak3)
raw_regenerate_peak3_sptr
raw_make_regenerate_peak3(unsigned int fft_length,
                          unsigned int symbol_length,
                          unsigned int delay,
                          unsigned int peak_delay,
                          const std::vector<std::vector<gr_complex> >  &preamble,
                          bool debug=false);
class raw_regenerate_peak3 : public gr::sync_block {};

GR_SWIG_BLOCK_MAGIC(raw,pnc_frequency_modulator_fc)
raw_pnc_frequency_modulator_fc_sptr raw_make_pnc_frequency_modulator_fc (double sensitivity);
class raw_pnc_frequency_modulator_fc : public gr::sync_block
{
 public:
  void set_sensitivity(float sens) { d_sensitivity = sens; }
  float sensitivity() const { return d_sensitivity; }
  void set_value(float value) { d_value = value; d_phase = 0; }
  float value() const { return d_value; }
};

GR_SWIG_BLOCK_MAGIC(raw,divide_ff)
raw_divide_ff_sptr raw_make_divide_ff(void);
class raw_divide_ff : public gr::sync_block
{
};

