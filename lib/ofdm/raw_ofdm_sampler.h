/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 * Copyright 2014 Lizhao You
 */

#ifndef INCLUDED_RAW_OFDM_SAMPLER_H
#define INCLUDED_RAW_OFDM_SAMPLER_H

#include <rawofdm_api.h>
#include <gnuradio/sync_block.h>

class raw_ofdm_sampler;
typedef boost::shared_ptr<raw_ofdm_sampler> raw_ofdm_sampler_sptr;

RAWOFDM_API raw_ofdm_sampler_sptr raw_make_ofdm_sampler(unsigned int fft_length,
                                            unsigned int symbol_length,
                                            unsigned int bandwidth,
                                            unsigned int timeout=1000,
                                            bool debug=false);

/*!
 * \brief samples the signal at symbol boundaries
 * \ingroup ofdm_blk
 * Inputs:
 *  [0] signal to be sampled
 *  [1] indicator of where the preamble symbol starts (first sample)
 * Outputs:
 *  [0] sampled symbols (fft_length long)
 *  [1] indicator of which symbol is the preamble (1 per symbol)
 */
class RAWOFDM_API raw_ofdm_sampler : public gr::block
{
  friend RAWOFDM_API raw_ofdm_sampler_sptr raw_make_ofdm_sampler (
                unsigned int fft_length,
                unsigned int symbol_length,
                unsigned int bandwidth,
                unsigned int timeout,
                bool debug);

  raw_ofdm_sampler (unsigned int fft_length,
                    unsigned int symbol_length,
                    unsigned int bandwidth,
                    unsigned int timeout,
                    bool debug);

 private:
  // params
  unsigned int d_timeout_max;
  unsigned int d_fft_length;
  unsigned int d_symbol_length;
  unsigned int d_bandwidth;

  // dynamic state
  enum state_t {STATE_NO_SIG, STATE_PREAMBLE, STATE_FRAME};
  state_t d_state;
  unsigned int d_timeout;      // remaining count

  uint64_t  d_init_hw_secs;    // init hw timestamp: secs
  double    d_init_hw_frac;    // init hw timestamp: frac
  uint64_t  d_init_pc_secs;    // init pc timestamp: secs
  double    d_init_pc_frac;    // init pc timestamp: frac

  uint64_t  d_samples_passed;  // number of samples passed
  double    d_time_per_sample; // time per sample

  uint64_t  d_sample_index;    // the required sample index

  double    d_noise;           // average noise power
  double    d_sum_noise;       // sum noise power
  int       d_noise_count;     // number of noise samples

  double    d_symbol1;         // signal power of the first symbol after frame sync
  double    d_symbol2;         // signal power of the second symbol after frame sync

  int       d_cur_symbol;      // current number of symbols
  int       d_peak_count;      // number of peaks detected
  bool      d_new_pkt;         // indicate a new packet
  bool      d_debug;

  bool      d_relay;           // indicate a relay node

  double    d_sync_cfo;
  double    d_sync_cfo0;
  double    d_sync_cfo1;

  uint64_t  d_decode_time_secs;
  double    d_decode_time_frac;

  int       d_trigger_val;
  
 private:
  void get_init_timestamp(int port, int items_num, pmt::pmt_t KEY, pmt::pmt_t KEY2);
  void write_stream_tags1(int out_port, int rel_pos);
  void write_stream_tags2(int out_port, int rel_pos);
  void read_stream_tags(int in_port, int rel_pos);
  
 public:
  void forecast (int noutput_items, gr_vector_int &ninput_items_required);

  double get_init_pctime() { return d_init_pc_secs+d_init_pc_frac; };
  double get_init_hwtime() { return d_init_hw_secs+d_init_hw_frac; };
  double get_current_hwtime();
  bool set_relay_flag(bool flag) { d_relay = flag; return true;}

  int general_work (int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items);
};

#endif
