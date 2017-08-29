/* -*- c++ -*- */

%include "gnuradio/swig/gnuradio.i"
%include "raw_swig_block_magic.i"

%{
#include "raw_ofdm_mapper.h"
#include "raw_ofdm_sampler.h"
#include "raw_ofdm_insert_preamble.h"
#include "raw_ofdm_cyclic_prefixer.h"
#include "raw_ofdm_demod.h"
#include "raw_pnc_demod.h"
#include "raw_ofdm_insert_amble.h"
%}

/*
%include "std_vector.i"

namespace std {
  %template(IntVector) vector<int>;
}
*/

RAW_SWIG_BLOCK_MAGIC(raw,ofdm_mapper);
raw_ofdm_mapper_sptr raw_make_ofdm_mapper (std::vector<int> carrier_map, unsigned int node_type=0);
class raw_ofdm_mapper : public gr::sync_block {};


RAW_SWIG_BLOCK_MAGIC(raw,ofdm_sampler)
raw_ofdm_sampler_sptr raw_make_ofdm_sampler (unsigned int fft_length,
               unsigned int symbol_length,
               unsigned int bandwidth,
               unsigned int timeout=1000,
               bool debug=false);
class raw_ofdm_sampler : public gr::sync_block {
 public:
  //void get_init_timestamp(int port, int items_num, pmt::pmt_t KEY, pmt::pmt_t KEY);
  //void wait_until_timestamp(int ts_secs, double ts_frac);
  double get_init_pctime();
  double get_init_hwtime();
  double get_current_hwtime();
  bool set_relay_flag(bool flag);
};

RAW_SWIG_BLOCK_MAGIC(raw,ofdm_insert_preamble);
raw_ofdm_insert_preamble_sptr
raw_make_ofdm_insert_preamble(int fft_length,
				  const std::vector<std::vector<gr_complex> > &preamble);
class raw_ofdm_insert_preamble : public gr::block {};

RAW_SWIG_BLOCK_MAGIC(raw,ofdm_cyclic_prefixer)
raw_ofdm_cyclic_prefixer_sptr 
raw_make_ofdm_cyclic_prefixer (size_t input_size, size_t output_size);
class raw_ofdm_cyclic_prefixer : public gr::sync_interpolator {};

RAW_SWIG_BLOCK_MAGIC(raw,ofdm_demod);
raw_ofdm_demod_sptr
raw_make_ofdm_demod(unsigned int fft_length,
                      unsigned int data_tones,
                      unsigned int cplen,
                      const std::vector<std::vector<gr_complex> > &preamble,
                      std::vector<int> carrier_map, 
                      unsigned int num_data_symbol, 
                      int nbits);
class raw_ofdm_demod : public gr::block
{
 public:
  void set_min_symbols(int val);
};

RAW_SWIG_BLOCK_MAGIC(raw,pnc_demod);
raw_pnc_demod_sptr
raw_make_pnc_demod(unsigned int fft_length,
                                unsigned int data_tones,
                                unsigned int cplen,
                                const std::vector<std::vector<gr_complex> > &preamble,
                                std::vector<int> carrier_map, unsigned int num_data_symbol, float alpha, 
                                int mode, int nbits);
class raw_pnc_demod : public gr::block
{
 public:
  void set_min_symbols(int val);
};


RAW_SWIG_BLOCK_MAGIC(raw,ofdm_insert_amble);
raw_ofdm_insert_amble_sptr
raw_make_ofdm_insert_amble(int fft_length,
          int ofdm_size,
          const std::vector<std::vector<gr_complex> > &preamble,
          const std::vector<std::vector<gr_complex> > &postamble);
class raw_ofdm_insert_amble : public gr::block {};
