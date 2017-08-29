/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You 
 */

#ifndef INCLUDED_RAW_OFDM_DEMOD_H
#define INCLUDED_RAW_OFDM_DEMOD_H

#include <rawofdm_api.h>
#include <gnuradio/block.h>
#include <vector>

class raw_ofdm_demod;
typedef boost::shared_ptr<raw_ofdm_demod> raw_ofdm_demod_sptr;

RAWOFDM_API raw_ofdm_demod_sptr
raw_make_ofdm_demod(unsigned int fft_length,
                    unsigned int data_tones,
                    unsigned int cplen,
                    const std::vector<std::vector<gr_complex> > &preamble,
                    std::vector<int> carrier_map, 
                    unsigned int num_data_symbol, 
                    int nbits);

/*!
 * \brief PNC: perform channel estimation and then PNC demodulation
 *
 * Channel Estimation: 1) use known PN sequences; 2) track and update residual phase for each symbol using pilots; 3) use avg pilot phase
 *
 * PNC Demodulation: 1) Hard Demod; 2) Soft Demod
 *
 * \ingroup ofdm_blk
 *
 *
 */

class RAWOFDM_API raw_ofdm_demod : public gr::block
{
  /*!
   * \brief
   * \param occupied_carriers The number of subcarriers with data in the received symbol
   * \param fft_length        The size of the FFT vector (occupied_carriers + unused carriers)
   * \param cplen             The length of the cycle prefix
   * \param preamble          A vector of vectors of complex numbers representing
   *                          the known symbol at the start of a frame (a PSK PN sequence)
   */
  friend RAWOFDM_API raw_ofdm_demod_sptr
  raw_make_ofdm_demod(unsigned int fft_length,
                      unsigned int data_tones,
                      unsigned int cplen,
                      const std::vector<std::vector<gr_complex> > &preamble,
                      std::vector<int> carrier_map, 
                      unsigned int num_data_symbol, 
                      int nbits);

protected:
  raw_ofdm_demod(unsigned int fft_length,
                 unsigned int data_tones,
                 unsigned int cplen,
                 const std::vector<std::vector<gr_complex> > &preamble,
                 std::vector<int> carrier_map, 
                 unsigned int num_data_symbol, 
                 int nbits);

 private:
  //void forecast(int noutput_items, gr_vector_int &ninput_items_required);

  void get_stream_tags(const int port, const uint64_t nconsumed);
  void get_stream_tags2(const int port, const uint64_t nconsumed);
  void write_stream_tags(int port, uint64_t nproduced);

  // channel estimate computation
  void init_estimate(const gr_complex *symbol);   // one even-freq-only symbol
  void update_estimate(const gr_complex *symbol); // extra symbols
  void finish_estimate(); // normalize
  void demap2(const gr_complex *in, gr_complex *out, float *noise_out, float *signal_out);
  float correlate(const gr_complex *symbol, int &coarse_freq);

  inline gr_complex compensate() const;
  inline int pad() const;

  // params
  unsigned int d_num_carriers;
  unsigned int d_occupied_carriers;   // includes DC
  unsigned int d_data_tones;
  unsigned int d_fft_length;
  unsigned int d_cplen;

  unsigned int        d_left_pad;

  std::vector<int> d_null_carriers;
  std::vector<int> d_data_carriers;
  std::vector<int> d_pilot_carriers;
  const std::vector<std::vector<gr_complex> >  d_preamble;

  // dynamic state
  gr_complex* d_est_x;
  gr_complex* d_demod;

  gr_complex       *d_hestimate;  // channel estimate (includes timing offset)
  float            d_phase;       // one for all
  float            d_freq;        // one for all

  unsigned int d_cur_symbol;        // how many symbols into the frame we are
  unsigned int d_min_symbols;       // how short frames to expect (excluding preambles)
  unsigned int d_num_data_syms;     // how many data symbols to expect in a packet?  

  bool d_newframe;
  bool d_signal_out; // indicate signal_out on the next symbol
  bool d_in_pkt;     // indicate the state: in packet or not

  float d_freq_noise;       // noise power per subcarrier
  float d_freq_signal;      // signal power per subcarrier
  float d_snr_fd;

  uint64_t d_sync_hw_secs;
  double   d_sync_hw_frac;
  uint64_t d_sync_pc_secs;
  double   d_sync_pc_frac;
  double   d_sync_cfo;

  double   d_sync_snr;

  uint64_t  d_decode_time_secs;
  double    d_decode_time_frac;

  float d_coarse_freq;

  // dynamic state
  std::vector<gr_complex> d_symbol_diff; // storage
  std::vector<gr_complex> d_known_diff;  // of d_preamble[0]
  float d_known_norm;

  gr_complex           *d_volk_known_symbol;
  gr_complex           *d_volk_tmp_symbol1;
  gr_complex           *d_volk_tmp_symbol2;

  int d_nbits;

 public:
  ~raw_ofdm_demod(void);
  int general_work(int noutput_items,
                  gr_vector_int &ninput_items,
                  gr_vector_const_void_star &input_items,
                  gr_vector_void_star &output_items);
  void set_min_symbols(int val) { d_min_symbols = val; }
};

#endif //INCLUDED_RAW_OFDM_DEMOD_H
