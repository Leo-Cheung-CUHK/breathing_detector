/* -*- c++ -*- */
/*
 * Copyright 2014 You Lizhao
 */

#ifndef INCLUDED_RAW_PNC_DEMOD_H
#define INCLUDED_RAW_PNC_DEMOD_H

#include <rawofdm_api.h>
#include <gnuradio/block.h>
#include <vector>

class raw_pnc_demod;
typedef boost::shared_ptr<raw_pnc_demod> raw_pnc_demod_sptr;

RAWOFDM_API raw_pnc_demod_sptr
raw_make_pnc_demod(unsigned int fft_length,
                      unsigned int data_tones,
                      unsigned int cplen,
                      const std::vector<std::vector<gr_complex> > &preamble,
                      std::vector<int> carrier_map, unsigned int num_data_symbol, float alpha, int mode, int nbits);

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

class RAWOFDM_API raw_pnc_demod : public gr::block
{
  /*!
   * \brief
   * \param occupied_carriers The number of subcarriers with data in the received symbol
   * \param fft_length        The size of the FFT vector (occupied_carriers + unused carriers)
   * \param cplen             The length of the cycle prefix
   * \param preamble          A vector of vectors of complex numbers representing
   *                          the known symbol at the start of a frame (a PSK PN sequence)
   */
  friend RAWOFDM_API raw_pnc_demod_sptr
  raw_make_pnc_demod(unsigned int fft_length,
                     unsigned int data_tones,
                     unsigned int cplen,
                     const std::vector<std::vector<gr_complex> > &preamble,
                     std::vector<int> carrier_map, unsigned int num_data_symbol, float alpha, int mode, int nbits);

protected:
  raw_pnc_demod (unsigned int fft_length,
                 unsigned int data_tones,
                 unsigned int cplen,
                 const std::vector<std::vector<gr_complex> > &preamble,
                 std::vector<int> carrier_map, unsigned int num_data_symbol, float alpha, int mode, int nbits);

 private:
  void forecast(int noutput_items, gr_vector_int &ninput_items_required);
  unsigned char joint_hard_demodulation(const gr_complex in, const gr_complex H_A, const gr_complex H_B, unsigned char &A, unsigned char &B);
  unsigned char joint_soft_demodulation(const gr_complex in, const gr_complex H_A, const gr_complex H_B, unsigned char &A, unsigned char &B);
  unsigned char joint_soft_demodulation2(const gr_complex in, const gr_complex H_A, const gr_complex H_B, unsigned char &A, unsigned char &B);

  // channel estimate computation
  void init_estimate(const gr_complex *symbol); 	    // initialization
  void update_estimate_A(const gr_complex *symbol, const gr_complex *symbol_td);     // extra symbols
  void update_estimate_B(const gr_complex *symbol, const gr_complex *symbol_td);     // extra symbols

  // soft decoding functions
  unsigned char calc_confidence1(const gr_complex in, const gr_complex x1, const gr_complex x2, float normalized_factor, float alpha);
  inline int pad() const;
  inline float calc_H_max();

  float avg_phase(float value1, float value2);

  gr_complex get_phase_complex(const gr_complex in);
  gr_complex get_phase_complex(const std::vector<gr_complex> in);
  void update_channel_estimation(const gr_complex rotation_A, const gr_complex rotation_B);
  
  void get_stream_tags(int port, const uint64_t nconsumed);
  void write_stream_tags(int port, const uint64_t nproduced);

  // general qam decoding
  gr_complex *d_demod;
  void qam_decode(const float *demod, const unsigned char *out);
  
  // not useful in this version
  void get_snr_tags(const int port, const uint64_t nconsumed);
  void add_power_tag(int port, int nout);
  void add_snr_tag(int port, int nout);
  void get_write_stream_tags(int in_port, int out_port, const uint64_t nconsumed, const uint64_t nproduced);
  

  // fixed params
  unsigned int d_occupied_carriers; // includes DC
  unsigned int d_data_tones;
  unsigned int d_fft_length;
  unsigned int d_cplen;
  unsigned int d_num_carriers;
  unsigned int d_mode;				// demodulation type: 1/2/3 (soft), 0 (hard)
  unsigned int d_min_symbols;   		// how short frames to expect (excluding preambles)
  unsigned int  d_pad;

  unsigned int d_cur_frame;
  unsigned int d_num_data_symbol;

  int d_nbits;

  float d_max_H_A;        		// max channel A over all subcarriers
  float d_max_H_B;        		// max channel B over all subcarriers
  float d_max_H;			// max channel AB over all subcarriers
  float d_max_2H;                       // defined in finish_estimate()
  float d_alpha2;			// new scaling factor for our maximum possible range

  std::vector<int> d_null_carriers;
  std::vector<int> d_data_carriers;
  std::vector<int> d_pilot_carriers;

  const std::vector<std::vector<gr_complex> >  d_preamble;

  bool  d_signal_out; 			// should indicate signal_out on the next symbol

  // dynamic state
  int                        d_user_mode;
  bool                       d_newframe;
  unsigned int d_cur_symbol;                    // how many symbols into the frame we are
  unsigned int d_cur_carrier;			// how many subcarrier into the symbol we are
  std::vector<gr_complex> d_channel_A;  	// channel estimate of A
  std::vector<gr_complex> d_channel_B;  	// channel estimate of B
  std::vector<gr_complex> d_pilot_rotate_A;	// pilot rotation of A (for each sym)
  std::vector<gr_complex> d_pilot_rotate_B;	// pilot rotation of B (for each sym)

  // time-domain SNR (preserve for future extention)
  double d_pre_signal_A;		// signal power of A's preamble (time-domain)
  double d_pre_signal_B;		// signal power of B's preamble (time-domain)
  double d_pre_noise;			// noise  power before preamble (time-domain)

  // freq-domain SNR (preserve for future extention)
  double d_noise;			// noise  power using data subcarriers
  double d_noise_A;			// noise  power of user A using pilots
  double d_noise_B;			// noise  power of user B using pilots

  double d_signal;		        // signal power of user A+B using data subcarriers
  double d_signal_A;			// signal power of user A using pilots
  double d_signal_B;			// signal power of user B using pilots
  double d_signal2_A;			// signal power of user A using data subcarriers
  double d_signal2_B;			// signal power of user B using data subcarriers

  double d_previous_snr_A;
  double d_previous_snr_B;
  double d_previous_snr;                // noise of previous packet as an estimation for current packet
  
  uint64_t d_sync_hw_secs;
  double   d_sync_hw_frac;
  uint64_t d_sync_pc_secs;
  double   d_sync_pc_frac;
  double   d_sync_cfo1;
  double   d_sync_cfo2;

  double   d_sync_snrA;
  double   d_sync_snrB;


 public:
  ~raw_pnc_demod(void);
  int general_work(int noutput_items,
                  gr_vector_int &ninput_items,
                  gr_vector_const_void_star &input_items,
                  gr_vector_void_star &output_items);
  void set_min_symbols(int val) { d_min_symbols = val; }
};

#endif //INCLUDED_RAW_PNC_DEMOD_H
