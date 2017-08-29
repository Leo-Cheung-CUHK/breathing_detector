/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */

#ifndef INCLUDED_RAW_REGENERATE_PEAK2_H
#define INCLUDED_RAW_REGENERATE_PEAK2_H

#include <rawofdm_api.h>
#include <gnuradio/sync_block.h>
#include <vector>
#include <list>
#include <tr1/tuple>

#define MAX_LEN 50

class raw_regenerate_peak2;
typedef boost::shared_ptr<raw_regenerate_peak2> raw_regenerate_peak2_sptr;

RAWOFDM_API raw_regenerate_peak2_sptr
raw_make_regenerate_peak2(unsigned int fft_length,
                          unsigned int symbol_length,
                          unsigned int delay,
                          const std::vector<std::vector<gr_complex> >  &preamble,
                          bool debug);

/*!
 * \brief Takes output of the self-correlation and performs cross-correlation 
 * with the known symbol to find the exact packet beginning.
 *
 * The PN sequence used for known symbol should be a PSK sequence.
 * The known symbol is known beforehand.
 *
 * \ingroup ofdm_blk
 *
 */

class RAWOFDM_API raw_regenerate_peak2 : public gr::sync_block
{
  /*!
   * \brief
   * \param fft_length        The size of the FFT vector (occupied_carriers + unused carriers)
   * \param symbol_length     The size of the symbol length (fft_length + cp_length)
   * \param delay             The length of delay in input signal
   * \param preamble          A vector of vectors of complex numbers representing
   *                          the known symbol at the start of a frame (a PSK PN sequence)
   */
  friend RAWOFDM_API raw_regenerate_peak2_sptr
  raw_make_regenerate_peak2(unsigned int fft_length,
                            unsigned int symbol_length,
                            unsigned int delay,
                            const std::vector<std::vector<gr_complex> > &preamble,
                            bool debug);

protected:
  raw_regenerate_peak2 (unsigned int fft_length,
                        unsigned int symbol_length,
                        unsigned int delay,
                        const std::vector<std::vector<gr_complex> > &preamble,
                        bool debug);

 private:
  inline float my_cross_correlate(gr_complex* symbol, int start_index, int length, const std::vector<gr_complex> &known, int mode);
  void write_stream_tags(int port, uint64_t pos, pmt::pmt_t KEY);
  int search_frame_start(float &cfo);

  // params
  unsigned int DELAY_NSYM;

  unsigned int d_fft_length;
  unsigned int d_symbol_length;
  unsigned int d_delay;
  unsigned int d_len;                      // length of correlation preamble
  const std::vector<std::vector<gr_complex> >  d_preamble;

  enum {STATE_NO_PEAK, STATE_AUTO_PEAK, STATE_HAVE_PEAK};

  // dynamic state
  gr_complex*        d_symbol;             // store input symbols
  float*             d_cfo;                // store input cfos 
  gr_complex*        d_symbol_volk;         // store input symbols in sequence for volk calculation
  gr_complex*        d_known_volk;
  float              d_norm_known;

  double             d_data;               // final cfo data
  double             d_cfo_A;              // two user case: cfo of A
  double             d_cfo_B;              // two user case: cfo of B

  int                d_start;              // start position of d_symbol 
  int                d_counter;            // counter for number of processing items
  int                d_state;              // have we found a peak?
  int                d_num_peaks;          // couting how many peaks

  int                d_num_outofcp;        // num of [out of cp]
  
  bool               d_debug;

  //std::list<std::tr1::tuple<double, double, int>> d_cor;
  std::vector<int> d_peak_pos;
  std::vector<double> d_peak_cfo;
  std::vector<double> d_peak_val;

  size_t            d_auto_peak_pos;
  size_t            d_cross_peak_pos;

  FILE*               d_file;

 public:
  ~raw_regenerate_peak2(void);

  void set_file_mode(int mode);

  int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif //INCLUDED_RAW_OFDM_REGENRATE_PEAK2_H
