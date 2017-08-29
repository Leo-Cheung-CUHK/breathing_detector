/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You, Henry Pan
 */

#ifndef INCLUDED_RAW_REGENERATE_PEAK3_H
#define INCLUDED_RAW_REGENERATE_PEAK3_H

#include <rawofdm_api.h>
#include <gnuradio/sync_block.h>
#include <vector>
#include <list>
#include <tr1/tuple>

class raw_regenerate_peak3;
typedef boost::shared_ptr<raw_regenerate_peak3> raw_regenerate_peak3_sptr;

RAWOFDM_API raw_regenerate_peak3_sptr
raw_make_regenerate_peak3(unsigned int fft_length,
                          unsigned int symbol_length,
                          unsigned int nsym_delay,
                          unsigned int peak_delay,
                          const std::vector<std::vector<gr_complex> >  &preamble,
                          bool debug=false);

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

class RAWOFDM_API raw_regenerate_peak3 : public gr::sync_block
{
  /*!
   * \brief
   * \param fft_length        The size of the FFT vector (occupied_carriers + unused carriers)
   * \param symbol_length     The size of the symbol length (fft_length + cp_length)
   * \param nsym_delay        The number of symbols delayed in input signal wrt. the cross peak
   * \param peak_delay        The number of samples delayed in input signal wrt. the auto peak
   * \param preamble          A vector of vectors of complex numbers representing
   *                          the known symbol at the start of a frame (a PSK PN sequence)
   */
  friend RAWOFDM_API raw_regenerate_peak3_sptr
  raw_make_regenerate_peak3(unsigned int fft_length,
                            unsigned int symbol_length,
                            unsigned int nsym_delay,
                            unsigned int peak_delay,
                            const std::vector<std::vector<gr_complex> > &preamble,
                            bool debug);

protected:
  raw_regenerate_peak3 (unsigned int fft_length,
                        unsigned int symbol_length,
                        unsigned int nsym_delay,
                        unsigned int peak_delay,
                        const std::vector<std::vector<gr_complex> > &preamble,
                        bool debug);

 private:
  void write_stream_tags(int port, uint64_t pos, pmt::pmt_t KEY);
  //int search_frame_start(float &cfo);  // not used in this version

  // params
  unsigned int DELAY_NSYM;

  unsigned int d_fft_length;
  unsigned int d_symbol_length;
  unsigned int d_len;                      // length of correlation preamble
  const std::vector<std::vector<gr_complex> >  d_preamble;

  enum {STATE_NO_PEAK, STATE_HAVE_PEAK, STATE_AUTO_PEAK};

  // dynamic state
  gr_complex*        d_symbol;             // store input symbols
  float*             d_cfo;                // store input cfos 
  int                d_index;              // the current sample index
  int                d_cur_symbol;         // the current symbol index

  double             d_data;               // final cfo data
  double             d_cfo_A;              // two user case: cfo of A
  double             d_cfo_B;              // two user case: cfo of B

  // parameters for auto correlation
  int                d_auto_state;         // auto correlation state
  int                d_auto_peak_delay;    // delay before the auto peak
  

  int                d_counter;            // counter for number of processing items
  int                d_cross_state;        // have we found a peak? (by cross corr)
  int                d_num_peaks;          // couting how many peaks
  uint64_t           d_pos_A;              // peak position of user A
  uint64_t           d_pos_B;              // peak position of user B
  uint64_t           d_pos;                // the final peak position
  int                d_offset;             // cfo offset
  char               d_peak_mode;          // the mode after cross correlation
  double*            d_energy_list;        // energy of three continous symbol
  double             d_energy;             // energy of one symbol

  int                d_num_outofcp;        // num of [out of cp]
  bool               d_debug;

  size_t             d_peak_statistics;

  bool               d_peak_A_found;
  bool               d_peak_B_found;
  uint64_t           d_pos_to_start;
  uint64_t           d_peak_B_pos;

  int                d_cross_counter;      // after auto peak, how many cross corr performed
  int                d_cross_counter_A;      // after auto peak, how many cross corr performed
  int                d_cross_counter_B;      // after auto peak, how many cross corr performed
  int                d_cross_computation;

  gr_complex*        d_symbol_volk;
  gr_complex*        d_known_volk;

  double             d_time_start_cross;

  //std::vector<int>    d_peak_pos;
  //std::vector<double> d_peak_cfo;
  //std::vector<double> d_peak_val;

 inline float my_cross_correlate(gr_complex* symbol, int start_index, int length, 
                              const std::vector<gr_complex> &known, float &asum, float &ssum, int mode);
 public:
  ~raw_regenerate_peak3(void);

  int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif //INCLUDED_RAW_OFDM_REGENRATE_PEAK3_H
