/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_regenerate_peak2.h>
#include <pnc_tags.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/expj.h>
#include <gnuradio/math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <sys/time.h>
#include <volk/volk.h>

#define VERBOSE 0
#define DEBUG 0
#define M_TWOPI (2*M_PI)
#define MAX_NUM_SYMBOLS 1000

raw_regenerate_peak2_sptr
raw_make_regenerate_peak2(unsigned int fft_length,
                          unsigned int symbol_length,
                          unsigned int delay,
                          const std::vector<std::vector<gr_complex> > &preamble,
                          bool debug)
{
  return raw_regenerate_peak2_sptr (
    new raw_regenerate_peak2 (fft_length, symbol_length, delay, preamble, debug)
  );
}

raw_regenerate_peak2::raw_regenerate_peak2 (
          unsigned int fft_length,
          unsigned int symbol_length,
          unsigned int delay,
          const std::vector<std::vector<gr_complex> > &preamble,
          bool debug)
  : gr::sync_block ("regenerate_peak2",
        gr::io_signature::make3 (3, 3,
          sizeof(char),
          sizeof(float),
          sizeof(gr_complex)),
        gr::io_signature::make2 (2, 2,
          sizeof(char), sizeof(float))),
    d_fft_length(fft_length),
    d_symbol_length(symbol_length),
    d_delay(symbol_length-fft_length),
    DELAY_NSYM(0),
    d_len(preamble[0].size()),
    d_preamble(preamble),
    d_counter(0),
    d_state(STATE_NO_PEAK),
    d_num_peaks(0),
    d_data(0.0),
    d_cfo_A(0.0),
    d_cfo_B(0.0),
    d_debug(debug)
{
  d_symbol = new gr_complex[d_len];
  d_cfo    = new float[d_len];
  for(unsigned i=0; i<d_len; i++) {
    d_symbol[i] = 0; 
    d_cfo[i] = 0;
  }
  d_start = 0;

  set_tag_propagation_policy(TPP_DONT);  /* TPP_DONT TPP_ALL_TO_ALL TPP_ONE_TO_ONE */
  d_file = NULL;

  d_symbol_volk = (gr_complex *)volk_malloc(sizeof(gr_complex)*d_len, volk_get_alignment());
  d_known_volk = (gr_complex *)volk_malloc(sizeof(gr_complex)*d_len, volk_get_alignment());

  std::cout << "============= volk_get_alignment() = " << volk_get_alignment() << std::endl;

}

raw_regenerate_peak2::~raw_regenerate_peak2(void)
{
  delete[] d_symbol;
  delete[] d_cfo;
  
  volk_free(d_symbol_volk);
  volk_free(d_known_volk);

  if (d_file != NULL)
    fclose(d_file);

}

void raw_regenerate_peak2::set_file_mode(int mode)
{
  if(mode == 0)
    d_file = fopen("raw_regen_peak2_A.log", "w");
  else if(mode == 1)
    d_file = fopen("raw_regen_peak2_B.log", "w");
  else
    printf("[REGEN2] mode error, must be 0 (A) or 1 (B) \n");
}

#define DEBUG_CROSSCORR_TIME  0

inline float
raw_regenerate_peak2::my_cross_correlate(gr_complex* symbol, int start_index, int length, const std::vector<gr_complex> &known, int mode)
{
  #if DEBUG_CROSSCORR_TIME
  struct timeval tv_s;
  gettimeofday(&tv_s, NULL);
  #endif

  float correlation = 0;

#if 1
  int len = (mode==1)?length:length/2;
  gr_complex sum_corr(0,0);
  gr_complex sum_known(0,0);
  gr_complex sum_symbol(0,0);

  for(unsigned i=0, j; i<len; i++) {
    j= (start_index+i) % length;
    d_symbol_volk[i] = symbol[j];
    d_known_volk[i] = known[i];
  }
  // volk_32fc_x2_conjugate_dot_prod_32fc volk_32fc_x2_dot_prod_32fc
  volk_32fc_x2_conjugate_dot_prod_32fc(&sum_corr, d_known_volk, d_symbol_volk, len);
  volk_32fc_x2_conjugate_dot_prod_32fc(&sum_known, d_known_volk, d_known_volk, len);
  volk_32fc_x2_conjugate_dot_prod_32fc(&sum_symbol, d_symbol_volk, d_symbol_volk, len);
  correlation  = abs(sum_corr) / sqrt(sum_known.real() * sum_symbol.real());
  //std::cout << "volk imag:  " << sum_known.imag() << " " << sum_symbol.imag() << std::endl;

#else
  /* 
   * Cross-Correlation:
   *   A = symbol[1:M]
   *   B = d_known[1:M]
   *   return abs(\sum A .* conj(B)) / sqrt(norm(A)*norm(B))
   *
   * For mode==1 (repeated preambles): M=N 
   * For mode==2 (single preamble):    M=N/2
   * 
   */

  gr_complex sum(0,0);
  double abs_sum = 0, abs_sum2 = 0;
  double norm_sum = 0, known_sum = 0;
  int len = (mode==1)?length:length/2;

  for(unsigned i=0; i<len; i++) {
    unsigned j = (start_index+i) % length;
    sum += (conj(known[i]) * symbol[j]);

    norm_sum += norm(symbol[i]);
    known_sum += norm(known[i]);
    abs_sum2 += norm(conj(known[i]) * symbol[j]);
  }
  abs_sum = abs(sum);


  correlation  = abs_sum / sqrt(known_sum * norm_sum);

  if((correlation > 0.7) && d_debug) {
    int nitems_written = this->nitems_written(0); // lzyou: not accurate, a rough position estimate
    std::cerr << " #################### correlation = " << correlation << " sum = " << abs_sum << " : " << abs_sum2 << " | " << nitems_written << std::endl;
  }
#endif



  #if DEBUG_CROSSCORR_TIME
  struct timeval tv_e;
  gettimeofday(&tv_e, NULL);
  printf("%s time=%.06fus, correlation = %.06f\n", __FUNCTION__, ((tv_e.tv_sec*1e6 + tv_e.tv_usec)-(tv_s.tv_sec*1e6 + tv_s.tv_usec)), correlation);;
  #endif

  return correlation;
}

void raw_regenerate_peak2::write_stream_tags(int port, uint64_t pos, pmt::pmt_t KEY)
{
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());

  if(pmt::equal(KEY, PEAK_SU_CFO)) {
    const pmt::pmt_t v = pmt::from_double((double)d_data);
    this->add_item_tag(port, pos, PEAK_SU_CFO, v, _id);
    //printf(" [REGEN_PEAK2] add PEAK_SU_CFO at pos = %d \n", pos);
  }
  else if(pmt::equal(KEY, PKT_DECODE_TIME)) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    const pmt::pmt_t v = pmt::make_tuple( pmt::from_uint64(tv.tv_sec),
                                          pmt::from_double(tv.tv_usec*1.0/1e6));
    this->add_item_tag(port, pos, PKT_DECODE_TIME, v, _id);

    if( (d_file != NULL) ) {
        fprintf(d_file, "%lu %.06f \n", tv.tv_sec, tv.tv_usec*1.0/1e6);
      }

  }

}

int raw_regenerate_peak2::search_frame_start(float &cfo) {
  //int p = std::distance(std::begin(v), std::max_element(std::begin(d_peak_val), std::end(d_peak_val))
  if(d_peak_cfo.size() == 0)
    return -1;

  int p = -1, r = 0;
  double v = -1;
  for(int i=0; i<d_peak_val.size(); i++)  {
    //std::cerr << d_peak_pos[i] << " " << d_peak_val[i] << " " << d_peak_cfo[i] << std::endl;
    if(d_peak_val[i]>v) {
      r = i;
      p = d_peak_pos[i]; 
      v = d_peak_val[i];
    }
  }
  cfo = d_peak_cfo[r];

  d_peak_cfo.clear();
  d_peak_val.clear();
  d_peak_pos.clear();

  return p;

  /*d_cor.sort();
  d_cor.reverse();

  std::list<std::tr1::tuple<double, double, int>>::iterator it = d_cor.begin();
  int m = std::tr1::get<2>(*it);
  cfo = std::tr1::get<1>(*it);

  d_cor.clear();
  d_cfo.clear();

  return m; */
}

int
raw_regenerate_peak2::work (int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
{
  // [IN]
  //   * Port 0: auto-correlation peak
  //   * Port 1: cfo value (d_delay behind auto peak)
  //   * Port 2: signal (d_delay behind auto peak)
  // [OUT]
  //   * Port 0: cross-correlation peak for one user
  //   * Port 1: cfo value
  //
  // We do cross-correlation to find the exact position
  //
  // Assumption:
  //   * The input symbols are delayed by d_delay samples (iptr[i] ~ symbol[i+cp_length])
  //   * The cut signal should also be delayed by d_delay samples
  //   * You can implement CP cut by delayed less than d_delay samples, e.g., d_delay-offset means cut at offset

  //printf("[PEAK2] noutput_items = %d\n", noutput_items);

  const char *iptr = (const char *) input_items[0];
  const float *cfo  = (const float *) input_items[1];
  const gr_complex *symbol = (const gr_complex *)input_items[2];

  char *optr = (char *) output_items[0];  
  float *ocfo = (float *) output_items[1];

  memset(optr, 0, noutput_items*sizeof(char));
  memset(ocfo, 0, noutput_items*sizeof(float));

  int iport = 0;
  uint64_t nread = this->nitems_read(iport);
  for(int i=0; i < noutput_items; i++) {

    // update d_symbol, d_cfo (circular buffer)
    d_symbol[d_start] = symbol[i];
    d_cfo[d_start] = cfo[i];
    d_start = (d_start+1) % d_len;

      // ------------------------------------------------- //    
      /*int index = i;
      int NSAMPLE_PER_BLOCK = 1000000;
      if( (d_file != NULL) && ((nread+index) % NSAMPLE_PER_BLOCK == 0) ) {
        timeval sampler_timer;
        gettimeofday(&sampler_timer, NULL);
        double pctime = sampler_timer.tv_sec+sampler_timer.tv_usec*1.0/1e6;
        fprintf(d_file, "%lu %f \n", (nread+index)/NSAMPLE_PER_BLOCK, pctime);
        //printf("%d %f \n", nread+index, pctime);
      }*/
      // ------------------------------------------------- //

    if(iptr[i]) {
      d_auto_peak_pos = nread+i;
      if(d_debug)
        printf("  |PEAK| auto peak in pos: %lu(%lu+%d) \n", nread+i, nread, i);

      d_state = STATE_AUTO_PEAK;
      d_counter = 0;
      d_data = 0;
    } 

    float value = 0;
    if(d_state == STATE_AUTO_PEAK) {
      value = my_cross_correlate(d_symbol, d_start, d_len, d_preamble[0], 1);

      if(d_debug)
        printf(" |PEAK| corr peak in pos=%lu(%lu+%d:%d) | value = %f | d_num_peaks = %d \n", nread+i, nread, i, noutput_items, value, d_num_peaks);

      d_cross_peak_pos = nread+i;

      if(value > 0.7f) {
        d_num_peaks++;
        d_data = d_cfo[((d_start-1)+d_len)%d_len];
        optr[i] = 1;
        ocfo[i] = d_data;
        write_stream_tags(iport, nread+i, PEAK_SU_CFO);
        write_stream_tags(iport, nread+i, PKT_DECODE_TIME);

        if(d_debug)
          printf(" |PEAK| CFO = %f  Peak vary: %zu\n", d_data,  d_cross_peak_pos - d_auto_peak_pos);

        d_state = STATE_NO_PEAK;
      }

      // here we assume the maximal cross-correlation range is [-d_delay:+d_delay]
      // reset to zero if we do enough cross-corelation
      if(d_counter == d_delay) {
        d_state = STATE_NO_PEAK;

      }
    }

    d_counter++;

    // FIXME: currently disable cfo compensation
    // Assumption: you have GPSDO
    ocfo[i] = 0;
  }

 return noutput_items;
}
