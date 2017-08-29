/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You, Henry Pan
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_regenerate_peak3.h>
#include <pnc_tags.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/expj.h>
#include <gnuradio/math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <volk/volk.h>
#include <sys/time.h>

#define VERBOSE 0
#define DEBUG 0

static double threshold = 0.000005;

enum {
  STATE_CROSS_NONE,
  STATE_CROSS_ONLYB,
  STATE_CROSS_BOTH,
};

raw_regenerate_peak3_sptr
raw_make_regenerate_peak3(unsigned int fft_length,
                          unsigned int symbol_length,
                          unsigned int nsym_delay,
                          unsigned int peak_delay,
                          const std::vector<std::vector<gr_complex> > &preamble,
                          bool debug)
{
  return raw_regenerate_peak3_sptr (
    new raw_regenerate_peak3 (fft_length, symbol_length, nsym_delay, peak_delay, preamble, debug)
  );
}

raw_regenerate_peak3::raw_regenerate_peak3 (
          unsigned int fft_length,
          unsigned int symbol_length,
          unsigned int nsym_delay,
          unsigned int peak_delay,
          const std::vector<std::vector<gr_complex> > &preamble,
          bool debug)
  : gr::sync_block ("regenerate_peak3",
        gr::io_signature::make3 (3, 3,
          sizeof(char),
          sizeof(gr_complex),
          sizeof(float)),
        gr::io_signature::make2 (2, 2,
          sizeof(char), sizeof(float))),
    d_fft_length(fft_length),
    d_symbol_length(symbol_length),
    DELAY_NSYM(nsym_delay),
    d_len(preamble[0].size()),
    d_preamble(preamble),
    d_auto_peak_delay(peak_delay),
    //d_auto_peak_delay(symbol_length*2),
    d_cross_counter(0),
    d_auto_state(STATE_NO_PEAK),
    d_counter(0),
    d_cur_symbol(0),
    d_cross_state(STATE_NO_PEAK),
    d_num_peaks(0),
    d_data(0.0),
    d_cfo_A(0.0),
    d_cfo_B(0.0),
    d_debug(debug)
{
  d_symbol = new gr_complex[d_len];
  for(unsigned i=0; i<d_len; i++) {
    d_symbol[i] = 0; 
  }
  d_index = 0;
  d_offset = 6;
  d_num_outofcp = 0;
  d_pos_A = d_pos_B = 0;
  d_pos = -1;

  d_cross_state = STATE_NO_PEAK;
  d_energy = 0;
  d_energy_list = new double[DELAY_NSYM];
  for(int i=0; i<DELAY_NSYM; i++)
    d_energy_list[i] = 0;
  d_peak_mode = NO_USER_MODE;

  d_peak_statistics = 0;

  d_peak_A_found = false;
  d_peak_B_found = false;

  d_cross_state = STATE_CROSS_NONE;

  d_symbol_volk = (gr_complex *)volk_malloc(sizeof(gr_complex)*d_len, volk_get_alignment());
  d_known_volk = (gr_complex *)volk_malloc(sizeof(gr_complex)*d_len, volk_get_alignment());

  std::cout << "============= volk_get_alignment() = " << volk_get_alignment() << std::endl;
}

raw_regenerate_peak3::~raw_regenerate_peak3(void)
{
  delete[] d_symbol;
  delete[] d_energy_list;

  volk_free(d_symbol_volk);
  volk_free(d_known_volk);
}

void raw_regenerate_peak3::write_stream_tags(int port, uint64_t pos, pmt::pmt_t KEY)
{
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());

  if(pmt::equal(KEY, PEAK_SU_CFO)) {
    const pmt::pmt_t v = pmt::from_double((double)d_data);
    this->add_item_tag(port, pos, PEAK_SU_CFO, v, _id);
    //printf(" [REGEN_PEAK] add PEAK_SU_CFO at pos = %d \n", pos);
  } 
  else if(pmt::equal(KEY, PEAK_MU_CFO)) {
    const pmt::pmt_t p_list = pmt::list2(pmt::from_double(d_cfo_A), pmt::from_double(d_cfo_B));
    this->add_item_tag(port, pos, PEAK_MU_CFO, p_list, _id);
    //printf(" [REGEN_PEAK] add PEAK_MU_CFO at pos = %d \n", pos);
  }
}

/*
int raw_regenerate_peak3::search_frame_start(float &cfo) {
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

  //d_cor.sort();
  //d_cor.reverse();

  //std::list<std::tr1::tuple<double, double, int>>::iterator it = d_cor.begin();
  //int m = std::tr1::get<2>(*it);
  //cfo = std::tr1::get<1>(*it);

  //d_cor.clear();
  //d_cfo.clear();

  //return m;
}*/

inline float
raw_regenerate_peak3::my_cross_correlate(gr_complex* symbol, int start_index, int length, 
                              const std::vector<gr_complex> &known, float &asum, float &ssum, int mode)
{
  /* Cross-Correlation:
   *   A = symbol[1:M]
   *   B = d_known[1:M]
   *   return abs(\sum A .* conj(B)) / sqrt(norm(A)*norm(B))
   *
   * For mode==1 (repeated preambles): M=N 
   * For mode==2 (single preamble):    M=N/2
   * 
   */
  float correlation = 0.0f;
  int len = (mode==1)?length:length/2;

#if 0
  gr_complex sum(0,0);
  double abs_sum = 0, abs_sum2 = 0;
  double norm_sum = 0, known_sum = 0;

  for(unsigned i=0; i<len; i++) {
    unsigned j = (start_index+i) % length;
    sum += (conj(known[i]) * symbol[j]);

    norm_sum += norm(symbol[i]);
    known_sum += norm(known[i]);
    abs_sum2 += norm(conj(known[i]) * symbol[j]);
  }
  abs_sum = abs(sum);


  float correlation  = abs_sum / sqrt(known_sum * norm_sum);

  if((correlation > 0.7) )//&& d_debug) 
  {
    #if DEBUG
    std::cerr << " #################### correlation = " << correlation << " sum = " << abs_sum << " : " << abs_sum2 << std::endl;
    #endif

  }

  asum = abs_sum;
  ssum = sqrt(known_sum * norm_sum); //norm_sum;

#else
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
  asum = abs(sum_corr);
  ssum = sqrt(sum_known.real() * sum_symbol.real());
  correlation  =  asum / ssum;
  
  #if DEBUG
  if(correlation > 0.7) {
    std::cerr << " #################### correlation = " << correlation << "  = " << asum << "/" <<  ssum << std::endl;
  }
  #endif

#endif
  return correlation;
}

int
raw_regenerate_peak3::work (int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
{
  // [IN]
  //   * Port 0: auto correlation peak indicator
  //   * Port 1: signal (for cross correlation)
  //   * Port 2: cfo value
  // [OUT]
  //   * Port 0: peak position (with different mode)
  //   * Port 1: cfo value
  //
  //  Algorithms: 
  //  we want to differentiate different modes, so 
  //   (1) we should look ahead 3 symbols
  //   (2) we should count the number of peaks 
  //
  
  const char *iptr  = (const char *)  input_items[0];
  const gr_complex *symbol = (const gr_complex *)input_items[1];
  const float *icfo = (const float *) input_items[2];

  char  *optr = (char *)  output_items[0];
  float *ocfo = (float *) output_items[1];

  memset(optr, 0, noutput_items*sizeof(char));
  memset(ocfo, 0, noutput_items*sizeof(float));

  int iport = 0;
  int oport = 0;
  uint64_t nread = this->nitems_read(iport);

  for(int i=0; i < noutput_items; i++) {
    // update d_symbol (circular buffer)
    d_symbol[d_index] = symbol[i];
    d_index = (d_index+1) % d_len;
    
    // -------------------------------------------------------------------- // 
    // based on auto corr results, do cross correlation
    if(iptr[i]) {
      d_auto_state = STATE_AUTO_PEAK;
      d_cross_counter = 0;
      d_cross_counter_A = 0;
      d_cross_counter_B = 0;
      //printf("i=%d\n",i+1);
      d_peak_A_found = false;
      d_peak_B_found = false;
      d_pos_to_start = nread + i;
      d_cross_computation = 0;
      d_cross_state = STATE_CROSS_ONLYB;
      d_peak_mode = NO_USER_MODE;

      #if DEBUG
      struct timeval tv;
      gettimeofday(&tv, NULL);
      d_time_start_cross = tv.tv_sec*1e6 + tv.tv_usec;
      #endif
    }

    if(d_auto_state == STATE_AUTO_PEAK) {
      float sum1 = 0;
      float sum0 = 0;
      float peakv = 0; 

      // Consider the case: only A;  only B; both
      // LTSB is closer to plateau which is generated by 10-STS
      // The auto-cross peak is

      switch(d_cross_state) {

        case STATE_CROSS_ONLYB: {
          if((d_cross_counter < d_auto_peak_delay) && !d_peak_B_found) {
            peakv = my_cross_correlate(d_symbol, d_index, d_len, d_preamble[1], sum1, sum0, 1);
            d_cross_computation++;
            if(peakv > 0.7f) {
              d_peak_B_found = true;
              d_cross_state = STATE_CROSS_NONE;
              d_pos_B = nread + i;
              d_cfo_B = icfo[i];
              d_pos = d_pos_B + d_auto_peak_delay;
              d_auto_state = STATE_HAVE_PEAK;
              d_peak_mode = USER_B_MODE;
              #if DEBUG
              printf(" |REGEN_PEAK3 ONLYB| corr peak pos=%lu\n", nread+i);
              #endif
            }
            if(d_debug) {
              printf(" |REGEN_PEAK3 ONLYB| corr peak in pos=%lu(%lu+%d:%d) | peakv=%.06f | d_num_peaks = %d \n", nread+i, nread, i, noutput_items, peakv, d_num_peaks);
            }
          }

          if(d_cross_counter >= d_auto_peak_delay) {
            d_cross_state = STATE_CROSS_BOTH;
          }
        } break;

        case STATE_CROSS_BOTH: {
          if((d_cross_counter > d_symbol_length) && (d_cross_counter < d_symbol_length + d_auto_peak_delay)
              && !d_peak_A_found) {
            peakv = my_cross_correlate(d_symbol, d_index, d_len, d_preamble[0], sum1, sum0, 1);
            d_cross_computation++;
            if(peakv > 0.7f) {
              d_peak_A_found = true;
              d_pos_A = nread + i;
              d_cfo_A = icfo[i];
              d_pos = d_pos_A + d_symbol_length + d_auto_peak_delay;
              d_peak_B_found = false;
              #if DEBUG
              printf(" |REGEN_PEAK3 BOTH/A| corr peak pos=%lu\n", nread+i);
              #endif
            }

            if(d_debug) {
              printf(" |REGEN_PEAK3 BOTH/A| corr peak in pos=%lu(%lu+%d:%d) | peakv=%.06f | d_num_peaks = %d \n", nread+i, nread, i, noutput_items, peakv, d_num_peaks);
            }

          }

          if((d_cross_counter > d_symbol_length*2) && (d_cross_counter < d_symbol_length*2 + d_auto_peak_delay) 
              && !d_peak_B_found) {
            peakv = my_cross_correlate(d_symbol, d_index, d_len, d_preamble[1], sum1, sum0, 1);
            d_cross_computation++;
            if(peakv > 0.7f) {
              d_peak_B_found = true;
              d_pos_B = nread + i;
              d_cfo_B = icfo[i];
              d_pos = d_pos_B + d_auto_peak_delay;

              if(d_peak_A_found) {
                d_peak_mode = TWO_USER_MODE;
              }
              d_auto_state = STATE_HAVE_PEAK;

              #if DEBUG
              printf(" |REGEN_PEAK3 BOTH/B| corr peak pos=%lu\n", nread+i);
              #endif
            }

            if(d_debug) {
              printf(" |REGEN_PEAK3 BOTH/B| corr peak in pos=%lu(%lu+%d:%d) | peakv=%.06f | d_num_peaks = %d \n", nread+i, nread, i, noutput_items, peakv, d_num_peaks);
            }
          }

        } break;

        default:
          break;
      }

      d_cross_counter++;

      #if 0
      printf("\t d_peak_A_found=%d d_peak_B_found=%d d_cross_counter=%d->%d  d_pos=%lu %lu\n", 
          d_peak_A_found, d_peak_B_found, d_cross_counter, d_auto_peak_delay + 2*d_symbol_length, d_pos, nread+i);
      #endif

      if(d_cross_counter > d_symbol_length*2 + d_auto_peak_delay) {
        if(d_peak_A_found) {
          d_peak_mode = USER_A_MODE;
          d_auto_state = STATE_HAVE_PEAK;
        }
      }

    }

     // -------------------------------------------------------------------- //
    if(d_auto_state == STATE_HAVE_PEAK) {

      if(nread + i == d_pos) {
        if(d_debug) {
          printf(" |REGEN_PEAK3 | d_cross_computation = %d  d_pos=%ld d_peak_mode=%d\n", d_cross_computation, d_pos, d_peak_mode);
        }
        
        #if DEBUG
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double d_time_stop_cross = tv.tv_sec*1e6 + tv.tv_usec;
        printf(" |REGEN_PEAK3 | Time from begining to cross to reaching d_pos  tv=%.06lfus\n", d_time_stop_cross - d_time_start_cross);
        #endif

        optr[i] = d_peak_mode;
        // write the cfo information
        int tagpos = nitems_written(oport)+i;                       // i means the offset of i+1-th items
        if(d_peak_mode >= TWO_USER_MODE) {
          // add MU cfo stream tags
          d_data = (d_cfo_A + d_cfo_B)/2;
          write_stream_tags(oport, tagpos, PEAK_MU_CFO);
        }
        else if(d_peak_mode > NO_USER_MODE) {
          // add SU cfo stream tags
          d_data = d_cfo_A;
          write_stream_tags(oport, tagpos, PEAK_SU_CFO);
        }

      }
      
      if(nread + i >= d_pos) {
        ocfo[i] = d_data;
      }

    }

  }
 
  return noutput_items;
}
