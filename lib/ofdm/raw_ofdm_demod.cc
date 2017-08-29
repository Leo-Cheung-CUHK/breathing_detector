/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_ofdm_demod.h>
#include <raw_qam.h>
#include <pnc_tags.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/fxpt.h>
#include <gnuradio/expj.h>
#include <gnuradio/sincos.h>
#include <gnuradio/math.h>
#include <iostream>
#include <stdio.h>
#include <sched.h>      // added this to get the time Quantum
#include <assert.h>
#include <volk/volk.h>

#define M_TWOPI (2*M_PI)
#define VERBOSE_TAG 0

#define DEBUG_COMPUTE_TIME 0

raw_ofdm_demod_sptr raw_make_ofdm_demod(unsigned int fft_length,
                                      unsigned int data_tones,
                                      unsigned int cplen,
                                      const std::vector<std::vector<gr_complex> > &preamble,
                                      std::vector<int> carrier_map,
                                      unsigned int num_data_symbol,
                                      int nbits)
{
  return raw_ofdm_demod_sptr(new raw_ofdm_demod(fft_length,
                                              data_tones,
                                              cplen,
                                              preamble,
                                              carrier_map,
                                              num_data_symbol,
                                              nbits) );
}

// lzyou: we should keep data_tones parameter used in gr_make_io_signature, though it can be set as length(d_data_carriers)
raw_ofdm_demod::raw_ofdm_demod(unsigned int fft_length,
                             unsigned int data_tones,
                             unsigned int cplen,
                             const std::vector<std::vector<gr_complex> > &preamble,
                             std::vector<int> carrier_map,
                             unsigned int num_data_symbol,
                             int nbits) :
			gr::block("ofdm_demod",
				gr::io_signature::make2(2, 2, sizeof(gr_complex) * fft_length,
						sizeof(char)),
				gr::io_signature::make2(2, 2, sizeof(unsigned char) * data_tones,
						sizeof(char))),
				d_num_carriers(preamble[0].size()),
				d_fft_length(fft_length),
				d_cplen(cplen),
				d_preamble(preamble),
				d_cur_symbol(0),
				d_min_symbols(0),
				d_signal_out(false),
        d_num_data_syms(num_data_symbol),
        d_known_norm(0),
        d_data_tones(data_tones),
        d_coarse_freq(0),
        d_nbits(nbits)
{
  //FIXME: dirty assert
  //assert(d_occupied_carriers == d_fft_length);
  assert(d_num_carriers == d_fft_length);
  assert(d_num_carriers == carrier_map.size());
  
  /*
  // !!!!!! NOTE: DON NOT set align, because it will increase scheduler overhead
  //
  //const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
  //set_alignment(std::max(1, alignment_multiple));
  //printf("########%s() volk set_alignment: %d\n", __FUNCTION__, std::max(1, alignment_multiple));
  */

  // initialize carrier map
  for (unsigned int i = 0; i < d_num_carriers; ++i) {
      switch (carrier_map[i]) {
      case 0: d_null_carriers.push_back(i); break;
      case 1: d_data_carriers.push_back(i); break;
      case 2: d_pilot_carriers.push_back(i); break;
      default: throw std::invalid_argument("raw_ofdm_demod: carrier_map must include only 0,1,2");
      }
  }

  assert(d_data_tones == d_data_carriers.size());
  d_occupied_carriers = d_data_carriers.size() + d_pilot_carriers.size();

  d_left_pad = (d_fft_length - (d_occupied_carriers))/2;
  printf("########%s() d_left_pad: %d\n", __FUNCTION__, d_left_pad);

  // d_preamble: LTS (2 syms)
  const std::vector<gr_complex> &first = d_preamble[0];
  d_known_diff.resize(d_num_carriers);
  d_symbol_diff.resize(d_num_carriers);
  for(unsigned i = 0; i < d_num_carriers; i++) {
    d_known_diff[i] = first[i] * conj(first[i]);
    d_known_norm += norm(d_known_diff[i]);
  }

  //d_est_x = new gr_complex[d_num_carriers];
  d_demod = new gr_complex[d_data_tones];

  set_tag_propagation_policy(TPP_DONT);  /* TPP_DONT TPP_ALL_TO_ALL TPP_ONE_TO_ONE */

  printf(">>>>>> %s() d_preamble.size() =%lu\n" , __FUNCTION__, d_preamble.size());


  d_volk_known_symbol = (gr_complex *)volk_malloc(sizeof(gr_complex) * d_num_carriers, volk_get_alignment());
  d_volk_tmp_symbol1 = (gr_complex *)volk_malloc(sizeof(gr_complex) * d_num_carriers, volk_get_alignment());
  d_volk_tmp_symbol2 = (gr_complex *)volk_malloc(sizeof(gr_complex) * d_num_carriers, volk_get_alignment());
  d_hestimate = (gr_complex *)volk_malloc(sizeof(gr_complex) * d_num_carriers, volk_get_alignment());
  d_est_x = (gr_complex *)volk_malloc(sizeof(gr_complex) * d_num_carriers, volk_get_alignment());;
}

raw_ofdm_demod::~raw_ofdm_demod()
{
  delete[] d_est_x;
  delete[] d_demod;

  volk_free(d_volk_known_symbol);
  volk_free(d_volk_tmp_symbol1);
  volk_free(d_volk_tmp_symbol2);
  volk_free(d_hestimate);
  volk_free(d_est_x);
}

/*
static const int LOOKAHEAD = 3;
void
raw_ofdm_demod::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++) {
    //ninput_items_required[i] = 1;
    ninput_items_required[i] = LOOKAHEAD; // NOTE: so that our frame train method works
  }
}
*/


/******************************* Equalization **********************************/
inline int
raw_ofdm_demod::pad() const {
  // amount of FFT padding on the left
  return (d_left_pad + d_coarse_freq);
}

inline gr_complex
raw_ofdm_demod::compensate() const {
  double carrier = (M_TWOPI * d_coarse_freq * d_cur_symbol * d_cplen) / d_fft_length;
  return gr_expj(-carrier);
}

void
raw_ofdm_demod::init_estimate(const gr_complex *symbol)
{
#if 0
  int pad = (d_fft_length - d_occupied_carriers)/2 + d_coarse_freq;

  // set every even tap based on known symbol
  for(int i = 0; i < d_occupied_carriers; i+=2) {
    gr_complex tx = d_known_symbol[i];
    gr_complex rx = symbol[i+pad];
    d_hestimate[i] = tx / rx;
  }

  // linearly interpolate between set carriers to set zero-filled carriers
  for(int i = 1; i < d_occupied_carriers - 1; i+= 2) {
    d_hestimate[i] = (d_hestimate[i-1] + d_hestimate[i+1]) / 2.0f;
  }

  // with even number of carriers; last equalizer tap is wrong
  if(!(d_occupied_carriers & 1)) {
    d_hestimate[d_occupied_carriers-1] = d_hestimate[d_occupied_carriers-2];
  }
#else // do not use the correlation symbol for channel estimation
  for(unsigned int i = 0; i < d_num_carriers; ++i) {
    d_hestimate[i] = 0;
  }
#endif
}

void
raw_ofdm_demod::update_estimate(const gr_complex *symbol)
{
  // take coarse frequency offset into account
  gr_complex comp(1.0,0.0); // = compensate();

  #if 0
  //printf("update_estimate: cur_pos=%d \n",d_cur_symbol+1);
  const std::vector<gr_complex> &known_symbol = d_preamble[d_cur_symbol];

  // set every even tap based on known symbol
  for(unsigned int i = 0; i < d_num_carriers; ++i) {
    gr_complex tx = known_symbol[i];
    gr_complex rx = symbol[i];
    d_hestimate[i] += tx / (rx * comp);

    //printf("%d rx=%f+%fi tx=%f+%fi\n", i, rx.real(), rx.imag(), tx.real(), tx.imag());
  }
  #else
  int i;
  for(i = 0; i < d_num_carriers; ++i) {
    d_volk_known_symbol[i] = d_preamble[d_cur_symbol][i];
  }
  for(i = 0; i < d_null_carriers.size(); ++i) {
    d_volk_known_symbol[d_null_carriers[i]] = 1;
  }
  // d_hestimate[i] += tx / (rx * comp);
  float d_factor[d_num_carriers*2];
  // rxc = rx*comp
  volk_32fc_s32fc_multiply_32fc(d_volk_tmp_symbol1, symbol, comp, d_num_carriers);
  
  volk_32fc_x2_multiply_conjugate_32fc(d_volk_tmp_symbol2, d_volk_tmp_symbol1, d_volk_tmp_symbol1, d_num_carriers);
  for(i = 0; i < d_num_carriers; i++) {
    d_factor[i*2+0] = d_volk_tmp_symbol2[i].real();
    d_factor[i*2+1] = d_volk_tmp_symbol2[i].real();
  }

  // tx/rxc = tx * conj(rxc)/(norm(rxc))
  volk_32fc_x2_multiply_conjugate_32fc(d_volk_tmp_symbol2, d_volk_known_symbol, d_volk_tmp_symbol1, d_num_carriers);

  volk_32f_x2_divide_32f((float *)d_volk_tmp_symbol1, (float *)d_volk_tmp_symbol2, d_factor, d_num_carriers * 2);
  volk_32f_x2_add_32f((float *)d_hestimate, (float *)d_hestimate, (float *)d_volk_tmp_symbol1, d_num_carriers*2);
  #endif


}

void
raw_ofdm_demod::finish_estimate()
{
#if 0
  // just normalize
  unsigned int num_symbols = d_preamble.size(); // the first one does not count
  for(unsigned int i = 0; i < d_num_carriers; ++i) {
    d_hestimate[i] /= num_symbols;;
  }
#else
  float d_factor[d_num_carriers*2];
  float factor = d_preamble.size();
  for(int i = 0; i < d_num_carriers*2; ++i) {
    d_factor[i] = factor;
  }
  volk_32f_x2_divide_32f((float *)d_hestimate, (float *)d_hestimate, d_factor, d_num_carriers*2);
#endif

#if VERBOSE
  float h_abs = 0.0f;
  float h_abs2 = 0.0f;
  float h_arg = 0.0f;
  float h_max = 0.0f;
  for(unsigned int i = 0; i < d_occupied_carriers; ++i) {
    if (i == (d_occupied_carriers+1)/2) {
      // skip the DC
      continue;
    }
    float aa = std::abs(d_hestimate[i]);
    h_abs += aa;
    h_abs2 += aa*aa;
    h_arg += std::arg(d_hestimate[i]); // FIXME: unwrap
    h_max = std::max(h_max, aa);
  }
  h_abs /= d_occupied_carriers-1;
  h_abs2 /= d_occupied_carriers-1;
  h_arg /= d_occupied_carriers-1;
  std::cerr << "H: phase = " << h_arg
            << "\tavg = " << h_abs
            << "\tmax = " << h_max
            << "\tSD = " << h_abs2 - h_abs*h_abs
            << std::endl;

  int p = pad();
  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
    std::cout << i+p << " \t " << d_hestimate[i] << " " << abs(d_hestimate[i])  << std::endl;
  }
#endif
}


float
raw_ofdm_demod::correlate(const gr_complex *symbol, int &coarse_freq)
{
  // Find a frequency offset that maximizes correlation of the input symbol
  // with the known symbol.
  // Due to possible phase offset we cannot take direct correlation.
  // If we assume that the phase offset is linear in frequency then we can
  // correlate the differential of the phase.
  // v = known symbol after FFT
  // x = input symbol after FFT
  // x[k] = v[k] exp(j (ak + b))
  // dv[k] = v[k] v[k+1]*
  // dx[k] = x[k] x[k+1]*
  // corr[k,d] = dv[k] dx[k+d]*
  // d_opt = argmax_d |sum_k corr[k,d]|
  // coarse_freq_offset = d_opt
  // time_offset = a = -mean(arg(corr[k,d]))
  //
  // Here v[2*k+1] = 0, so we change
  // dv[k] = v[k] v[k+2]*
  // dx[k] = x[k] x[k+2]*
  //

  for(unsigned i = 0; i < d_num_carriers; i++) {
    d_symbol_diff[i] = conj(symbol[i]) * symbol[i];
  }

  // sweep through all possible/allowed frequency offsets and select the best
  double best_sum = 0.0;
  unsigned int best_d = 0;

  for(unsigned d = 0; d < d_left_pad; ++d) {
    gr_complex sum = 0.0;
    for(unsigned j = 0; j < d_num_carriers; j++) {
      sum += (d_known_diff[j] * d_symbol_diff[(d+j)%d_num_carriers]);
    }
    
    #if 0
    std::cerr << "d = " << d << "\tM = " << abs(sum) << std::endl;
    #endif
    
    double asum = abs(sum);
    if(asum > best_sum) {
      best_sum = asum;
      best_d = d;
    }
  }
  coarse_freq = best_d;
  


  double norm_sum = 0.0;
  for(unsigned j = 0; j < d_occupied_carriers; j++) {
    norm_sum += norm(d_symbol_diff[(best_d+j)%d_num_carriers]);
  }
  float correlation = best_sum / sqrt(d_known_norm * norm_sum);

  #if 1 //VERBOSE
  std::cerr << "###[ofdm_demod] coarse_freq = " << coarse_freq  << " correlation = " << correlation << std::endl;
  #endif

#if 0
  // nice little trick gives us timing offset (but we don't need it)
  double phase_sum = 0.0;
  for(unsigned j = 0; j < d_occupied_carriers-2; j+= 2) {
    // arg is sort of expensive, but it's the correct thing to do here
    float phase = std::arg(d_known_diff[j] * d_symbol_diff[best_d+j]);
    // FIXME: make sure it doesn't wrap (arg(mean) is the alternative)
    phase_sum += phase;
  }
  //std::cerr << "phase_sum " << phase_sum << std::endl;
  float time_offset = phase_sum / (d_occupied_carriers-2);
  std::cerr << "time_offset = " << time_offset
            << " = " << time_offset * d_fft_length / M_TWOPI
            << " samples" << std::endl;
#endif
  return correlation;
}

/** process one symbol **/
void raw_ofdm_demod::demap2(const gr_complex *in, gr_complex *out,
                            float *noise_out, float *signal_out) {
  // simplfied implementation of RawOFDM: remove dfe, sfo, et al

  gr_complex phase_error = 0.0;

  // update CFO PLL based on pilots
  // TODO: FIXME: 802.11a-1999 p.23 (25) defines p_{0..126v} which is cyclic
  int cur_pilot = 1;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); ++i) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    phase_error += in[di] * conj(pilot_sym);
  }

  float angle = gr::fast_atan2f(phase_error);
  gr_int32 fp_angle = gr::fxpt::float_to_fixed(-angle);
  float oi, oq;
  gr::fxpt::sincos(fp_angle, &oq, &oi);
  gr_complex rotation(oi, oq);

  // estimate SNR in freq domain with pilots
  float noise = 0.0f;
  float signal = 0.0f;
  cur_pilot = 1;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); ++i) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    gr_complex sigeq = in[di] * rotation;
    noise += norm(sigeq - pilot_sym);
    signal += norm(sigeq);
  }

  if (noise_out)
    *noise_out = noise / d_pilot_carriers.size();

  for (unsigned int i = 0; i < d_data_carriers.size(); ++i) {
    int di = d_data_carriers[i];
    out[i] = in[di] * rotation;
    signal += norm(out[i]);
  }

  if (signal_out)
    *signal_out = signal / (d_data_carriers.size()+d_pilot_carriers.size());
}


/******************************* STREAM TAGS **********************************/

void raw_ofdm_demod::get_stream_tags(const int port, const uint64_t nin)
{
  std::vector<gr::tag_t> rx_sync_tags;
  const uint64_t nread = nitems_read(port);        
  uint64_t spos = nread;
  uint64_t epos = nread + nin + 1;

  get_tags_in_range(rx_sync_tags, port, spos, epos, SAMPLER_SU_CFO);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_cfo = pmt::to_double(value);
#if VERBOSE_TAG
    printf("---- [OFDM_DEMOD] Get cfo tag, Range: [%d:%d) | Offset: %d | CFO: %f \n", pos, pos+1, sample_offset, d_sync_cfo);
#endif
    } else {
      printf("---- [OFDM_DEMOD] Preamble received, with no CFO?  Range: [%lu:%lu) \n", spos, epos);
    }

  this->get_tags_in_range(rx_sync_tags, port, spos, epos, SAMPLER_HW_TIME);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_hw_secs = pmt::to_uint64(pmt::tuple_ref(value,0));
    d_sync_hw_frac = pmt::to_double(pmt::tuple_ref(value,1));
#if VERBOSE_TAG
    printf("---- [OFDM_DEMOD] Get SYNC_HW_TIME tag, Range: [%d:%d) | Offset: %d | Time: %f \n", spos, epos, sample_offset, d_sync_hw_secs + d_sync_hw_frac);
#endif
  } else {
    printf("---- [OFDM_DEMOD] Preamble received, with no HW time?  Range: [%lu:%lu) \n", spos, epos);
  }

  this->get_tags_in_range(rx_sync_tags, port, spos, epos, SAMPLER_PC_TIME);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_pc_secs = pmt::to_uint64(pmt::tuple_ref(value,0));
    d_sync_pc_frac = pmt::to_double(pmt::tuple_ref(value,1));
#if VERBOSE_TAG
    printf("---- [OFDM_DEMOD] Get pc time tag, Range: [%d:%d) | Offset: %d | Time: %f \n", pos, pos+1, sample_offset, d_sync_pc_secs + d_sync_pc_frac);
#endif
  } else {
    printf("---- [OFDM_DEMOD] Preamble received, with no PC time?  Range: [%lu:%lu) \n", spos, epos);
  }

  this->get_tags_in_range(rx_sync_tags, port, spos, epos, PKT_DECODE_TIME);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_decode_time_secs = pmt::to_uint64(pmt::tuple_ref(value,0));
    d_decode_time_frac = pmt::to_double(pmt::tuple_ref(value,1));
  } else {
    printf("---- [OFDM_DEMOD] Preamble received, with no DECODE time?  Range: [%lu:%lu) \n", spos, epos);
  }

}

void raw_ofdm_demod::get_stream_tags2(const int port, const uint64_t nin)
{
  std::vector<gr::tag_t> rx_sync_tags;
  const uint64_t nread = nitems_read(port);        
  uint64_t spos = nread;
  uint64_t epos = nread + nin + 1;

  d_sync_snr = 0;
  this->get_tags_in_range(rx_sync_tags, port, spos, epos, SYNC_SU_SNR);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_snr = pmt::to_double(pmt::nth(0, value));
  } else {
    printf("---- [OFDM_DEMOD] Preamble received, with no SYNC_SU_SNR?  Range: [%lu:%lu) \n", spos, epos);
  }
}

void raw_ofdm_demod::write_stream_tags(int port, uint64_t nout)
{
  uint64_t nwritten_pos = nitems_written(port)+nout;
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
  const pmt::pmt_t sync_hw_time = pmt::make_tuple(
    pmt::from_uint64(d_sync_hw_secs),       // FPGA clock in seconds that we found the sync
    pmt::from_double(d_sync_hw_frac)        // FPGA clock in fractional seconds that we found the sync
  );
  const pmt::pmt_t sync_pc_time = pmt::make_tuple(
    pmt::from_uint64(d_sync_pc_secs),       // PC clock in seconds that we found the sync
    pmt::from_double(d_sync_pc_frac)        // PC clock in fractional seconds that we found the sync
  );
  add_item_tag(port, nwritten_pos, SYNC_HW_TIME, sync_hw_time, _id);
  add_item_tag(port, nwritten_pos, SYNC_PC_TIME, sync_pc_time, _id);

  const pmt::pmt_t cfo_val = pmt::from_double(d_sync_cfo);
  add_item_tag(port, nwritten_pos, SYNC_SU_CFO, cfo_val, _id);

  const pmt::pmt_t snr_val = pmt::list1(pmt::from_double(d_sync_snr));
  add_item_tag(port, nwritten_pos, SYNC_SU_SNR, snr_val, _id);
  /*
  const pmt::pmt_t cfo_vals = pmt::list2(
    pmt::from_double(d_sync_cfo),          // CFO of user A
    pmt::from_double(0)                    // CFO of user B
  );
  add_item_tag(port, nwritten_pos, SYNC_MU_CFO, cfo_vals, _id);
  */

  const pmt::pmt_t decode_time = pmt::make_tuple( pmt::from_uint64(d_decode_time_secs),
                                                  pmt::from_double(d_decode_time_frac));
  add_item_tag(port, nwritten_pos, PKT_DECODE_TIME, decode_time, _id);

  // we calculate snr_fd in raw.ofdm_sampler
  //d_snr_fd = 10*log10(d_freq_signal/d_freq_noise-1);
  //const pmt::pmt_t snr_val = pmt::pmt_list(pmt::pmt_from_double(d_snr_fd));
  //add_item_tag(port, nwritten_pos, SNR_SU_KEYS, snr_val, _id);

#if VERBOSE_TAG
  printf("---- [OFDM_DEMOD] Add SYNC_SU_CFO  tag, pos: %d  | CFO: %f \n", nwritten_pos, d_sync_cfo);
  printf("---- [OFDM_DEMOD] Add SYNC_HW_TIME tag, pos: %d | TIME: %f \n", nwritten_pos, d_sync_hw_secs + d_sync_hw_frac);
  printf("---- [OFDM_DEMOD] Add SYNC_PC_TIME tag, pos: %d | TIME: %f \n", nwritten_pos, d_sync_pc_secs + d_sync_pc_frac);
  //printf("---- [OFDM_DEMOD] Add SNR_SU_KEYS  tag, pos: %d  | SNR_FD: %f | s: %f n: %f \n", nwritten_pos, d_snr_fd, d_freq_signal, d_freq_noise);
#endif
}



/******************************* WORK ******* **********************************/
// RAW_OFDM_DEMOD BLOCK
// INPUT:
//   [IN0] signal: complex*fft_length | [IN1] packet/peak vector
// OUTPUT:
//   [OUT0] signal:  complex*data_tones    | [OUT1] SNR
int raw_ofdm_demod::general_work( int noutput_items,
                                    gr_vector_int &ninput_items,
                                    gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items)
{
  const gr_complex *in = (const gr_complex *)input_items[0];
  const char *signal_in = (const char *)input_items[1];

  unsigned char *out = (unsigned char *)output_items[0];
  char *signal_out = (char *) output_items[1];

  int ninput = (ninput_items[0]<ninput_items[1]) ? ninput_items[0] : ninput_items[1];
  int nout = 0;
  int nin = 0;

  bool newframe = false;

  while((nin < ninput) && (nout < noutput_items)) {
    // first, try to determine if new frame
    int coarse_freq = 0;

    newframe = false;
    // regular mode
    if (*signal_in) {
      // we do not estimate integer CFO, so ignore correlation
      //float corr = correlate(in, coarse_freq);
      // newframe = (corr > 0.7f); // TODO: choose proper correlation threshold
      newframe = true;
    
      d_sync_hw_secs = 0; d_sync_hw_frac = 0;
      d_sync_pc_secs = 0; d_sync_pc_frac = 0;
      d_sync_cfo = 0;
    
      int port = 1;
      get_stream_tags(port, nin);
      write_stream_tags(port, nout);
    }

    if (newframe) {
      d_coarse_freq = coarse_freq;
      d_cur_symbol = 0;
      d_signal_out = true;
      d_in_pkt = true;
      d_freq_noise = 0;
      d_freq_signal = 0;
      init_estimate(in);
    }

    //printf("\t [demod] d_cur_symbol =%d/%d sig=%hhd, ninput=%d[%d %d]\n", d_cur_symbol, d_num_data_syms+d_preamble.size(),  *signal_in, ninput, ninput_items[0], ninput_items[1]);


    if(d_in_pkt) { 
      if (d_cur_symbol < d_preamble.size()) {
        //std::cout << ">>>>>>>>> update_estimate d_cur_symbol=" << d_cur_symbol <<std::endl;
        // use for equalization

        #if DEBUG_COMPUTE_TIME
        struct timeval tv_s;
        gettimeofday(&tv_s, NULL);
        #endif
        update_estimate(in);
        #if DEBUG_COMPUTE_TIME
        struct timeval tv_e;
        gettimeofday(&tv_e, NULL);
        printf("%s time=%.06fus\n", __FUNCTION__, ((tv_e.tv_sec*1e6 + tv_e.tv_usec)-(tv_s.tv_sec*1e6 + tv_s.tv_usec)));
        #endif

        ++nin;
        ++signal_in;
        in += d_fft_length;
        ++d_cur_symbol;

        if (d_cur_symbol == d_preamble.size()) {
          int port = 1;
          get_stream_tags2(port, nin);
        }

        continue;
      }

      // time to produce
      if (d_cur_symbol == d_preamble.size()) {
        finish_estimate();
      }

	    if(d_cur_symbol == d_num_data_syms+d_preamble.size()) {
        int port = 1;
        //write_stream_tags(port, nout);
	    }
     if(d_cur_symbol == d_num_data_syms+d_preamble.size()) {
      //std::cerr << " Pkt Finished " << std::endl;
        d_in_pkt = false;
     }

     //std::cout << ">>>>>>>>> decoding d_cur_ymbol=" << d_cur_symbol <<std::endl;

      *signal_out = 0;
      if (d_signal_out) {
        *signal_out = 1;
        d_signal_out = false;
      }

      int p = pad();
      gr_complex comp(1.0,0.0); // = compensate();  // lzyou: disable interger CFO compensation
        #if DEBUG_COMPUTE_TIME
        struct timeval tv_s;
        gettimeofday(&tv_s, NULL);
        #endif

      #if 0
      for(unsigned int i = 0; i < d_num_carriers; i++) {
        d_est_x[i] = d_hestimate[i] * comp * in[i];
      }
      #else
      volk_32fc_s32fc_multiply_32fc(d_volk_tmp_symbol1, d_hestimate, comp, d_num_carriers);
      volk_32fc_x2_multiply_32fc(d_est_x, d_volk_tmp_symbol1, (gr_complex *)in, d_num_carriers);
      #endif
        #if DEBUG_COMPUTE_TIME
        struct timeval tv_e;
        gettimeofday(&tv_e, NULL);
        printf("%s equalization time=%.06fus\n", __FUNCTION__, ((tv_e.tv_sec*1e6 + tv_e.tv_usec)-(tv_s.tv_sec*1e6 + tv_s.tv_usec)));
        #endif


      // estimate the residual phase based on pilots, and then compensate
      float noise_out=0.0f, demap_signal_out=0.0f;
      demap2(d_est_x, d_demod, &noise_out, &demap_signal_out);
      d_freq_noise += noise_out;
      d_freq_signal += demap_signal_out;

      // demodulation
      float *demod = (float *)d_demod;

      switch(d_nbits) {
        case 1: { // BPSK
          QAM<1> q(1.0f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*demod, out); demod+= 2; out+= 1;
            //printf("d_demod=%.04f, out=%02hhx \n", *(demod-2), *(out-1));
          }
        } break;
        case 2: { // QPSK
          QAM<1> q(0.5f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*demod, out); demod+= 1; out+= 1;
            q.decode(*demod, out); demod+= 1; out+= 1;
          }
        } break;
        case 4: { // QAM16
          QAM<2> q(0.5f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*demod, out); demod+= 1; out+= 2;
            q.decode(*demod, out); demod+= 1; out+= 2;
          }
        } break;
        case 6: { // QAM64
          QAM<3> q(0.5f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*demod, out); demod+= 1; out+= 3;
            q.decode(*demod, out); demod+= 1; out+= 3;
          }
        } break;
      }
      

      ++signal_out;
      ++nout;
    }

    ++nin;
    ++signal_in;
    in += d_fft_length;
    ++d_cur_symbol;

    
    //std::cerr << " d_cur_symbol = " << d_cur_symbol << std::endl;
    

    //std::cerr << nin << " : " << ninput << " : " << nout << " : " << noutput_items << std::endl;
  }

  consume_each(nin);
  #if 0
  std::cerr << "num symbols " << (d_cur_symbol+1) << std::endl;
  #endif
  //std::cerr << "===" << nout << " : " << nout << std::endl;

  return nout;
}

