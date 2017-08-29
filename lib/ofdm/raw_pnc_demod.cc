/* -*- c++ -*- */
/*
 * Copyright 2014 You Lizhao
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_pnc_demod.h>
 #include <raw_qam.h>
#include <pnc_tags.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/fxpt.h>
#include <gnuradio/sincos.h>
#include <gnuradio/math.h>
#include <iostream>
#include <stdio.h>
#include <sched.h>      // added this to get the time Quantum
#include <boost/lexical_cast.hpp>

#define USE_KNOWN_DATA false
#define SOFT_MAX_VALUE  650
#define NOISE_POWER   0.0000011

#define THRESHOLD 2*log(2)

#define DEBUG               0
#define DEMOD_DEBUG         0
#define USERMODE_DEBUG      0

#define M_TWOPI             (2*M_PI)
#define MAX_VALUE           10000

#define PNC_STS_LENGTH 2
#define PNC_LTS_LENGTH 2
#define A_LTS_INDEX 0+PNC_STS_LENGTH
#define B_LTS_INDEX 1+PNC_STS_LENGTH
#define DATA_SYM_INDEX 2+PNC_STS_LENGTH

raw_pnc_demod_sptr raw_make_pnc_demod(unsigned int fft_length,
                                      unsigned int data_tones,
                                      unsigned int cplen,
                                      const std::vector<std::vector<gr_complex> > &preamble,
                                      std::vector<int> carrier_map,
                                      unsigned int num_data_symbol,
                                      float alpha,
                                      int mode,
                                      int nbits)
{
  return raw_pnc_demod_sptr(new raw_pnc_demod(fft_length,
                                              data_tones,
                                              cplen,
                                              preamble,
                                              carrier_map,
                                              num_data_symbol,
                                              alpha, mode, nbits) );
}

std::vector<int>
get_out_sizeofs(size_t item_size)
{
  std::vector<int> out_sizeofs;
  for(unsigned int i = 0; i < 2; i++) {
    if(i==1)
      out_sizeofs.push_back(sizeof(char));
    else
      out_sizeofs.push_back(sizeof(char)*item_size);
  }
  return out_sizeofs;
} 

// lzyou: we should keep data_tones parameter used in gr_make_io_signature, though it can be set as length(d_data_carriers)
raw_pnc_demod::raw_pnc_demod(unsigned int fft_length,
                             unsigned int data_tones,
                             unsigned int cplen,
                             const std::vector<std::vector<gr_complex> > &preamble,
                             std::vector<int> carrier_map,
                             unsigned int num_data_symbol,
                             float alpha,int mode, int nbits) :
            gr::block("pnc_demod",
                gr::io_signature::make3(2, 3, sizeof(gr_complex) * fft_length,
                        sizeof(char), sizeof(gr_complex) * fft_length),
                gr::io_signature::makev(2, 2, get_out_sizeofs(data_tones))),
//                d_occupied_carriers(preamble[0].size()),
                d_fft_length(fft_length),
                d_data_tones(data_tones),
                d_cplen(cplen),
                d_preamble(preamble),
                d_cur_symbol(0),
                d_min_symbols(0),
                d_cur_frame(0),
                d_signal_out(false),
                d_num_carriers(carrier_map.size()),
                d_num_data_symbol(num_data_symbol),
                d_mode(mode),
                d_alpha2(alpha),
                d_noise_A(0),
                d_signal_A(0),
                d_noise_B(0),
                d_signal_B(0),
                d_nbits(nbits)
{
  //FIXME: dirty assert
  assert(d_num_carriers == 64);
  assert((d_mode==0) || (d_mode==1));

  d_channel_A.resize(d_num_carriers);
  d_channel_B.resize(d_num_carriers);
  d_max_2H = 0;
  d_max_H_A = 0;
  d_max_H_B = 0;
  d_previous_snr = 0;
  d_previous_snr_A = 0;
  d_previous_snr_B = 0;

  d_newframe = false;

  // initialize carrier map
  for (unsigned int i = 0; i < d_num_carriers; ++i) {
      switch (carrier_map[i]) {
      case 0: d_null_carriers.push_back(i); break;
      case 1: d_data_carriers.push_back(i); break;
      case 2: d_pilot_carriers.push_back(i); break;
      default: throw std::invalid_argument("raw_pnc_demod: carrier_map must include only 0,1,2");
      }
  }
  d_occupied_carriers = d_data_carriers.size() + d_pilot_carriers.size() + 1; // including DC

  assert(d_data_tones == d_data_carriers.size());
  assert(d_pilot_carriers.size()%2==0); // assume even number of pilots for two user

  d_demod = new gr_complex[d_data_tones];

  // substract 1 DC
  d_pad = (d_fft_length-(d_occupied_carriers-1))/2;

  printf("%s() d_occupied_carriers=%d\n", __FUNCTION__, d_occupied_carriers);
}

raw_pnc_demod::~raw_pnc_demod(void)
{
  delete[] d_demod;
}

static const int LOOKAHEAD = 1;
void raw_pnc_demod::forecast(int noutput_items,
                    gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size();
  for (unsigned i = 0; i < ninputs; i++)
    //    ninput_items_required[i] = 1;
    ninput_items_required[i] = LOOKAHEAD; // NOTE: so that our frame train method works
}

inline int raw_pnc_demod::pad() const {
  // amount of FFT padding on the left
  return (d_fft_length - d_occupied_carriers + 1) / 2;
}

void raw_pnc_demod::init_estimate(const gr_complex *symbol)
{
  // do not use the correlation symbol for channel estimation
  for (unsigned int i = 0; i < d_num_carriers; ++i) {
      d_channel_A[i] = 0;
      d_channel_B[i] = 0;
  }
  d_max_H_A = 0;
  d_max_H_B = 0;
  d_signal_A = 0;
  d_noise_A  = 0;
  d_signal_B = 0;
  d_noise_B  = 0;

  // new approach to cacl SNR
  d_signal2_A = 0;
  d_signal2_B = 0;
  d_noise = 0;
  d_signal = 0;
}

void raw_pnc_demod::update_estimate_A(const gr_complex *symbol, const gr_complex *symbol_td)
{
  int p = pad();
  const std::vector<gr_complex> &known_symbol = d_preamble[d_cur_symbol];
  d_pre_signal_A = 0;
  //std::cout<<d_cur_symbol<<std::endl;

  if (DEBUG)
    std::cout << " ===== A: Channel Estimation ===== \n";
  for (unsigned int i = 0; i < d_occupied_carriers; ++i) {
      gr_complex tx = known_symbol[i + d_pad];
      gr_complex rx = symbol[i + d_pad];
      d_channel_A[i + d_pad] += rx / tx;
      if (!(isinf(abs(d_channel_A[i + d_pad]))) && (abs(d_channel_A[i + d_pad]) > d_max_H_A))
        d_max_H_A = abs(d_channel_A[i + d_pad]);
      if (DEBUG) {
        std::cout << tx << " \t " << rx << " \t " << i + d_pad  << std::endl;
        //printf(" %f+%f*1j %f+");
      }

      if(symbol_td) {
          d_pre_signal_A += norm(symbol_td[i + d_pad]);
      }
  }
  d_pre_signal_A /= d_occupied_carriers;
  if(DEBUG) {
      printf(" [PNC_DEMOD] User A Signal: %f | Noise = %f | SNR = %10.6f\n", d_pre_signal_A, d_pre_noise, 10*log10(d_pre_signal_A/d_pre_noise-1));
  }
  if (DEBUG) {
      for (unsigned int i = 0; i < d_fft_length; ++i) {
        //std::cout << d_channel_A[i] << std::endl;
        printf(" %f %f \n", real(d_channel_A[i]), imag(d_channel_A[i]));
      }
  }
}

void raw_pnc_demod::update_estimate_B(const gr_complex *symbol, const gr_complex *symbol_td)
{
  int p = pad();
  const std::vector<gr_complex> &known_symbol = d_preamble[d_cur_symbol];
  d_pre_signal_B = 0;

  if (DEBUG)
    std::cout << "\n ===== B: Channel Estimation ===== \n";
  for (unsigned int i = 0; i < d_occupied_carriers; ++i) {
      gr_complex tx = known_symbol[i + d_pad];
      gr_complex rx = symbol[i + d_pad];
      d_channel_B[i + d_pad] += rx / tx;
      if (!(isinf(abs(d_channel_B[i + d_pad]))) && (abs(d_channel_B[i + d_pad]) > d_max_H_B))
        d_max_H_B = abs(d_channel_B[i + d_pad]);
      if (DEBUG)
        std::cout << tx << " \t " << rx << " \t " << i + d_pad << std::endl;
      if(symbol_td) {
          d_pre_signal_B += norm(symbol_td[i + d_pad]);
      }
  }

  d_pre_signal_B /= d_num_carriers;
  if(DEBUG) {
      printf(" [PNC_DEMOD] User B Signal: %f | Noise = %f | SNR = %10.6f \n", d_pre_signal_B, d_pre_noise, 10*log10(d_pre_signal_B/d_pre_noise-1));
  }
  if (DEBUG) {
      for (unsigned int i = 0; i < d_fft_length; ++i) {
        //std::cout << d_channel_B[i] << std::endl;
        printf(" %f %f \n", real(d_channel_B[i]), imag(d_channel_B[i]));
      }
  }
  d_max_2H = 2 * (d_max_H_A + d_max_H_B);

  // use new approach to calculate d_max_H
  d_max_H = calc_H_max();
}

float raw_pnc_demod::calc_H_max()
{
  int p = pad();
  float max_value = 0;
  for(unsigned int i=0; i < d_occupied_carriers; ++i) {
    if(!(isinf(abs(d_channel_A[i + d_pad]))) && !(isinf(abs(d_channel_B[i + d_pad])))) {
      // choose minimum of A and B
      float k_value = abs(d_channel_A[i + d_pad]) > abs(d_channel_B[i + d_pad]) ? abs(d_channel_B[i + d_pad]) : abs(d_channel_A[i + d_pad]);
      // choose max over all subcarriers
      if(k_value > max_value)
        max_value = k_value;
    }
  }
  return max_value;
}

unsigned char raw_pnc_demod::joint_hard_demodulation(const gr_complex in,
        const gr_complex H_A, const gr_complex H_B, unsigned char &A,
        unsigned char &B)
{
  // FIXME: (1) add other modulation schemes; (2) look for implementation with high efficiency
  float min = MAX_VALUE;
  int seq = 0;
  unsigned char v = 128;

  // pair (1,1)
  float d1 = norm( H_A + H_B - in);
  float s  = norm( H_A + H_B);
  min = d1; seq = 1;

  float d2 = norm(-H_A + H_B - in);
  if (d2 < min) {
      s = norm(-H_A + H_B);
      min = d2; seq = 2;
  }

  float d3 = norm( H_A - H_B - in);
  if (d3 < min) {
      s = norm( H_A - H_B);
      min = d3; seq = 3;
  }

  float d4 = norm(-H_A - H_B - in);
  if (d4 < min) {
      s = norm(-H_A - H_B);
      min = d4; seq = 4;
  }

  if (min > s) {
    //printf(" "); printf(": NOISE > SIGNAL \n");
    d_signal += s;
  }

  switch (seq) {
  case 1:
    A = 255; B = 255; v = 0;   break;  // pair (1,1)
  case 2:
    A = 0;   B = 255; v = 255; break;  // pair (-1,1)
  case 3:
    A = 255; B = 0;   v = 255; break;  // pair (1,-1)
  case 4:
    A = 0;   B = 0;   v = 0; break;    // pair (-1,-1)
  default:
    printf(" [PNC DECODING] ERROR: not valid mapping \n");
    break;
  }

  return v;
}

unsigned char raw_pnc_demod::joint_soft_demodulation2(const gr_complex in,
                                                        const gr_complex H_A,
                                                        const gr_complex H_B,
                                                        unsigned char& A,
                                                        unsigned char& B) 
{
  // Algorithm defined in NCMA paper, should be the same with joint_soft_demodulation
  // NOTE: the mapping is different from what paper described

  float d1 = norm( H_A + H_B - in);     // (1,1)
  float d2 = norm(-H_A + H_B - in);     // (-1,1)
  float d3 = norm( H_A - H_B - in);     // (1,-1)
  float d4 = norm(-H_A - H_B - in);     // (-1,-1)

  /*
  float d1 = norm(-H_A - H_B - in);     // (1,1)
  float d2 = norm( H_A - H_B - in);     // (-1,1)
  float d3 = norm( H_A + H_B - in);     // (1,-1)
  float d4 = norm( H_A + H_B - in);     // (-1,-1)
  */

  // ------------------------------------------------------------------- //
  //                             User A                                     //
  // ------------------------------------------------------------------- //
  float softA = 0;
  float NA = 0;
  if((d1>d3)&&(d2>d4)) {                // pick (1,-1) and (-1,-1)
    softA = real(H_A)*real(in+H_B)+imag(H_A)*imag(in+H_B);
    NA = sqrt(norm(H_A));
  }
  else if((d1>d3)&&(d2<=d4)) {            // pick (1,-1) and (-1,1)
    softA = real(in)*real(H_A-H_B)+imag(in)*imag(H_A-H_B);
    NA = sqrt(norm(H_A-H_B));
  }
  else if((d1<=d3)&&(d2>d4)) {            // pick (1,1) and (-1,-1)
    softA = real(in)*real(H_A+H_B)+imag(in)*imag(H_A+H_B);
    NA = sqrt(norm(H_A+H_B));
  }
  else if((d1<=d3)&&(d2<=d4)) {            // pick (1,1) and (-1,1)
    softA = real(H_A)*real(in-H_B)+imag(H_A)*imag(in-H_B);
    NA = sqrt(norm(H_A));
  }
  NA = d_max_H_A;
  int vA = round(((softA/NA) * d_alpha2 / d_max_H_A + 0.5)*255);
  A = (vA < 0) ? 0 : ((vA > 255) ? 255 : vA);
  // ------------------------------------------------------------------- //

  // ------------------------------------------------------------------- //
  //                             User B                                     //
  // ------------------------------------------------------------------- //
  float softB = 0;
  float NB = 0;
  if((d1>d2)&&(d3>d4)) {                // pick (-1,1) and (-1,-1)
    softB = real(H_B)*real(in+H_A)+imag(H_B)*imag(in+H_A);
    NB = sqrt(norm(H_B));
  }
  else if((d1>d2)&&(d3<=d4)) {            // pick (-1,1) and (1,-1)
    softB = real(in)*real(H_B-H_A)+imag(in)*imag(H_B-H_A);
    NB = sqrt(norm(H_A-H_B));
  }
  else if((d1<=d2)&&(d3>d4)) {            // pick (1,1) and (-1,-1)
    softB = real(in)*real(H_A+H_B)+imag(in)*imag(H_A+H_B);
    NB = sqrt(norm(H_A+H_B));
  }
  else if((d1<=d2)&&(d3<=d4)) {            // pick (1,1) and (1,-1)
    softB = real(H_B)*real(in-H_A)+imag(H_B)*imag(in-H_A);
    NB = sqrt(norm(H_B));
  }
  NB = d_max_H_B;
  int vB = round(((softB/NB) * d_alpha2 / d_max_H_B + 0.5)*255);
  B = (vB < 0) ? 0 : ((vB > 255) ? 255 : vB);
  // ------------------------------------------------------------------- //

  // ------------------------------------------------------------------- //
  //                             User XOR                                 //
  // ------------------------------------------------------------------- //
  float soft = 0;
  float N = 0;
  int d_state=0;
  if ((d1>d4) && (d2>d3)) {             // pick (-1,-1) and (1,-1)
    soft = 1*(real(H_A)*real(in+H_B)+imag(H_A)*imag(in+H_B));
    N = sqrt(norm(H_A));
    d_state = 4;
  }
  else if((d1>d4) && (d2<=d3)) {         // pick (-1,-1) and (-1,1)
    soft = 1*(real(H_B)*real(in+H_A)+imag(H_B)*imag(in+H_A));
    N = sqrt(norm(H_B));
    d_state = 2;
  }
  else if((d1<=d4) && (d2>d3)) {         // pick (1,1) and (1,-1)
    soft = -1*(real(H_B)*real(in-H_A)+imag(H_B)*imag(in-H_A));
    N = sqrt(norm(H_B));
    d_state = 3;
  }
  else if((d1<=d4) && (d2<=d3)) {         // pick (1,1) and (-1,1)
    soft = -1*(real(H_A)*real(in-H_B)+imag(H_A)*imag(in-H_B));
    N = sqrt(norm(H_A));
    d_state = 1;
  }
  N = d_max_H;
  int v = round(((soft/N) * d_alpha2 / d_max_H + 0.5)*255);
  v = (v < 0) ? 0 : ((v > 255) ? 255 : v);
  // ------------------------------------------------------------------- //

  //printf(" d_state=%d | Soft2: vA=%d vB=%d v=%d Dnm=%f(%f/%f) d=(%f,%f,%f,%f) d_alpha2=%f d_max_H=%f\n", d_state, vA, vB, v, soft/N,soft,N, d1,d2,d3,d4, d_alpha2, d_max_H);
  //printf(" Soft2: A=%d(softA=%f NA=%f) B=%d(softB=%f NB=%f) X=%d(soft=%f N=%f)\n",A,softA,NA,B,softB,NB,v,soft,N);
  return v;
}

gr_complex raw_pnc_demod::get_phase_complex(const gr_complex in)
{
  // NOTE1: sincos_f and sincos are the same, but gr_fxpt::sincos is faster.
  // NOTE2: angle and gr_fast_atan2f(rotation) are slightly different, in order 10^-3
  float angle = gr::fast_atan2f(in);
  gr_int32 fp_angle = gr::fxpt::float_to_fixed(angle);

  float oi, oq; 
  gr::sincosf(angle, &oq, &oi);
  gr_complex rotation2(oi, oq);
  gr::fxpt::sincos(fp_angle, &oq, &oi);
  gr_complex rotation(oi, oq);
  //printf(" %10.6f %10.6f %10.6f \n", angle, gr_fast_atan2f(rotation), gr_fast_atan2f(rotation2));
  return rotation;
}

gr_complex raw_pnc_demod::get_phase_complex(const std::vector<gr_complex> in)
{
  float angle = 0.0;
  for(int i=0; i<in.size(); i++) {
    angle += gr::fast_atan2f(in[i]);
  }
  angle /= in.size();
  gr_int32 fp_angle = gr::fxpt::float_to_fixed(angle);

  float oi, oq; 
  gr::fxpt::sincos(fp_angle, &oq, &oi);
  gr_complex rotation(oi, oq);
  return rotation;
}

void raw_pnc_demod::update_channel_estimation(const gr_complex rotation_A, const gr_complex rotation_B)
{
  for(int i=0; i<d_channel_A.size(); i++)
    d_channel_A[i] *= rotation_A;
  for(int i=0; i<d_channel_B.size(); i++)
    d_channel_B[i] *= rotation_B;
}

void raw_pnc_demod::get_stream_tags(const int port, const uint64_t nconsumed)
{
  std::vector<gr::tag_t> rx_sync_tags;
  const uint64_t nread = nitems_read(port);        
  int spos = nread;
  int epos = nread + nconsumed + 1;

  this->get_tags_in_range(rx_sync_tags, port, spos, epos, SAMPLER_MU_CFO);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_cfo1 = pmt::to_double(pmt::nth(0,value));
    d_sync_cfo2 = pmt::to_double(pmt::nth(1,value));
#if VERBOSE_TAG
    printf("---- [PNC_DEMOD] Get cfo tag, Range: [%d:%d) | Offset: %d | CFO: %f \n", spos, epos, sample_offset, d_sync_cfo);
#endif
  }
    // FIXME: if in TWO_USER MODE, open this output
    /* else {
      printf("---- [PNC_DEMOD] Preamble received, with no CFO?  Range: [%d:%d) \n", spos, epos);
    }*/

  this->get_tags_in_range(rx_sync_tags, port, spos, epos, SAMPLER_HW_TIME);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_hw_secs = pmt::to_uint64(tuple_ref(value,0));
    d_sync_hw_frac = pmt::to_double(tuple_ref(value,1));
#if VERBOSE_TAG
    printf("---- [PNC_DEMOD] Get hw time tag, Range: [%d:%d) | Offset: %d | Time: %f \n", spos, epos, sample_offset, d_sync_hw_secs + d_sync_hw_frac);
#endif
  } else {
    printf("---- [PNC_DEMOD] Preamble received, with no HW time?  Range: [%d:%d) \n", spos, epos);
  }

  this->get_tags_in_range(rx_sync_tags, port, spos, epos, SAMPLER_PC_TIME);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_pc_secs = pmt::to_uint64(tuple_ref(value,0));
    d_sync_pc_frac = pmt::to_double(tuple_ref(value,1));
#if VERBOSE_TAG
    printf("---- [PNC_DEMOD] Get pc time tag, Range: [%d:%d) | Offset: %d | Time: %f \n", spos, epos, sample_offset, d_sync_pc_secs + d_sync_pc_frac);
#endif
  } else {
    printf("---- [PNC_DEMOD] Preamble received, with no PC time?  Range: [%d:%d) \n", spos, epos);
  }
}

void raw_pnc_demod::write_stream_tags(int port, const uint64_t nproduced)
{
  uint64_t nwritten_pos = nitems_written(port)+nproduced;
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
  const pmt::pmt_t sync_hw_time = pmt::make_tuple(
    pmt::from_uint64(d_sync_hw_secs),       // FPGA clock in seconds that we found the sync
    pmt::from_double(d_sync_hw_frac)        // FPGA clock in fractional seconds that we found the sync
  );
  const pmt::pmt_t sync_pc_time = pmt::make_tuple(
    pmt::from_uint64(d_sync_pc_secs),       // PC clock in seconds that we found the sync
    pmt::from_double(d_sync_pc_frac)        // PC clock in fractional seconds that we found the sync
  );
  const pmt::pmt_t cfo_vals = pmt::list2(
    pmt::from_double(d_sync_cfo1),          // CFO of user A
    pmt::from_double(d_sync_cfo2)           // CFO of user B
  );
  add_item_tag(port, nwritten_pos, SYNC_HW_TIME, sync_hw_time, _id);
  add_item_tag(port, nwritten_pos, SYNC_PC_TIME, sync_pc_time, _id);
  add_item_tag(port, nwritten_pos, SYNC_MU_CFO, cfo_vals, _id);

#if VERBOSE_TAG
  printf("---- [PNC_DEMOD] Add SYNC_MU_CFO tags, pos: %d | CFO:  %f \n", nwritten_pos, d_sync_cfo);
  printf("---- [PNC_DEMOD] Add SYNC_HW_TIME tag, pos: %d | TIME: %f \n", nwritten_pos, d_sync_hw_secs + d_sync_hw_frac);
  printf("---- [PNC_DEMOD] Add SYNC_PC_TIME tag, pos: %d | TIME: %f \n", nwritten_pos, d_sync_pc_secs + d_sync_pc_frac);
#endif
}



void raw_pnc_demod::qam_decode(const float *demod, const unsigned char *out)
{
        float *pdemod = (float *)demod;
        unsigned char *pout = (unsigned char *)out;
        switch(d_nbits) {
        case 1: { // BPSK
          QAM<1> q(1.0f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*pdemod, pout); pdemod+= 2; pout+=1;
            //printf("d_demod=%.04f, out=%02hhx \n", *(demod-2), *(out-1));
          }
        } break;
        case 2: { // QPSK
          QAM<1> q(0.5f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*pdemod, pout); pdemod+= 1; pout+=1;
            q.decode(*pdemod, pout); pdemod+= 1; pout+= 1;
          }
        } break;
        case 4: { // QAM16
          QAM<2> q(0.5f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*pdemod, pout); pdemod+= 1; pout+= 2;
            q.decode(*pdemod, pout); pdemod+= 1; pout+= 2;
          }
        } break;
        case 6: { // QAM64
          QAM<3> q(0.5f);
          for (int i = 0; i < d_data_tones; ++i) {
            q.decode(*pdemod, pout); pdemod+= 1; pout+= 3;
            q.decode(*pdemod, pout); pdemod+= 1; pout+= 3;
          }
        } break;
      }

}



// RAW_PNC_DEMOD BLOCK
// INPUT:
//   [IN0] signal: complex*fft_length | [IN1] packet/peak vector
// OUTPUT:
//   [OUT0] bits:  char*data_tones    | [OUT1] SNR
int raw_pnc_demod::general_work( int noutput_items,
                                    gr_vector_int &ninput_items,
                                    gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items)
{
  const gr_complex *symbol = (const gr_complex *) input_items[0];
  const char *signal_in = (const char *) input_items[1];
  const gr_complex *symbol_td  = (input_items.size() > 2) ? (const gr_complex *) input_items[2] : NULL;

  unsigned char *out = (unsigned char *) output_items[0];
  unsigned char *flag = (unsigned char *) output_items[1];

  int ninput = std::min(ninput_items[0], ninput_items[1]);
  int nproduced = 0;
  int nconsumed = 0;

  while ((nconsumed < ninput) && (nproduced < noutput_items)) {
      //bool newframe = false;

      // regular mode
      if (*signal_in) {
          d_newframe = true;

          d_sync_hw_secs = 0; d_sync_hw_frac = 0;
          d_sync_pc_secs = 0; d_sync_pc_frac = 0;
          d_sync_cfo1 = 0; d_sync_cfo2 = 0;
          d_sync_snrA = 0; d_sync_snrB = 0;

          d_user_mode = signal_in[0];
          #if USERMODE_DEBUG
          printf(" | PNC_DEMOD | signal_in[0] = %hhu [%d-USER_A_MODE, %d-USER_B_MODE, %d-TWO_USER_MODE]\n", signal_in[0], USER_A_MODE, USER_B_MODE, TWO_USER_MODE); 
          #endif


          int port = 1;
          get_stream_tags(port, nconsumed);
          write_stream_tags(port, nproduced);
          //get_write_stream_tags(in_port, out_port, nconsumed, nproduced);

      }


      if (d_newframe) {
          d_cur_symbol = 0;
          d_signal_out = true;
          // if we find a new frame, do nothing just initialize
          d_cur_symbol = 0;
          d_signal_out = true;
          init_estimate(symbol);
          //printf(" A_LTS_INDEX = %d B_LTS_INDEX = %d DATA_SYM_INDEX = %d \n", A_LTS_INDEX, B_LTS_INDEX, DATA_SYM_INDEX);
          d_newframe = false;
      }

      if (d_cur_symbol == 0) {
          update_estimate_A(symbol, symbol_td);

          ++nconsumed;
          ++signal_in;
          symbol += d_fft_length;
          if(symbol_td) symbol_td += d_fft_length;
          d_cur_symbol++;
          continue;
      }

      if (d_cur_symbol == 1) {
          update_estimate_B(symbol, symbol_td);

          ++nconsumed;
          ++signal_in;
          symbol += d_fft_length;
          if(symbol_td) symbol_td += d_fft_length;
          d_cur_symbol++;
          continue;
      }

      //printf("pnc_demod d_cur_symbol=%d\n", d_cur_symbol);

      *flag = 0;
      if ((d_signal_out) && (d_cur_symbol >= 2)) {
         if (d_cur_symbol == 2) {
            *flag = d_user_mode; //1
         }

         // calculate the residual phase, and update channel estimation based on pilots (for each symbol)
         // assumption: 1) pilot is always pilot_sym; 2) user A/B's pilots are in sequence;
          int cur_pilot = 1;
          gr_complex pilot_sym(cur_pilot, 0.0), rotate;
          gr_int32 fp_angle;
          float oi, oq;
          for(unsigned int i=0; i<d_pilot_carriers.size(); i++) {
              int pos = d_pilot_carriers[i];

              switch(i%2) {
              case 0:
                rotate = symbol[pos] / (d_channel_A[pos]*pilot_sym);
                d_pilot_rotate_A.push_back(rotate);
                break;
              case 1:
                rotate = symbol[pos] / (d_channel_B[pos]*pilot_sym);
                d_pilot_rotate_B.push_back(rotate);
                break;
              default:
                throw std::invalid_argument("raw_pnc_demod: pilot index mod must include only 0,1\n");
              }
          }

          gr_complex rotation_A, rotation_B;
          // use avg of pilots' phase
          rotation_A = get_phase_complex(d_pilot_rotate_A);
          rotation_B = get_phase_complex(d_pilot_rotate_B);

          // debug output
          if(d_cur_symbol <= 0) {
            printf("A: %10.6f %10.6f %10.6f | B: %10.6f %10.6f %10.6f \n", gr::fast_atan2f(rotation_A), gr::fast_atan2f(d_pilot_rotate_A[0]), gr::fast_atan2f(d_pilot_rotate_A[1]), gr::fast_atan2f(rotation_B), gr::fast_atan2f(d_pilot_rotate_B[0]), gr::fast_atan2f(d_pilot_rotate_B[1]));
          }

          d_pilot_rotate_A.clear();
          d_pilot_rotate_B.clear();

          // Calc SNR using pilots
          for(unsigned int i=0; i<d_pilot_carriers.size(); i++) {
              int pos = d_pilot_carriers[i];

              switch(i%2) {
              case 0:
                d_signal_A += norm(symbol[pos]);
                d_noise_A += norm(symbol[pos]-(d_channel_A[pos]*rotation_A*pilot_sym));
                break;
              case 1:
                d_signal_B += norm(symbol[pos]);
                d_noise_B += norm(symbol[pos]-(d_channel_B[pos]*rotation_B*pilot_sym));
                break;
              default:
                throw std::invalid_argument("raw_pnc_demod: pilot index mod must include only 0,1\n");
              }
          }

          // process one symbol per time
          switch(d_user_mode) {

            case TWO_USER_MODE: {
              unsigned char A, B;
              for (unsigned int i = 0; i < d_data_tones; i++) {
                int di = d_data_carriers[i];
                d_cur_carrier = di;
                //printf("   d_symbol=%d; d_carrier=%d; d_txSymA=%f+1j*%f; d_txSymB=%f+1j*%f; H_A=%f+1j*%f; H_B=%f+1j*%f; in=%f+1j*%f; \n",d_cur_symbol,d_cur_carrier,d_txSymA.real(),d_txSymA.imag(),d_txSymB.real(),d_txSymB.imag(),real(H_A),imag(H_A),real(H_B),imag(H_B),real(in),imag(in));
                //printf(" %f+1j*%f; \n", symbol[di].real(), symbol[di].imag());

                if (d_mode != 0) {
                  unsigned int soft2 = joint_soft_demodulation2(symbol[di], d_channel_A[di]*rotation_A, d_channel_B[di]*rotation_B, A, B);
                  out[i] = soft2;
                    //unsigned int exact=joint_exact_demodulation(symbol[di], d_channel_A[di]*rotation_A, d_channel_B[di]*rotation_B, A, B);
                    //out[i] = exact;
                    //printf(" out_soft[i]=%3d out_exact[i]=%3d out_exact1[i]=%3d\n", soft, exact, exact1);
                    //printf(" %3d %3d %3d\n", soft, exact, exact1);
                }
                else if (d_mode == 0) {
                  //printf(" %f %f %f %f %f %f %f %f %f %f \n",real(symbol[di]),imag(symbol[di]),real(d_channel_A[di]),imag(d_channel_A[di]),
                  //      real(rotation_A),imag(rotation_A),real(d_channel_B[di]),imag(d_channel_B[di]),real(rotation_B),imag(rotation_B));
                  out[i] = joint_hard_demodulation(symbol[di], d_channel_A[di]*rotation_A, d_channel_B[di]*rotation_B, A, B);
                }


                if (DEMOD_DEBUG) {
                  const char *pnc_mode = (d_mode==0)? "Hard:" : "Soft:";
                  std::cout << pnc_mode << d_cur_symbol << ": " << i << ": " << symbol[di]
                                             << " " << d_channel_A[di]
                                             << " " << d_channel_B[di]
                                             << " " << d_pilot_rotate_A[0]
                                             << " " << d_pilot_rotate_B[0]
                                             << " " << int(out[i])
                                             << std::endl;
                }
              }
            } break;

            case USER_A_MODE: {
              int di = 0;
              for(int i = 0; i < d_data_tones; ++i) {
                di = d_data_carriers[i];
                d_demod[i] = symbol[di] /(d_channel_A[di] * rotation_A);
              }
              qam_decode((float *)d_demod, out);
            } break;
            case USER_B_MODE: {
              int di = 0;
              for(int i = 0; i < d_data_tones; ++i) {
                di = d_data_carriers[i];
                d_demod[i] = symbol[di] /(d_channel_B[di] * rotation_B);
              }
              qam_decode((float *)d_demod, out);
            } break;
            default:
              std::cerr << "pnc_demod Unsupported user mode" << std::endl;
              break;
          }

          // after each sym, update d_channel_A and d_channel_B
          for(int i=0; i<d_channel_A.size(); i++) {
            d_channel_A[i] *= rotation_A;
          }
          for(int i=0; i<d_channel_B.size(); i++) {
            d_channel_B[i] *= rotation_B;
          }

          // after each sym, update pointers
          flag++;
          out += d_data_tones;
          ++nproduced;

          if(false) {
            float noise = (d_noise_A+d_noise_B)/2;
            float SNR_A = 10*log10(d_signal_A/noise - 1);
            float SNR_B = 10*log10(d_signal_B/noise - 1);
            float symSNR_A = 10*log10(d_signal2_A/d_noise - 1);
            float symSNR_B = 10*log10(d_signal2_B/d_noise - 1);
            printf("d_cur_sym: %d | SNR: %f %f | Power: %f %f %f | SNR: %f %f | Power: %f %f %f %f\n", d_cur_symbol, symSNR_A, symSNR_B, d_signal2_A, d_signal2_B, d_noise, SNR_A, SNR_B, d_signal_A, d_noise_A, d_signal_B, d_noise_B);
          }

          // lzyou: we have two long training symbol
          if (d_cur_symbol + 1 == d_num_data_symbol + PNC_LTS_LENGTH) {
              d_signal_out = false;
              d_previous_snr_A = d_signal_A/d_noise_A;
              d_previous_snr_B = d_signal_B/d_noise_B;
              d_previous_snr = (d_previous_snr_A+d_previous_snr_B)/2;

              int tag_out_port = 1;
              //add_power_tag(tag_out_port,nproduced);
              //add_snr_tag(tag_out_port,nproduced);
              d_pre_signal_A = d_pre_signal_B = 0;
          }

          d_cur_symbol++;
      }

      ++nconsumed;
      ++signal_in;
      symbol += d_fft_length;
      if(symbol_td) { 
        symbol_td += d_fft_length;
      }
  }

  consume_each(nconsumed);
  return nproduced;
}

