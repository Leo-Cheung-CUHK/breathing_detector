/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_ofdm_sampler.h>
#include <pnc_tags.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/expj.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <boost/thread.hpp>
#include <sys/time.h>

timeval sampler_timer;

#define SAMPLER_DEBUG 0

// START_INDEX is used to avoid unexpected rise up (caused by hardware) at the beginning
#define START_INDEX   2000000
#define NOISE_COUNT   1000000

raw_ofdm_sampler_sptr
raw_make_ofdm_sampler (unsigned int fft_length,
                       unsigned int symbol_length,
                       unsigned int bandwidth,
                       unsigned int timeout,
                       bool debug)
{
  return raw_ofdm_sampler_sptr (
    new raw_ofdm_sampler (fft_length, symbol_length, bandwidth, timeout, debug)
  );
}

raw_ofdm_sampler::raw_ofdm_sampler (unsigned int fft_length,
                                    unsigned int symbol_length,
                                    unsigned int bandwidth,
                                    unsigned int timeout,
                                    bool debug)
  : gr::block ("ofdm_sampler",
      gr::io_signature::make3 (3, 3,
        sizeof (gr_complex),
        sizeof(char),
        sizeof(float)),
      gr::io_signature::make(1, 1,
        sizeof (gr_complex)*fft_length)),
//      gr::io_signature::make2 (2, 2, sizeof (gr_complex)*fft_length,sizeof(char))
    d_fft_length(fft_length),
    d_symbol_length(symbol_length),
    d_bandwidth(bandwidth),
    d_state(STATE_NO_SIG),
    d_debug(debug)
{
  d_timeout_max = timeout;

  d_noise = 1e-8;
  d_sum_noise = 0;
  d_noise_count = 0;
  d_cur_symbol = 0;
  d_symbol1 = 0;
  d_symbol2 = 0;

  d_new_pkt = false;
  d_relay = false;

  d_init_hw_secs = 0;
  d_init_hw_frac = 0;
  d_init_pc_secs = 0;
  d_init_pc_frac = 0;

  // The ADC is 100 million samples / sec.  That translates to 2.5ns of time for every sample.
  // And, we need to take decimation rate into account.
  d_time_per_sample = 1.0 / d_bandwidth;
  
  //set_relative_rate(1.0/(double) fft_length);   // buffer allocator hint

  set_tag_propagation_policy(TPP_DONT);  /* TPP_DONT TPP_ALL_TO_ALL TPP_ONE_TO_ONE */
}

double raw_ofdm_sampler::get_current_hwtime()
{
  double elapsed = (double)d_samples_passed * d_time_per_sample;
  uint64_t sync_sec = (int)elapsed + d_init_hw_secs;
  double sync_frac_sec = elapsed - (int)elapsed + d_init_hw_frac;
  double time = sync_sec + sync_frac_sec;

  return time;
}


void
raw_ofdm_sampler::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  // each time we allow one symbol in
  int nreqd = d_symbol_length + d_fft_length;
  unsigned ninputs = ninput_items_required.size();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = noutput_items*d_symbol_length;
}


// http://gnuradio.org/redmine/projects/gnuradio/wiki/Guided_Tutorial_Programming_Topics#523-Tag-propagation
void raw_ofdm_sampler::get_init_timestamp(int port, int items_num, pmt::pmt_t KEY, pmt::pmt_t KEY2)
{
  // Use the stream tags to get USRP *INIT* timestamp
  std::vector<gr::tag_t> rx_time_tags;
  std::vector<gr::tag_t> rx_rate_tags;
  const uint64_t nread = this->nitems_read(port); //number of items read on port 0
  double offset_time_us = 0;

  this->get_tags_in_range(rx_rate_tags, port, nread, nread+items_num, RATE_KEY);
  if(rx_rate_tags.size()>0) {
      size_t t = rx_rate_tags.size()-1;
      const uint64_t sample_offset = rx_rate_tags[t].offset;  // distance from sample to timestamp in samples
      const pmt::pmt_t &value = rx_rate_tags[t].value;
      double rate = pmt::to_double(value);

      d_bandwidth = rate;

      printf(" [SAMPLER] USRP Rate = %f \n", rate);

  }

  this->get_tags_in_range(rx_time_tags, port, nread, nread+items_num, KEY);

  // See if there is a RX timestamp (only on first block or after underrun)
  if(rx_time_tags.size()>0) {
    size_t t = rx_time_tags.size()-1;

    // Take the last timestamp
    const uint64_t sample_offset = rx_time_tags[t].offset;  // distance from sample to timestamp in samples
    const pmt::pmt_t &value = rx_time_tags[t].value;

    // If the offset is greater than 0, this is a bit odd and complicated
    // A timestamp tag is prodcued at uhd start and after overflows
    // NOTE: offset is absolutely
    offset_time_us = sample_offset * 1.0/d_bandwidth;

    if(sample_offset>0) {
      #if 0
      printf(" WARNING: USRP OVERFLOW | nread=%ld | offset=%ld ", (long)nread, (long)sample_offset);
      std::cout << value << std::endl;
      return;
      #else
      printf(" [SAMPLER] WARNING: USRP TIME_KEY | nread=%ld | offset=%ld \n", (long)nread, (long)sample_offset);
      //offset_time_us = sample_offset * 1.0/rx_rate;
      #endif
    }

    // Now, compute the actual time in seconds and fractional seconds of the preamble
    d_init_hw_secs = pmt::to_uint64(tuple_ref(value,0));
    d_init_hw_frac = pmt::to_double(tuple_ref(value,1))-offset_time_us;

    gettimeofday(&sampler_timer, NULL);
    d_init_pc_frac = sampler_timer.tv_usec*1.0/1e6;
    d_init_pc_secs = sampler_timer.tv_sec;
    //my change
    //printf(" [SAMPLER] USRP HW init timestamp = %f %f %f\n", d_init_pc_secs+d_init_pc_frac, d_init_hw_frac+d_init_hw_secs, sample_offset);
    //printf(" [SAMPLER] USRP HW init timestamp = (pc)%f (usrp)%f rx_rate=%d ninputs=%d\n", d_init_pc_secs+d_init_pc_frac, d_init_hw_frac+d_init_hw_secs, d_bandwidth, items_num);  
    printf(" [SAMPLER] init timestamp = (pc)%f (usrp)%f rx_rate=%d\n", d_init_pc_secs+d_init_pc_frac, d_init_hw_frac+d_init_hw_secs, d_bandwidth);  
  }


}




void raw_ofdm_sampler::read_stream_tags(int port, int rel_pos) 
{
  // get PEAK_CFO stream tags, and then write SAMPLER_CFO stream tags
  // convert PEAK_CFO to SAMPLER_CFO, but in different position
  // I guess gnuradio does not have good support of stream tag in gr_block

  std::vector<gr::tag_t> rx_sync_tags;

  const uint64_t nstart = nitems_read(port);
  const uint64_t nend   = nstart+rel_pos+1+1;  // FIXME: 1 magic number

  this->get_tags_in_range(rx_sync_tags, port, nstart, nend, PEAK_SU_CFO);
  d_sync_cfo = 0;
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    d_sync_cfo = pmt::to_double(value);
    if(d_debug) 
      printf(" [SAMPLER] Get SU cfo tag, Range: [%ld:%ld) | Offset: %ld | CFO: %f | rel_pos = %d \n", 
        (long)nstart, (long)nend, (long)sample_offset, d_sync_cfo, rel_pos);
  } else {
    if(d_debug) //d_debug)
      printf(" [SAMPLER] Preamble received, with no SU CFO?  Range: [%ld:%ld) | rel_pos = %d \n", (long)nstart, (long)nend, rel_pos);
  }

  this->get_tags_in_range(rx_sync_tags, port, nstart, nend, PEAK_MU_CFO);
  d_sync_cfo0 = 0; d_sync_cfo1 = 0;
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    const pmt::pmt_t list = rx_sync_tags[t].value;
    d_sync_cfo0 = pmt::to_double(pmt::nth(0, list));
    d_sync_cfo1 = pmt::to_double(pmt::nth(1, list));
    if(d_debug) {
      printf(" [SAMPLER] Get MU cfo tag, Range: [%ld:%ld) | Offset: %ld CFO: %f %f | rel_pos = %d \n", 
        (long)nstart, (long)nend, (long)sample_offset, d_sync_cfo0, d_sync_cfo1, rel_pos);
    }
  } else {
    if(d_debug)
      printf(" [SAMPLER] Preamble received, with no MU CFO?  Range: [%ld:%ld) | rel_pos = %d \n", (long)nstart, (long)nend, rel_pos);
  }

  this->get_tags_in_range(rx_sync_tags, port, nstart, nend, PKT_DECODE_TIME);
  d_decode_time_secs = 0;
  d_decode_time_frac = 0;
  
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    d_decode_time_secs =  pmt::to_uint64(pmt::tuple_ref(value,0));
    d_decode_time_frac =  pmt::to_double(pmt::tuple_ref(value,1));
    if(d_debug) {
      printf(" [SAMPLER] Get decode time tag, Range: [%ld:%ld) | Offset: %ld time: %lf | rel_pos = %d \n", 
        (long)nstart, (long)nend, (long)sample_offset, d_decode_time_secs+d_decode_time_frac, rel_pos);
    }
  } else {
    if(d_debug)
      printf(" [SAMPLER] Preamble received, with no PKT_DECODE_TIME?  Range: [%ld:%ld) | rel_pos = %d \n", (long)nstart, (long)nend, rel_pos);
  }

}


void raw_ofdm_sampler::write_stream_tags1(int out_port, int rel_pos)
{
  uint64_t pos = nitems_written(out_port) + rel_pos;

  // get triggering timestamp
  double elapsed = (double)d_samples_passed * d_time_per_sample;

  #if 0
  // use the last time stamp to calculate the time of the preamble synchronization
  uint64_t sync_sec = (int)elapsed + d_init_hw_secs;
  double sync_frac_sec = elapsed - (int)elapsed + d_init_hw_frac;
  if(sync_frac_sec>1) {
    sync_sec += (uint64_t)sync_frac_sec; 
    sync_frac_sec -= (uint64_t)sync_frac_sec;
  }
  #else

  double sync_hw_time = d_init_hw_secs + d_init_hw_frac + elapsed;
  uint64_t sync_sec = (uint64_t)sync_hw_time;
  double sync_frac_sec = sync_hw_time - sync_sec;
  #endif

  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
  const pmt::pmt_t hw_time = pmt::make_tuple(
    pmt::from_uint64(sync_sec),      // FPGA clock in seconds that we found the sync
    pmt::from_double(sync_frac_sec)  // FPGA clock in fractional seconds that we found the sync
  );
  //const pmt::pmt_t samples = pmt::pmt_from_uint64(d_samples_passed);
  
  gettimeofday(&sampler_timer, NULL);
  const pmt::pmt_t pc_time = pmt::make_tuple(
    pmt::from_uint64(sampler_timer.tv_sec),         // PC time in seconds that we found the sync
    pmt::from_double(sampler_timer.tv_usec*1.0/1e6)   // PC time in fractional seconds that we found the sync
  );

  // write SAMPLER_TIME stream tags
  std::vector<gr::tag_t> rx_sync_tags;

  // TODO: understand the relationship between flag_output and nitems_written(1)
  this->add_item_tag(out_port, pos, SAMPLER_HW_TIME, hw_time, _id);
  this->add_item_tag(out_port, pos, SAMPLER_PC_TIME, pc_time, _id);
  if(d_debug)
    printf(" [SAMPLER] Adding time tags, written_pos: %ld | Time: %f | d_samples_passed: %ld \n",  (long)pos, sync_sec+sync_frac_sec, (long)d_samples_passed);

  if(std::abs(d_sync_cfo) > 1e-4) { // valid value
    pmt::pmt_t value = pmt::from_double(d_sync_cfo);
    this->add_item_tag(out_port, pos, SAMPLER_SU_CFO, value, _id);
  }

  if(std::abs(d_sync_cfo0) > 1e-4) { // valid value
    pmt::pmt_t value0 = pmt::from_double(d_sync_cfo0);
    pmt::pmt_t value1 = pmt::from_double(d_sync_cfo1);

    pmt::pmt_t list = pmt::list2(value0, value1);
    this->add_item_tag(out_port, pos, SAMPLER_MU_CFO, list, _id);
  }

  if(d_decode_time_secs > 0) {
    const pmt::pmt_t decode_time = pmt::make_tuple( pmt::from_uint64(d_decode_time_secs),
                                                    pmt::from_double(d_decode_time_frac));
    this->add_item_tag(out_port, pos, PKT_DECODE_TIME, decode_time,  _id);
    #if 0
    static struct timeval tv_last;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("\t[sampler] tv = %.06f\n", tv.tv_sec-tv_last.tv_sec + (tv.tv_usec-tv_last.tv_usec)*1.0/1e6 );
    memcpy(&tv_last, &tv, sizeof(tv));
    #endif

  }

  /*
  const uint64_t nstart = nread;
  const uint64_t nend = nread+j+1;  // FIXME: 1 magic number
  this->get_tags_in_range(rx_sync_tags, in_port, nstart, nend, PEAK_SU_CFO);
  double sync_cfo = 0;
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const pmt::pmt_t value = rx_sync_tags[t].value;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    sync_cfo = pmt::to_double(value);
    this->add_item_tag(out_port, pos, SAMPLER_SU_CFO, value, _id);
    if(d_debug) 
      printf(" [SAMPLER] Get SU cfo tag, Range: [%ld:%ld) | Offset: %ld | written_pos: %ld | CFO: %f | index = %d \n", (long)nstart, (long)nend, (long)sample_offset, (long)pos, sync_cfo, j);
  } else {
    if(1)
      printf(" [SAMPLER] Preamble received, with no SU CFO?  Range: [%d:%d) | index = %d \n", nread, nread+j, j);
  }

  this->get_tags_in_range(rx_sync_tags, in_port, nstart, nend, PEAK_MU_CFO);
  if(rx_sync_tags.size()>0) {
    size_t t = rx_sync_tags.size()-1;
    const uint64_t sample_offset = rx_sync_tags[t].offset;
    const pmt::pmt_t list = rx_sync_tags[t].value;
    double cfo0 = pmt::to_double(pmt::nth(0, list));
    double cfo1 = pmt::to_double(pmt::nth(1, list));
    this->add_item_tag(out_port, pos, SAMPLER_MU_CFO, list, _id);
    if(d_debug) {
      printf(" [SAMPLER] Get MU cfo tag, Range: [%ld:%ld) | Offset: %ld CFO: %f %f | index = %d \n", (long)nstart, (long)nend, (long)sample_offset, cfo0, cfo1, j);
      printf(" [SAMPLER] Write MU cfo tag, Port: %d | Pos: %ld | CFO: %f %f \n", out_port, (long)pos, cfo0, cfo1);
    }
  } else {
    if(d_debug)
      printf(" [SAMPLER] Preamble received, with no MU CFO?  Range: [%ld:%ld) | index = %d \n", (long)nstart, (long)nend, j);
  }*/

  /* --------------------------------------------------------------------------------------------------------- */
  /*                                 End of Stream Tags Function                                               */
  /* --------------------------------------------------------------------------------------------------------- */

  if(0) {
    std::cout << "[SAMPLER] got a preamble.... calculating timestamp of sync\n";
    std::cout << "... relative_rate: " << relative_rate() << "\n";
    std::cout << "... d_time_per_sample: " << d_time_per_sample << "\n";
    std::cout << "... d_samples_passed: " << d_samples_passed << "\n";
    std::cout << "... bandwidth:" << d_bandwidth << "\n";
    std::cout << "... elapsed: "<< elapsed << "\n";
    std::cout << "... sync_sec: "<< sync_sec << "\n";
    std::cout << "... sync_fs: "<< sync_frac_sec << "\n";
    std::cout << "... written_pos: "<< pos << "\n";
    //std::cout << "... index: "<< index << "\n";
    //std::cout << "... cfo: "<<sync_cfo<<"\n";
  }
}

void raw_ofdm_sampler::write_stream_tags2(int out_port, int rel_pos)
{
  // write SYNC_SU_SNR / SYNC_MU_SNR
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
  uint64_t pos =  nitems_written(out_port) + rel_pos;
  /*
  const pmt::pmt_t noise_value = pmt::pmt_from_double(d_noise);
  this->add_item_tag(port, nwritten_pos, SAMPLER_NOISE_KEY, noise_value, _id);
  if(d_debug)
    printf(" [SAMPLER] d_noise = %f  d_noise_count = %d nread = %d  \n", d_noise, d_noise_count, this->nitems_read(0));
  */
  d_symbol1 = d_symbol1/d_fft_length;
  d_symbol2 = d_symbol2/d_fft_length;
  if(!d_relay) {
    // LTS in freq is multiplied by 2; so the norm should be multiplied by 4
    double snr_td = 10*log10((d_symbol1+d_symbol2)/(2*4*d_noise)-1);
    const pmt::pmt_t snr_val = pmt::list1(pmt::from_double(snr_td));
    this->add_item_tag(out_port, pos, SYNC_SU_SNR, snr_val, _id);
    //printf(" [SAMPLER] d_symbol1=%f d_symbol2=%f snr=%f d_noise=%10.8f | pos=%d \n",d_symbol1,d_symbol2,snr_td,d_noise,pos);
  }
  else{
    // FIXME: remove constant 4. Here we enlarge the power of LTS by 2 for better detection performance
    double snr_td_A = 10*log10(d_symbol1/(4*d_noise)-1);
    double snr_td_B = 10*log10(d_symbol2/(4*d_noise)-1);
    const pmt::pmt_t snr_val = pmt::list2(pmt::from_double(snr_td_A),pmt::from_double(snr_td_B));
    this->add_item_tag(out_port, pos, SYNC_MU_SNR, snr_val, _id);
    //printf(" [SAMPLER] d_symbolA=%f d_symbolB=%f snrA=%f snrB=%f d_noise=%10.8f | pos=%d \n",d_symbol1,d_symbol2,snr_td_A,snr_td_B,d_noise,pos);
  }
}



int
raw_ofdm_sampler::general_work (int noutput_items,
                                gr_vector_int &ninput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items)
{
  int port = 0;
  const uint64_t nread = this->nitems_read(port);

  int ninput = std::min(ninput_items[0], ninput_items[1]);
  ninput = std::min(ninput, ninput_items[2]);

  //printf("[sampler] nin=%d %d %d nout=%d nread=%d \n", ninput_items[0], 
  //  ninput_items[1], ninput_items[2], noutput_items, nread);

  get_init_timestamp(port, ninput, TIME_KEY, PCTIME_KEY);

  const gr_complex *iptr = (const gr_complex *) input_items[0];
  const char *trigger = (const char *) input_items[1];
  const float* cfo = (const float *) input_items[2];

  gr_complex *optr = (gr_complex *) output_items[0];
//  char *outsig = (char *) output_items[1];
  memset(optr, 0, noutput_items*sizeof(gr_complex));
//  memset(outsig, 0, noutput_items);
  int nconsume = 0;
  int nproduce = 0;

  while((nconsume < ninput) && (nproduce < noutput_items) ) {
    switch(d_state) {
      case STATE_NO_SIG : {
        // Search for a preamble trigger signal during the next symbol length
        // calculate the noise using a range of samples
        if((nread + nconsume > START_INDEX) &&  (d_noise_count < NOISE_COUNT) ) {
          d_sum_noise += norm(iptr[nconsume]);
          d_noise_count++;

          if(d_noise_count == NOISE_COUNT) {
            double noise = d_sum_noise;
            d_noise = noise*1.0 / d_noise_count;
            if(d_noise > 1e-4) {
              printf("[USRP][SAMPLER] d_noise is too LARGE. RESET TO DEFAULT 1e-8 \n");
              d_noise = 1e-8;
            }
            printf("[USRP][SAMPLER] d_noise = %g noise = %f d_noise_count = %d nread = %ld  \n", d_noise, noise, d_noise_count, (long)nread+nconsume);
            d_noise_count++;  // to stop noise couting
          }
        }
      
        if(trigger[nconsume] == USER_MODE   || 
          trigger[nconsume] == USER_A_MODE ||
          trigger[nconsume] == USER_B_MODE ||
          trigger[nconsume] == TWO_USER_MODE) {
          //printf("[SAMP] find a packet @pos=%d nread=%d index=%d | nin=%d nout=%d \n", nread+index, nread, index, ninput, noutput_items);
          d_state = STATE_FRAME;
          d_cur_symbol = 0;
          d_symbol1 = 0; 
          d_symbol2 = 0;
          d_samples_passed = nread + nconsume;

          d_sync_cfo = cfo[nconsume];

          d_trigger_val = trigger[nconsume];
          int port = 1;
          read_stream_tags(port, nconsume);
        }

        ++nconsume;
      }
      break;

      case STATE_FRAME: {
        // use this state when we have processed STS and are getting the rest of the frames
        if(d_cur_symbol == 0) {
   //       outsig[nproduce] = d_trigger_val; // tell the next block there is a preamble coming
          //printf(" nread=%d index=%d d_samples_passed=%d \n",nread,index,d_samples_passed);
  //        int out_port = 1;
  //        write_stream_tags1(out_port, nproduce);
        }

        int tmp = (ninput - nconsume)/d_symbol_length;
        int nsymbol_min = std::min(tmp, noutput_items);

        int nsym = 0;
        const gr_complex* src = iptr + nconsume;
        gr_complex* dst = optr + nproduce * d_fft_length;

        for(nsym = 0; nsym < nsymbol_min; ) {
          #if SAMPLER_DEBUG
          printf("d_cur_symbol=%d nsym=%d nsymbol_min=%d | nin=%d nout=%d | src=%u dst=%u \n", d_cur_symbol, nsym, nsymbol_min, ninput, noutput_items, src, dst);
          #endif

//          memcpy(dst, src, d_fft_length * sizeof(gr_complex));

          if(d_cur_symbol == 0) {
            memcpy(dst, src, d_fft_length * sizeof(gr_complex));
            dst +=d_fft_length;
            src += d_symbol_length;
            memcpy(dst, src, d_fft_length * sizeof(gr_complex));
            dst +=d_fft_length;
            nproduce+=2;
//            for(int pos=0; pos<d_symbol_length; pos++)
//              d_symbol1 += norm(src[pos]);
          }
          if(d_cur_symbol == 1) {
//            memcpy(dst, src, d_fft_length * sizeof(gr_complex));
//            dst +=d_symbol_length;
//            nproduce+=1;
//            for(int pos=0; pos<d_symbol_length; pos++)
//              d_symbol2 += norm(src[pos]);

//            int out_port = 1;
//            write_stream_tags2(out_port, nproduce);
          }          
          src += d_symbol_length;
//          dst += d_fft_length;

          nsym++;
          d_cur_symbol++;

          if(d_cur_symbol >= d_timeout_max) {
            d_state = STATE_NO_SIG;
            break;
          }
        }

//        nproduce += nsym;
        nconsume += nsym * d_symbol_length;

        consume_each(nconsume);
        return nproduce;
       }
       break;

      default:
        d_state = STATE_NO_SIG;
        break;
    }

  }

#if SAMPLER_DEBUG
  printf("RETURN: nconsume=%d nproduce=%d d_timeout_max=%d \n", nconsume, nproduce, d_timeout_max);
#endif

  consume_each(nconsume);
  return nproduce;

}

