/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_message_sink.h>
#include <pnc_tags.h>
#include <gnuradio/io_signature.h>
#include <cstdio>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <string.h>

#define DEBUG 0

std::vector<pmt::pmt_t> BA_KEY_SET;
std::vector<pmt::pmt_t> SU_KEY_SET;
std::vector<pmt::pmt_t> MU_KEY_SET;

std::vector<int>
get_in_sizeofs(size_t item_size, size_t num_symbol)
{
  std::vector<int> in_sizeofs;
  for(unsigned int i = 0; i < 4; i++) {
    if(i == 1)
      in_sizeofs.push_back(sizeof(char)*num_symbol);
    else
      in_sizeofs.push_back(item_size);
  }
  return in_sizeofs;
}

// public constructor that returns a shared_ptr
raw_message_sink_sptr 
raw_make_message_sink (size_t itemsize, size_t num_symbol, raw_msg_queue_sptr msgq, bool dont_block)
{
  return gnuradio::get_initial_sptr(new raw_message_sink(itemsize, num_symbol, msgq, dont_block));
}

raw_message_sink::raw_message_sink (size_t itemsize, size_t num_symbol, raw_msg_queue_sptr msgq, bool dont_block)
  : gr::sync_block("raw_message_sink",
    gr::io_signature::makev(1, 4, get_in_sizeofs(itemsize,num_symbol)),
    gr::io_signature::make(0, 0, 0)),
    d_itemsize(itemsize), d_msgq(msgq), d_dont_block(dont_block), d_num_symbol(num_symbol)
{
  BA_KEY_SET.push_back(SYNC_HW_TIME);
  BA_KEY_SET.push_back(SYNC_PC_TIME);
  BA_KEY_SET.push_back(PKT_DECODE_TIME);

  //SU_KEY_SET.push_back(SNR_SU_KEYS);
  SU_KEY_SET.push_back(SYNC_SU_CFO);
  SU_KEY_SET.push_back(SYNC_SU_SNR);

  MU_KEY_SET.push_back(SYNC_MU_CFO);
  MU_KEY_SET.push_back(SYNC_MU_SNR);
  //MU_KEY_SET.push_back(POWER_KEY1);
  //MU_KEY_SET.push_back(POWER_KEY2);
  //MU_KEY_SET.push_back(POWER_KEY3);
}

raw_message_sink::~raw_message_sink()
{
}

void
raw_message_sink::test_tags_reception(const int port, const uint64_t nitems)
{
  // NOTE: add your TAGS here
  uint64_t pos = this->nitems_read(port);
  std::vector<gr::tag_t> tags;

  /*
  // debug timestamp output
  this->get_tags_in_range(tags, port, pos, pos+nitems, PEAK_TIME);
  //printf(" [FIX] Range: [%d:%d) | noutput_items = %d | nitems = %d \n", nread, nread + ninput_items[port], noutput_items, ninput_items[port]);
  if(tags.size()>0) {
    size_t t = tags.size()-1;
    const uint64_t sample_offset = tags[t].offset;  // get offset for DEBUG
    const pmt::pmt_t value = tags[t].value;
    uint64_t lts_sync_secs = pmt::to_uint64(pmt::tuple_ref(value, 0));
    double lts_sync_frac_of_secs = pmt::to_double(pmt::tuple_ref(value,1));
    if(DEBUG) printf(" [SINK_TAG_RECEPTION][PEAK_TIME] Range: [%d:%d) | Time = %f | Offset = %d | t = %d \n", pos, pos + nitems, lts_sync_secs + lts_sync_frac_of_secs, sample_offset, t);
  } */

  this->get_tags_in_range(tags, port, pos, pos+nitems, PEAK_SU_CFO);
  if(tags.size()>0) {
    size_t t = tags.size()-1;
    const uint64_t sample_offset = tags[t].offset;  // get offset for DEBUG
    const pmt::pmt_t value = tags[t].value;
    double cfo = pmt::to_double(value);
    if(1) printf(" [SINK_TAG_RECEPTION][PEAK_SU_CFO] CFO = %10.6f | Offset = %ld | t = %ld \n", cfo, (long)sample_offset, (long)t);
  }

  this->get_tags_in_range(tags, port, pos, pos+nitems, PEAK_MU_CFO);
  if(tags.size()>0) {
    size_t t = tags.size()-1;
    const uint64_t sample_offset = tags[t].offset;  // get offset for DEBUG
    const pmt::pmt_t p_list = tags[t].value;

    for(size_t i=0; i<2; i++) {
       double cfo = pmt::to_double(pmt::nth(i, p_list));
       if(1) printf(" [SINK_TAG_RECEPTION][PEAK_MU_CFO] CFO = %10.6f | Offset = %ld \n", cfo, (long)sample_offset);
    }
  }

  this->get_tags_in_range(tags, port, pos, pos+nitems, SYNC_HW_TIME);
  if(tags.size()>0) {
    size_t t = tags.size()-1;
    const uint64_t sample_offset = tags[t].offset;  // get offset for DEBUG
    const pmt::pmt_t value = tags[t].value;

    double a = pmt::to_uint64(pmt::tuple_ref(value,0));
    double b = pmt::to_double(pmt::tuple_ref(value,1));
    if(1) printf(" [SINK_TAG_RECEPTION][SYNC_HW_TIME] | HW Time = %f | Offset = %ld \n", a + b, (long)sample_offset);
  }

  this->get_tags_in_range(tags, port, pos, pos+nitems, SYNC_MU_SNR);
  if(tags.size()>0) {
    size_t t = tags.size()-1;
    const uint64_t sample_offset = tags[t].offset;  // get offset for DEBUG
    const pmt::pmt_t p_list = tags[t].value;

    for(size_t i=0; i<2; i++) {
       double cfo = pmt::to_double(pmt::nth(i, p_list));
       if(1) printf(" [SINK_TAG_RECEPTION][PEAK_MU_CFO] CFO = %10.6f | Offset = %ld \n", cfo, (long)sample_offset);
    }
  }
}

void raw_message_sink::get_stream_tags(const int port, const uint64_t nitems, pmt::pmt_t KEY)
{
    std::vector<gr::tag_t> rx_tags;
    const uint64_t nread = this->nitems_read(port);

    // get POWER list tags
    this->get_tags_in_range(rx_tags, port, nread, nread+nitems, KEY);
    if(rx_tags.size()>0) {
        size_t t = rx_tags.size()-1;
        const pmt::pmt_t value = rx_tags[t].value;
        const uint64_t sample_offset = rx_tags[t].offset;

        if(pmt::equal(KEY, SYNC_SU_SNR)) {
          d_snr_list.clear();
          for(size_t i=0; i<pmt::length(value); i++) {
            double snr = pmt::to_double(pmt::nth(i, value));
            d_snr_list.push_back(snr);
            if(DEBUG) printf(" [SINK] SYNC_SU_SNR | SU_SNR[%d] = %10.6f | Offset = %ld \n", (int)i, snr, (long)sample_offset);
          }
        }
        if(pmt::equal(KEY, SYNC_MU_SNR)) {
          d_snr_list.clear();
          for(size_t i=0; i<pmt::length(value); i++) {
            double snr = pmt::to_double(pmt::nth(i, value));
            d_snr_list.push_back(snr);
            if(DEBUG) printf(" [SINK] SYNC_MU_SNR | MU_SNR[%d] = %10.6f | Offset = %ld \n", (int)i, snr, (long)sample_offset);
          }
        }
        /*if(pmt_equal(KEY, SNR_SU_KEYS)) {
        	d_snr_list.clear();
        	for(size_t i=0; i<pmt_length(value); i++) {
        		double snr = pmt::to_double(pmt::pmt_nth(i, value));
        		d_snr_list.push_back(snr);
        		if(DEBUG) printf(" [SINK] SNR_SU_KEYS | SNR[%d] = %10.6f | Offset = %d \n", i, snr, sample_offset);
        	}
        }*/
        else if(pmt::equal(KEY, POWER_KEY1)) {
          d_power_list.clear();
            for(size_t i=0; i<pmt::length(value); i++) {
                double power = pmt::to_double(pmt::nth(i, value));
                d_power_list.push_back(power);
                if(DEBUG) printf(" [SINK] POWER_KEY1 | POWER1[%d] = %10.6f | Offset = %ld \n", (int)i, power, (long)sample_offset);
            }
        }
        else if(pmt::equal(KEY, POWER_KEY2)) {
            for(size_t i=0; i<pmt::length(value); i++) {
                double power = pmt::to_double(pmt::nth(i, value));
                d_power_list.push_back(power);
                if(DEBUG) printf(" [SINK] POWER_KEY2 | POWER2[%d] = %10.6f | Offset = %ld \n", (int)i, power, (long)sample_offset);
            }
        }
        else if(pmt::equal(KEY, POWER_KEY3)) {
            for(size_t i=0; i<pmt::length(value); i++) {
                double power = pmt::to_double(pmt::nth(i, value));
                d_power_list.push_back(power);
                if(DEBUG) printf(" [SINK] POWER3[%d] = %10.6f | Offset = %ld \n", (int)i, power, (long)sample_offset);
            }
        }
        else if(pmt::equal(KEY, SYNC_HW_TIME)) {
          d_peak_hw_secs = pmt::to_uint64(pmt::tuple_ref(value,0));
          d_peak_hw_frac = pmt::to_double(pmt::tuple_ref(value,1));
          if(DEBUG) printf(" [SINK] SYNC_HW_TIME | HW Time = %f | Offset = %ld \n", d_peak_hw_secs + d_peak_hw_frac, (long)sample_offset);
        }
        else if(pmt::equal(KEY, SYNC_PC_TIME)) {
          d_peak_pc_secs = pmt::to_uint64(pmt::tuple_ref(value,0));
          d_peak_pc_frac = pmt::to_double(pmt::tuple_ref(value,1));
          if(DEBUG) printf(" [SINK] SYNC_PC_TIME | PC Time = %f | Offset = %ld \n", d_peak_pc_secs + d_peak_pc_frac, (long)sample_offset);
        }
        else if(pmt::equal(KEY, PKT_DECODE_TIME)) {
          d_decode_time_secs = pmt::to_uint64(pmt::tuple_ref(value,0));
          d_decode_time_frac = pmt::to_double(pmt::tuple_ref(value,1));
          if(DEBUG) printf(" [SINK] PKT_DECODE_TIME | DECODE Time = %f | Offset = %ld \n", d_decode_time_secs + d_decode_time_frac, (long)sample_offset);
        }
        else if(pmt::equal(KEY, SYNC_SU_CFO)) {
          d_cfo_list.clear();
          double cfo = pmt::to_double(value);
          d_cfo_list.push_back(cfo);
          if(DEBUG) printf(" [SINK] SYNC_SU_CFO | SU_CFO = %f | Offset = %ld \n", cfo, (long)sample_offset);
        }
        else if(pmt::equal(KEY, SYNC_MU_CFO)) {
          d_cfo_list.clear();
          for(size_t i=0; i<pmt::length(value); i++) {
            double cfo = pmt::to_double(pmt::nth(i, value));
            d_cfo_list.push_back(cfo);
            if(DEBUG) printf(" [SINK] SYNC_MU_CFO | MU_CFO[%d] = %f | Offset = %ld | nread=%ld \n", (int)i, cfo, (long)sample_offset, (long)nread);
          }
        }
    }
}

int
raw_message_sink::work(int noutput_items,
		      gr_vector_const_void_star &input_items,
		      gr_vector_void_star &output_items)
{
  const char *in0 = (const char *) input_items[0];
  const char *signal_in = (const char *) input_items[1];
  const char *in1  = (input_items.size() > 2) ? (const char *) input_items[2] : NULL;
  const char *in2  = (input_items.size() > 3) ? (const char *) input_items[3] : NULL;

  // build a message to hold whatever we've got
  // lzyou: the message includes XOR data, A's data and B's data
  int len = 1;
  if(in1 && in2) {        // ncma type, port 0/1/2 has data
    len += 2;
  }
  else if (in1 && !in2) { // rmud type, port 0/1 has data
    len = len+1;
  }
  else if (!in1 && in2) {
    printf(" ERROR: [RAW_MESSAGE_SINK] in1 must be not NULL in this case \n");
  }

  int np_items = 1;
  for(int ii=0; ii<np_items; ii++) {

    int port = 1;
    const uint64_t nitems = 1;
    //test_tags_reception(port, nitems);

    if (signal_in[0]) {
 
      for(int i=0; i<BA_KEY_SET.size(); i++) {
        get_stream_tags(port, nitems, BA_KEY_SET[i]);
      }

      for(int i=0; i<SU_KEY_SET.size(); i++) {
        get_stream_tags(port, nitems, SU_KEY_SET[i]);
      }

      for(int i=0; i<MU_KEY_SET.size(); i++) {
        get_stream_tags(port, nitems, MU_KEY_SET[i]);
      }
    }
    else {
      printf(" ======================= [RAW_MESSAGE_SINK] NO TAGS WHY??? =========== ");
    }


  // if we'd block, drop the data on the floor and say everything is OK
  if (d_dont_block && d_msgq->full_p()) {
    printf(" WARNING: [RAW_MESSAGE_SINK] pkts dropped, current pktNum=%d noutput_items=%d\n", d_msgq->count(), noutput_items);
    break;
  }

  //gettimeofday(&sink_timer, NULL);
  //printf("CCCCCC++: Before A MSG at time: %d %d \n", sink_timer.tv_sec, sink_timer.tv_usec);

  raw_message_sptr msg = raw_make_message(0,                // msg type
                                          d_itemsize,       // arg1 for other end
                                          len,	            // arg2 for other end (redundant)
                                          d_itemsize*len);  // len of msg
  memcpy(msg->msg(), in0, d_itemsize);

  if(len == 3) {
    memcpy(msg->msg()+d_itemsize, in1, d_itemsize);    
    memcpy(msg->msg()+d_itemsize*2, in2, d_itemsize);
  }
  else if (len == 2) {
    memcpy(msg->msg()+d_itemsize, in1, d_itemsize);    
  }

  if (signal_in[0]) {
    int user_type = signal_in[0];

    msg->set_snr_list(d_snr_list);
    msg->set_power_list(d_power_list);
    msg->set_timestamp(d_peak_hw_secs, d_peak_hw_frac);
    msg->set_pctime(d_peak_pc_secs, d_peak_pc_frac);
    msg->set_cfo(d_cfo_list);
    msg->set_decode_time(d_decode_time_secs, d_decode_time_frac);

    msg->set_user_type(user_type);

    /*struct timeval tv;
    gettimeofday(&tv, NULL);
    double cur_time = tv.tv_sec + tv.tv_usec*1.0/1e6;
    double gap = cur_time - (d_decode_time_secs+d_decode_time_frac);
    uint64_t secs = gap;
    double frac = gap - secs;
    msg->set_decode_time(secs, frac);*/

    //printf("[SINK] d_decode_time_secs=%lu d_decode_time_frac=%f \n",d_decode_time_secs,d_decode_time_frac);
    //msg->set_decode_time(d_peak_pc_secs, d_peak_pc_frac);

    if(d_peak_hw_secs == 0 && d_peak_hw_frac == 0) {
      printf(" =================================================== \n");
      printf(" [RAW_MESSAGE_SINK] d_peak_hw_secs(frac) == 0 \n");
      printf(" =================================================== \n");
      //exit(-1);
    }

    d_peak_hw_secs = 0; d_peak_hw_frac = 0;
    d_peak_pc_secs = 0; d_peak_pc_frac = 0;
    d_decode_time_secs = 0; d_decode_time_frac = 0;
  }
  //printf(" Add A MSG, noutput_items=%d ii=%d d_itemsize=%d \n", noutput_items, ii, d_itemsize);

  // FIXME: if we want to use raw_message_sptr
  // Error: raw_message_sink.cc:278:21: error: no matching function for call to 'gr_msg_queue::handle(raw_message_sptr&)'
  d_msgq->handle(msg);		// send it


  }

  return np_items;
}
