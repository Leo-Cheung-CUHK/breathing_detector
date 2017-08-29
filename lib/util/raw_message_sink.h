/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */
 
#ifndef INCLUDED_RAW_MESSAGE_SINK_H
#define INCLUDED_RAW_MESSAGE_SINK_H

#include <gnuradio/blocks/api.h>
#include <gnuradio/sync_block.h>
#include <raw_message.h>
#include <raw_msg_queue.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

class raw_message_sink;
typedef boost::shared_ptr<raw_message_sink> raw_message_sink_sptr;

BLOCKS_API raw_message_sink_sptr raw_make_message_sink (size_t itemsize, size_t num_symbol,
					   raw_msg_queue_sptr msgq,
					   bool dont_block);

/*!
 * \brief Gather received items into messages and insert into msgq
 * \ingroup sink_blk
 */
class BLOCKS_API raw_message_sink : public gr::sync_block
{
 private:
  size_t	 	d_itemsize;
  size_t                d_num_symbol;
  raw_msg_queue_sptr	d_msgq;
  //gr_msg_queue_sptr	d_msgq;
  bool			d_dont_block;

  // FIXME: use std::vector<double> to generize SU and MU, and remove d_power_list
  std::vector<double>	d_snr_list;		// d_snr_fd, and d_snr_td, d_snr_fd2
  std::vector<double>   d_power_list;
  std::vector<double>   d_cfo_list;

  uint64_t		d_peak_hw_secs;
  double		d_peak_hw_frac;
  uint64_t		d_peak_pc_secs;
  double		d_peak_pc_frac;

  uint64_t      d_peak_hw_samples;

  uint64_t      d_decode_time_secs;
  double        d_decode_time_frac;

  friend BLOCKS_API raw_message_sink_sptr
  raw_make_message_sink(size_t itemsize, size_t num_symbol, raw_msg_queue_sptr msgq, bool dont_block);

 protected:
  raw_message_sink (size_t itemsize, size_t num_symbol, raw_msg_queue_sptr msgq, bool dont_block);

 public:
  ~raw_message_sink ();

  void test_tags_reception(const int port, const uint64_t nitems);
  void get_stream_tags(const int port, const uint64_t nitems, pmt::pmt_t KEY);

  int work (int noutput_items,
	    gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
};

#endif /* INCLUDED_RAW_MESSAGE_SINK_H */
