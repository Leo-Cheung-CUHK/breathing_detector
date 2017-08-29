/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */

#ifndef INCLUDED_RAW_MESSAGE_SOURCE_H
#define INCLUDED_RAW_MESSAGE_SOURCE_H

#include <gnuradio/blocks/api.h>
#include <gnuradio/sync_block.h>
#include <raw_message.h>
#include <raw_msg_queue.h>

class raw_message_source;
typedef boost::shared_ptr<raw_message_source> raw_message_source_sptr;

BLOCKS_API raw_message_source_sptr raw_make_message_source (size_t itemsize, int msgq_limit=0, bool flag=false);
BLOCKS_API raw_message_source_sptr raw_make_message_source (size_t itemsize, raw_msg_queue_sptr msgq);

/*!
 * \brief Turn received messages into a stream
 * \ingroup source_blk
 */
class BLOCKS_API raw_message_source : public gr::sync_block
{
 private:
  size_t	 	d_itemsize;
  raw_msg_queue_sptr	d_msgq;
  raw_message_sptr	d_msg;
  unsigned		d_msg_offset;
  bool			d_eof;
  bool                  d_flag;     // flag for sending TIME_KEY or SYNC_TIME

  friend BLOCKS_API raw_message_source_sptr
  raw_make_message_source(size_t itemsize, int msgq_limit, bool flag);
  friend BLOCKS_API raw_message_source_sptr
  raw_make_message_source(size_t itemsize, raw_msg_queue_sptr msgq);

 protected:
  raw_message_source (size_t itemsize, int msgq_limit, bool flag);
  raw_message_source (size_t itemsize, raw_msg_queue_sptr msgq);

 public:
  ~raw_message_source ();

  raw_msg_queue_sptr	msgq() const { return d_msgq; }

  int work (int noutput_items,
	    gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
};

#endif /* INCLUDED_RAW_MESSAGE_SOURCE_H */
