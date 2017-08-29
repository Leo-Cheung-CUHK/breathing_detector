/* -*- c++ -*- */
/*
 * Copyright 2014 Lizhao You
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_message_source.h>
#include <gnuradio/io_signature.h>
#include <cstdio>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <string.h>

#include <pnc_tags.h>

#define VERBOSE 0

// public constructor that returns a shared_ptr

raw_message_source_sptr
raw_make_message_source(size_t itemsize, int msgq_limit, bool flag)
{
  return gnuradio::get_initial_sptr(new raw_message_source(itemsize, msgq_limit, flag));
}

// public constructor that takes existing message queue
raw_message_source_sptr
raw_make_message_source(size_t itemsize, raw_msg_queue_sptr msgq)
{
  return gnuradio::get_initial_sptr(new raw_message_source(itemsize, msgq));
}

raw_message_source::raw_message_source (size_t itemsize, int msgq_limit, bool flag)
  : gr::sync_block("message_source",
		  gr::io_signature::make(0, 0, 0),
		  gr::io_signature::make(1, 1, itemsize)),
    d_itemsize(itemsize), d_msgq(raw_make_msg_queue(msgq_limit)), d_msg_offset(0), d_eof(false), d_flag(flag)
{
}

raw_message_source::raw_message_source (size_t itemsize, raw_msg_queue_sptr msgq)
  : gr::sync_block("message_source",
		  gr::io_signature::make(0, 0, 0),
		  gr::io_signature::make(1, 1, itemsize)),
    d_itemsize(itemsize), d_msgq(msgq), d_msg_offset(0), d_eof(false)
{
}

raw_message_source::~raw_message_source()
{
}

int
raw_message_source::work(int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items)
{
  char *out = (char *) output_items[0];
  int nn = 0;
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());

  while (nn < noutput_items){
    if (d_msg){
      // Consume whatever we can from the current message
      int mm = std::min(noutput_items - nn, (int)((d_msg->length() - d_msg_offset) / d_itemsize));

      if( (d_msg_offset == 0) && (d_msg->timestamp_valid()) ) {  // start of the message
        const pmt::pmt_t val = pmt::make_tuple( 
                    pmt::from_uint64(d_msg->timestamp_sec()),      // FPGA clock in seconds that we found the sync
                    pmt::from_double(d_msg->timestamp_frac_sec())  // FPGA clock in fractional seconds that we found the sync
          );
        if(VERBOSE)
          printf(">>> set SYNC_TIME tag, timestamp %f at %ld | noutput_items = %d | d_itemsize = %d | d_flag = %d\n", d_msg->timestamp_sec()+d_msg->timestamp_frac_sec(), (long)nitems_written(0)+nn, noutput_items, (int)d_itemsize, d_flag);
        this->add_item_tag(0, nitems_written(0)+nn, SYNC_TIME, val, _id);  // nn denotes the starting point of a message
      }

      memcpy (out, &(d_msg->msg()[d_msg_offset]), mm * d_itemsize);

      nn += mm;
      out += mm * d_itemsize;
      d_msg_offset += mm * d_itemsize;
      assert(d_msg_offset <= d_msg->length());

      if (d_msg_offset == d_msg->length()){
        if (d_msg->type() == 1) {	           // type == 1 sets EOF
          d_eof = true;
        }
        d_msg.reset();
      }
    }
    else {
      // No current message
      if (d_msgq->empty_p() && nn > 0){    // no more messages in the queue, return what we've got
        break;
      }

      if (d_eof)
        return -1;

      d_msg = d_msgq->delete_head();	   // block, waiting for a message
      d_msg_offset = 0;

      if ((d_msg->length() % d_itemsize) != 0)
        throw std::runtime_error("msg length is not a multiple of d_itemsize");
    }
  }

  return nn;
}
