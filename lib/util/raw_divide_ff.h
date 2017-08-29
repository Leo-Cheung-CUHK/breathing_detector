/* -*- c++ -*- */
/*
 * Copyright 20160121 CHEN,Jian
 */

#ifndef INCLUDED_DIVIDE_FF_H
#define INCLUDED_DIVIDE_FF_H

#include <rawofdm_api.h>
#include <gnuradio/sync_block.h>

class raw_divide_ff;

typedef boost::shared_ptr<raw_divide_ff> raw_divide_ff_sptr;

RAWOFDM_API raw_divide_ff_sptr
raw_make_divide_ff(void);

class RAWOFDM_API raw_divide_ff : public gr::sync_block
{

friend RAWOFDM_API raw_divide_ff_sptr
  raw_make_divide_ff(void);

protected:
  raw_divide_ff ();

 public:
  ~raw_divide_ff(void);

  int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif //INCLUDED_DIVIDE_FF_H
