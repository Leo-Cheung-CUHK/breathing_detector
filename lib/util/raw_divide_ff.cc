/* -*- c++ -*- */
/*
 * Copyright 2015 CHEN,Jian
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_divide_ff.h>
#include <gnuradio/io_signature.h>

#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <sys/time.h>

#include <volk/volk.h>

raw_divide_ff_sptr
raw_make_divide_ff(void)
{
  return raw_divide_ff_sptr (
        new raw_divide_ff ()
        );
}

raw_divide_ff::raw_divide_ff (void)
  : gr::sync_block ("divide_ff",
        gr::io_signature::make2 (2, 2, sizeof(float), sizeof(float)),
        gr::io_signature::make (1, 1, sizeof(float)))
{

  //const int alignment_multiple = volk_get_alignment() / sizeof(float);
  //set_alignment(std::max(1, alignment_multiple));
  
}

raw_divide_ff::~raw_divide_ff(void)
{

}


int
raw_divide_ff::work (int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
{
    const float *iptr0 = (float *) input_items[0];
    const float *iptr1 = (float *) input_items[1];

    float *optr = (float *) output_items[0];

    volk_32f_x2_divide_32f(optr,  iptr0, iptr1, noutput_items);

    return noutput_items;
    
}