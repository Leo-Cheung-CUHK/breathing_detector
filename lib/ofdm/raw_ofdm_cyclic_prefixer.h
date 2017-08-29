/* -*- c++ -*- */
/*
 * Copyright 2004-2006,2011 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RAW_OFDM_CYCLIC_PREFIXER_H
#define INCLUDED_RAW_OFDM_CYCLIC_PREFIXER_H

#include <gnuradio/digital/api.h>
#include <gnuradio/sync_interpolator.h>
#include <stdio.h>

class raw_ofdm_cyclic_prefixer;
typedef boost::shared_ptr<raw_ofdm_cyclic_prefixer> raw_ofdm_cyclic_prefixer_sptr;

DIGITAL_API raw_ofdm_cyclic_prefixer_sptr 
raw_make_ofdm_cyclic_prefixer (size_t input_size, size_t output_size);


/*!
 * \brief adds a cyclic prefix vector to an input size long ofdm
 * symbol(vector) and converts vector to a stream output_size long.
 * \ingroup ofdm_blk
 */
class DIGITAL_API raw_ofdm_cyclic_prefixer : public gr::sync_interpolator
{
  friend DIGITAL_API raw_ofdm_cyclic_prefixer_sptr
    raw_make_ofdm_cyclic_prefixer (size_t input_size, size_t output_size);

 protected:
  raw_ofdm_cyclic_prefixer (size_t input_size, size_t output_size);

 public:
  int work (int noutput_items,
	    gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
 private:
  size_t d_input_size;
  size_t d_output_size;
};

#endif /* INCLUDED_RAW_OFDM_CYCLIC_PREFIXER_H */
