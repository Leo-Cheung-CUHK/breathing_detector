/* -*- c++ -*- */
/*
 * Copyright 2004,2010,2011 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <raw_pnc_frequency_modulator_fc.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sincos.h>
#include <math.h>
#include <boost/math/special_functions/trunc.hpp>
#include <iostream>

raw_pnc_frequency_modulator_fc_sptr raw_make_pnc_frequency_modulator_fc (double sensitivity)
{
  return gnuradio::get_initial_sptr(new raw_pnc_frequency_modulator_fc (sensitivity));
}

raw_pnc_frequency_modulator_fc::raw_pnc_frequency_modulator_fc (double sensitivity)
  : gr::sync_block ("pnc_frequency_modulator_fc",
		   gr::io_signature::make (1, 1, sizeof (gr_complex)),
		   gr::io_signature::make (1, 1, sizeof (gr_complex))),
    d_sensitivity (sensitivity), d_phase (0), d_value(0)
{
}

int
raw_pnc_frequency_modulator_fc::work (int noutput_items,
				 gr_vector_const_void_star &input_items,
				 gr_vector_void_star &output_items)
{
  gr_complex *in = (gr_complex *) input_items[0];
  gr_complex *out = (gr_complex *) output_items[0];


  for (int i = 0; i < noutput_items; i++){
    d_phase = d_phase + d_sensitivity * d_value;
    float oi, oq;
    gr::sincosf (d_phase, &oq, &oi);
    gr_complex t = gr_complex (oi, oq);
    out[i] = in[i] * t;
    //std::cout<<t<<" : "<<d_value<<std::endl;
    //std::cout<<i<<" : "<<noutput_items<<" : "<<d_phase<<" : "<<in[i]<<" : "<<out[i]<<std::endl;
  }

  // Limit the phase accumulator to [-16*pi,16*pi]
  // to avoid loss of precision in the addition above.

  if (fabs (d_phase) > 16 * M_PI){
    double ii = boost::math::trunc (d_phase / (2 * M_PI));
    d_phase = d_phase - (ii * 2 * M_PI);
  }

  return noutput_items;
}
