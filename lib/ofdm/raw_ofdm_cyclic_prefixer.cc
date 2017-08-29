/* -*- c++ -*- */
/*
 * Copyright 2004,2006,2010,2011 Free Software Foundation, Inc.
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

#include <raw_ofdm_cyclic_prefixer.h>
#include <gnuradio/io_signature.h>

static const pmt::pmt_t SYNC_TIME = pmt::string_to_symbol("sync_time");
static const pmt::pmt_t SOB_KEY = pmt::string_to_symbol("tx_sob");
static const pmt::pmt_t TIME_KEY = pmt::string_to_symbol("tx_time");
static const pmt::pmt_t EOB_KEY = pmt::string_to_symbol("tx_eob");

raw_ofdm_cyclic_prefixer_sptr
raw_make_ofdm_cyclic_prefixer (size_t input_size, size_t output_size)
{
  return gnuradio::get_initial_sptr(new raw_ofdm_cyclic_prefixer (input_size,
								      output_size));
}

raw_ofdm_cyclic_prefixer::raw_ofdm_cyclic_prefixer (size_t input_size,
							    size_t output_size)
  : gr::sync_interpolator ("ofdm_cyclic_prefixer",
			  gr::io_signature::make2 (1, 2, input_size*sizeof(gr_complex), sizeof(char)),
			  gr::io_signature::make (1, 1, sizeof(gr_complex)),
			  output_size), 
    d_input_size(input_size),
    d_output_size(output_size)
{

  /*  TPP_DONT  TPP_ALL_TO_ALL  TPP_ONE_TO_ONE */
  set_tag_propagation_policy(TPP_DONT);
}

int
raw_ofdm_cyclic_prefixer::work (int noutput_items,
				    gr_vector_const_void_star &input_items,
				    gr_vector_void_star &output_items)
{
  gr_complex *in = (gr_complex *) input_items[0];
  const unsigned char *in_flag = NULL;

  gr_complex *out = (gr_complex *) output_items[0];

  size_t cp_size = d_output_size - d_input_size;
  unsigned int i=0, j=0;

  if (input_items.size() == 2)
    in_flag = (const unsigned char *) input_items[1];  // the flag indicates whether one packet ends
#if 0
  j = cp_size;
  for(i=0; i < d_input_size; i++,j++) {
    out[j] = in[i];
  }

  j = d_input_size - cp_size;
  for(i=0; i < cp_size; i++, j++) {
    out[i] = in[j];
  }

  if (in_flag) {
    char x = in_flag[0];
    //printf(" >>>>>>> x: %d\n", x);
    if (x == 2){  // end of a packet
      const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
      this->add_item_tag(0, nitems_written(0)+d_output_size-1, EOB_KEY, pmt::PMT_T, _id);
      if (false)
        printf(">>> [CP] add EOB flag at %ld >>> \n", (long)nitems_written(0)+d_output_size-1);
    }
  }

  return d_output_size;
#else
  int nsym =0;
  int nsym_max = int(noutput_items / d_output_size);

  for(nsym = 0; nsym < nsym_max; ++nsym )
  {
    j = cp_size;
    for(i=0; i < d_input_size; i++,j++) {
      out[j] = in[i];
    }

    j = d_input_size - cp_size;
    for(i=0; i < cp_size; i++, j++) {
      out[i] = in[j];
    }

    if (in_flag) {
      char x = in_flag[0];
      //printf(" >>>>>>> x: %d\n", x);
      const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
      if(x==1 || x==2) {
        this->add_item_tag(0, nitems_written(0) + nsym*d_output_size, SOB_KEY, pmt::PMT_T, _id);
        if(x==2) {
          int tag_in_port = 1;
          std::vector<gr::tag_t> rx_sync_tags;
          //std::cout << ">>> [CP] nitems_read(tag_in_port)=" << nitems_read(tag_in_port) << std::endl;
          this->get_tags_in_range(rx_sync_tags, tag_in_port, nitems_read(tag_in_port)+nsym, nitems_read(tag_in_port)+nsym_max, SYNC_TIME);
          if(rx_sync_tags.size()>0) {
              size_t t = rx_sync_tags.size()-1;
              const pmt::pmt_t &value = rx_sync_tags[t].value;
              uint64_t sync_secs = pmt::to_uint64(pmt::tuple_ref(value, 0));
              double sync_frac_of_secs = pmt::to_double(pmt::tuple_ref(value,1));
              //std::cout << ">>> [CP] get SYNC TIME" << std::endl;
              this->add_item_tag(0, nitems_written(0) + nsym*d_output_size, TIME_KEY, value, _id);
          }
        }
      }

      if (x == 9){  // end of a packet
        this->add_item_tag(0, nitems_written(0)+(nsym+1)*d_output_size-1, EOB_KEY, pmt::PMT_T, _id);
        if (false)
          printf(">>> [CP] add EOB flag at %ld >>> \n", (long)nitems_written(0)+(nsym+1)*d_output_size-1);
      }
    }
    in_flag++;
    out += d_output_size;
    in += d_input_size;
  }

  return (nsym*d_output_size);

#endif
}
