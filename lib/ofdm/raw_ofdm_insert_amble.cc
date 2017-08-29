/* -*- c++ -*- */
/*
 * Copyright 2007,2010-2012 Free Software Foundation, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <raw_ofdm_insert_amble.h>
#include <gnuradio/io_signature.h>
#include <stdexcept>
#include <iostream>
#include <string.h>
#include <cstdio>
#include <pnc_tags.h>
#define DEBUG 0


raw_ofdm_insert_amble_sptr
raw_make_ofdm_insert_amble(int fft_length,
                          int ofdm_size,
          const std::vector<std::vector<gr_complex> > &preamble,
          const std::vector<std::vector<gr_complex> > &postamble)
{
  return (raw_ofdm_insert_amble_sptr)(new raw_ofdm_insert_amble(fft_length, ofdm_size, 
                                  preamble, postamble));
}

raw_ofdm_insert_amble::raw_ofdm_insert_amble (int fft_length,
        int ofdm_size,
        const std::vector<std::vector<gr_complex> > &preamble,
        const std::vector<std::vector<gr_complex> > &postamble)
    : gr::block("ofdm_insert_amble",
                gr::io_signature::make2(1, 2,
                                        sizeof(gr_complex)*fft_length,
                                        sizeof(char)),
                gr::io_signature::make2(1, 2,
                                        sizeof(gr_complex)*fft_length,
                                        sizeof(char))
                ),
    d_fft_length(fft_length),
    d_ofdm_size(ofdm_size),
    d_ofdm_size_output(0),
    d_preamble(preamble),
    d_postamble(postamble),
    d_state(ST_IDLE),
    d_nsymbols_output(0),
    d_pending_flag(0)
{
  // sanity check preamble symbols
  for(size_t i = 0; i < d_preamble.size(); i++) {
    if(d_preamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("raw_ofdm_insert_amble: invalid length for preamble symbol");
  }
  for(size_t i = 0; i < d_postamble.size(); i++) {
    if(d_postamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("raw_ofdm_insert_amble: invalid length for postamble symbol");
  }

  printf("raw_ofdm_insert_amble() d_preamble.size()=%lu d_postamble.size()=%lu\n", d_preamble.size(), d_postamble.size());

  /*  TPP_DONT  TPP_ALL_TO_ALL  TPP_ONE_TO_ONE */
  set_tag_propagation_policy(TPP_DONT);

  enter_idle();
}


raw_ofdm_insert_amble::~raw_ofdm_insert_amble()
{
}

void raw_ofdm_insert_amble::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  ninput_items_required[0] = noutput_items;
}

int
raw_ofdm_insert_amble::general_work(int noutput_items,
             gr_vector_int &ninput_items_v,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items)
{
  const pmt::pmt_t _id = pmt::string_to_symbol(this->name());
 
  int ninput_items = ninput_items_v.size()==2?std::min(ninput_items_v[0], ninput_items_v[1]):ninput_items_v[0];
  const gr_complex *in_sym = (const gr_complex *) input_items[0];
  const unsigned char *in_flag = NULL;
  if (input_items.size() == 2) {
    in_flag = (const unsigned char *) input_items[1];
  }

  gr_complex *out_sym = (gr_complex *) output_items[0];
  unsigned char *out_flag = NULL;
  if (output_items.size() == 2) {
    out_flag = (unsigned char *) output_items[1];
  }

  #if DEBUG
  if(in_flag) {
    std::vector<gr::tag_t> rx_tags;
    int tag_in_port = 1;
    this->get_tags_in_range(rx_tags, tag_in_port, nitems_read(tag_in_port), nitems_read(tag_in_port)+input_items.size(), SYNC_TIME);
    // See if there is a RX timestamp (only on first block or after underrun)
    if(rx_tags.size()>0) {
      size_t t = rx_tags.size()-1;
      const uint64_t my_tag_count = rx_tags[t].offset;
      std::cout<<">>> [PREAMBLE] tag count: "<< my_tag_count << " Range: ["<<nitems_read(tag_in_port) << ":" <<nitems_read(tag_in_port)+input_items.size() <<") \n";
    }
  }
  #endif

  int no = 0;     // number items output
  int ni = 0;     // number items read from input

  /* ctrl packet, only data payload
   * Normal:    1, 0, 0, ..., 9      out: 1, 0, 0,..., 6,6,6..., 9
   * TimeStamp: 2, 0, 0, ..., 9      out: 2, 0, 0,..., 6,6,6..., 9
   */

  while (no < noutput_items && ni < ninput_items){
    switch(d_state){
      case ST_IDLE:
        if (in_flag && (in_flag[ni]==1 || in_flag[ni]==2)) {  // this is first symbol of new payload
          enter_preamble();
          d_pending_flag = in_flag[ni];
        }
        else {
          ni++;			// eat one input symbol
        }
        break;

      case ST_PREAMBLE:
        assert(in_flag && in_flag[ni]>=1 && in_flag[ni]<=2);
        if (d_nsymbols_output >= (int) d_preamble.size()){
          // we've output all the preamble
          enter_first_payload();
        }
        else {
          memcpy(&out_sym[no * d_fft_length],
               &d_preamble[d_nsymbols_output][0],
               d_fft_length*sizeof(gr_complex));
          ////////////////////////////////////////////////////////////////////////////////////////////////
          if(d_nsymbols_output == 0) {  // first preamble
            // add by lzyou: get sync time, then add SOB, TIME tags
            std::vector<gr::tag_t> rx_sync_tags;
            int tag_in_port = 1;  // NOTE: we must add ni
            this->get_tags_in_range(rx_sync_tags, tag_in_port, nitems_read(tag_in_port)+ni, nitems_read(tag_in_port)+ninput_items_v[1], SYNC_TIME);
        
            if(rx_sync_tags.size()>0) {
              size_t t = rx_sync_tags.size()-1;
              const pmt::pmt_t &value = rx_sync_tags[t].value;
              uint64_t sync_secs       = pmt::to_uint64(pmt::tuple_ref(value, 0));
              double sync_frac_of_secs = pmt::to_double(pmt::tuple_ref(value,1));

              // chenjian@20150302: tags will stop at this block, rewrite the SYNC_TIME to port 1
              //       tx_sob/tx_time/tx_eob will move to cp_adder
              if(out_flag) {
                this->add_item_tag(tag_in_port, nitems_written(0)+no, SYNC_TIME, value, _id);
              }
              if (DEBUG) {
                std::cout.precision(16);
                std::cout << ">>> [PREAMBLE] timestamp received at " << rx_sync_tags[t].offset << " | "<<(double)(sync_secs+sync_frac_of_secs) << std::endl;
              }
            }
            
          }
          ////////////////////////////////////////////////////////////////////////////////////////////////
          if(out_flag) {
            out_flag[no] = d_pending_flag;
          }
          d_pending_flag = 0;
          no++;
          d_nsymbols_output++;
        }
        break;

      case ST_FIRST_PAYLOAD:
        // copy first payload symbol from input to output
        memcpy(&out_sym[no * d_fft_length],
          &in_sym[ni * d_fft_length],
          d_fft_length * sizeof(gr_complex));
        if(out_flag) {
          out_flag[no] = 6;
        }
        no++;
        ni++;
        d_ofdm_size_output++;
        enter_payload();
        break;
      
      case ST_PAYLOAD:
        if (in_flag && in_flag[ni]>=1 && in_flag[ni]<=2){ // this is first symbol of a new payload
          enter_idle();
          break;
        }
        // copy a symbol from input to output
        memcpy(&out_sym[no * d_fft_length],
          &in_sym[ni * d_fft_length],
          d_fft_length * sizeof(gr_complex));
        d_ofdm_size_output++;

        if(out_flag) {
          out_flag[no] = 6;
        }
        no++;
        ni++;
        // strong condition
        if(d_ofdm_size_output >= d_ofdm_size) {
          enter_postamble();
          // a little trick, let input has 1 item, so as gnuradio be able to continue scheduled
          ni--;
          //printf("raw_ofdm_insert_amble(%d) d_ofdm_size_output=%d (int)d_postamble.size()=%d\n", __LINE__, d_ofdm_size_output, (int)d_postamble.size());
        }
        break;

      case ST_POSTAMBLE:
        memcpy(&out_sym[no * d_fft_length],  
          &d_postamble[d_postamble_output][0],
          d_fft_length * sizeof(gr_complex));
        if (out_flag) {
          out_flag[no] = 0;
        }

        d_postamble_output++;

        if(d_postamble_output >= (int)d_postamble.size()) {
          // we send the postamble completely, so we tell the scheduler
          if(in_flag && in_flag[ni]==9 && out_flag) {
            out_flag[no] = 9;
          }
          ni++;
          enter_idle();
        }
        //printf(">>>>>>>>>>>>>>>d_postamble_output=%d\n",d_postamble_output);
        no++;
        break;

      default:
        std::cerr << "raw_ofdm_insert_amble: (can't happen) invalid state, resetting\n";
        enter_idle();
        break;
    }

    //printf("raw_ofdm_insert_amble(%d) d_output_symbol_debug=%d d_state=%d\n", __LINE__, d_output_symbol_debug, d_state);
  }
  //printf("raw_ofdm_insert_amble(%d) [%d %i] [%d %d]\n", __LINE__, ni, no, noutput_items, ninput_items);
  consume_each(ni);
  return no;
}

void
raw_ofdm_insert_amble::enter_idle()
{
  d_state = ST_IDLE;
  d_nsymbols_output = 0;
  d_pending_flag = 0;
  d_output_symbol_debug=0;

}

void
raw_ofdm_insert_amble::enter_preamble()
{
  d_state = ST_PREAMBLE;
  d_nsymbols_output = 0;
  d_pending_flag = 1;
}

void
raw_ofdm_insert_amble::enter_first_payload()
{
  d_state = ST_FIRST_PAYLOAD;
  d_ofdm_size_output = 0;
}

void
raw_ofdm_insert_amble::enter_payload()
{
  d_state = ST_PAYLOAD;
}

void
raw_ofdm_insert_amble::enter_postamble()
{
  d_state = ST_POSTAMBLE;
  d_postamble_output = 0;
}
