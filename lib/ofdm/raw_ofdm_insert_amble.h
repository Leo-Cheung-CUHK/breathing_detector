/* -*- c++ -*- */
/*
 * Copyright 2007,2011 Free Software Foundation, Inc.
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

#ifndef INCLUDED_RAW_OFDM_INSERT_AMBLE_H
#define INCLUDED_RAW_OFDM_INSERT_AMBLE_H


#include <gnuradio/block.h>
#include <vector>
#include <rawofdm_api.h>

class raw_ofdm_insert_amble;
typedef boost::shared_ptr<raw_ofdm_insert_amble> raw_ofdm_insert_amble_sptr;

RAWOFDM_API raw_ofdm_insert_amble_sptr
raw_make_ofdm_insert_amble(int fft_length, int ofdm_size,
				  const std::vector<std::vector<gr_complex> > &preamble,
          const std::vector<std::vector<gr_complex> > &postamble);

/*!
 * \brief insert "pre-modulated" preamble symbols before each payload.
 * \ingroup sync_blk
 * \ingroup ofdm_blk
 *
 * <pre>
 * input 1: stream of vectors of gr_complex [fft_length]
 *          These are the modulated symbols of the payload.
 *
 * input 2: stream of char.  The LSB indicates whether the corresponding
 *          symbol on input 1 is the first symbol of the payload or not.
 *          It's a 1 if the corresponding symbol is the first symbol,
 *          otherwise 0.
 *
 * N.B., this implies that there must be at least 1 symbol in the payload.
 *
 *
 * output 1: stream of vectors of gr_complex [fft_length]
 *           These include the preamble symbols and the payload symbols.
 *
 * output 2: stream of char.  The LSB indicates whether the corresponding
 *           symbol on input 1 is the first symbol of a packet (i.e., the
 *           first symbol of the preamble.)   It's a 1 if the corresponding
 *           symbol is the first symbol, otherwise 0.
 * </pre>
 *
 * \param fft_length length of each symbol in samples.
 * \param preamble   vector of symbols that represent the pre-modulated preamble.
 */

class RAWOFDM_API raw_ofdm_insert_amble : public gr::block
{
  friend RAWOFDM_API raw_ofdm_insert_amble_sptr
  raw_make_ofdm_insert_amble(int fft_length, int ofdm_size,
				    const std::vector<std::vector<gr_complex> > &preamble,
            const std::vector<std::vector<gr_complex> > &postamble);

protected:
  raw_ofdm_insert_amble(int fft_length, int ofdm_size,
			       const std::vector<std::vector<gr_complex> > &preamble,
             const std::vector<std::vector<gr_complex> > &postamble);

private:
  enum state_t {
    ST_IDLE,
    ST_PREAMBLE,
    ST_FIRST_PAYLOAD,
    ST_PAYLOAD,
    ST_POSTAMBLE
  };

  int						d_fft_length;
  int           d_ofdm_size;
  int           d_ofdm_size_output;
  int           d_postamble_output;
  const std::vector<std::vector<gr_complex> > 	d_preamble;
  const std::vector<std::vector<gr_complex> >   d_postamble;
  state_t					d_state;
  int						d_nsymbols_output;
  int						d_pending_flag;
  int           d_output_symbol_debug;

public:
  ~raw_ofdm_insert_amble();

  void enter_idle();
  void enter_preamble();
  void enter_first_payload();
  void enter_payload();
  void enter_postamble();

  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);
  void forecast (int noutput_items, gr_vector_int &ninput_items_required);

};

#endif /* INCLUDED_RAW_OFDM_INSERT_AMBLE_H */
