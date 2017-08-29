/* -*- c++ -*- */
/*
 * Copyright 2010 Szymon Jakubczak
 * Copyright 2012 Minglong/YQ
 */

#ifndef INCLUDED_RAW_OFDM_MAPPER_H
#define INCLUDED_RAW_OFDM_MAPPER_H

#include <rawofdm_api.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

class raw_ofdm_mapper;
typedef boost::shared_ptr<raw_ofdm_mapper> raw_ofdm_mapper_sptr;

RAWOFDM_API raw_ofdm_mapper_sptr
raw_make_ofdm_mapper (std::vector<int> carrier_map, unsigned int node_type=0); // @lzyou: default value for raw 
// Modified by minglong 2012-07-18, change the pilot for endnode

/*!
 * \brief take a stream of samples in and map to a vector of complex
 * constellation points suitable for IFFT input to be used in an ofdm
 * modulator.
 * This module only inserts pilots and empty samples for mute subcarriers.
 *   inputs:
 *      complex data
 *      enable flag [optional]
 *   outputs:
 *      data to be sent to IFFT
 *   parameters:
 *      carrier_map
 *      -- length implies fft_length
 *      -- value == 0, carrier empty
 *      -- value == 1, data carrier
 *      -- value == 2, pilot carrier ((-1,0) or (1,0))
 * \ingroup modulation_blk
 * \ingroup ofdm_blk
 */

class RAWOFDM_API raw_ofdm_mapper : public gr::sync_block
{
  friend RAWOFDM_API raw_ofdm_mapper_sptr
    raw_make_ofdm_mapper(std::vector<int> carrier_map, unsigned int node_type);
    //raw_make_ofdm_mapper(std::vector<int> carrier_map);        Modified by minglong 2012-07-18, change the pilot for endnode
 protected:
  raw_ofdm_mapper (std::vector<int> carrier_map, unsigned int node_type);  
  //raw_ofdm_mapper (std::vector<int> carrier_map);              Modified by minglong 2012-07-18, change the pilot for endnode

 private:
  // fixed params
  
  unsigned int          d_fft_length;
  std::vector<int>      d_data_carriers;
  std::vector<int>      d_pilot_carriers;
  unsigned int          d_node_type;

 public:
  int work(int noutput_items,
           //unsigned int node_type,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
  
};

#endif
