/* -*- c++ -*- */
/*
 * Copyright 2005 Free Software Foundation, Inc.
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
#ifndef INCLUDED_RAW_MESSAGE_H
#define INCLUDED_RAW_MESSAGE_H

#include <gnuradio/blocks/api.h>
#include <gnuradio/types.h>
#include <string>
#include <stdint.h>

class raw_message;
typedef boost::shared_ptr<raw_message> raw_message_sptr;

/*!
 * \brief public constructor for raw_message
 */
BLOCKS_API raw_message_sptr
raw_make_message(long type = 0, double arg1 = 0, double arg2 = 0, size_t length = 0);

BLOCKS_API raw_message_sptr
raw_make_message_from_string(const std::string s, long type = 0, double arg1 = 0, double arg2 = 0);

/*!
 * \brief Message class.
 *
 * \ingroup misc
 * The ideas and method names for adjustable message length were
 * lifted from the click modular router "Packet" class.
 */
class BLOCKS_API raw_message {
  raw_message_sptr d_next;    // link field for msg queue
  long             d_type;    // type of the message
  double           d_arg1;    // optional arg1
  double           d_arg2;    // optional arg2

  bool	         d_timestamp_valid;             // whether the timestamp is valid
  uint64_t       d_timestamp_sec;               // the preamble sync time in seconds
  double         d_timestamp_frac_sec;	        // the preamble sync time's fractional seconds
  double         d_pc_time_secs;                // the preamble sync pc time in seconds
  double         d_pc_time_frac;                // the preamble sync pc time in fractional secons
  bool           d_cfo_valid;                   // whether cfo value is valid
  double         d_snr;                         // the snr value for RawOFDM
  std::vector<double> d_snr_list;               // the snr value for OFDMs
  std::vector<double> d_cfo_list;               // the cfo value for Multi-User/Single-User
  std::vector<double> d_power_list;             // the effective power values for PNC
  std::vector<double> d_power_list2;            // the power values for PNC

  bool           d_decode_time_valid;
  uint64_t       d_decode_time_secs;
  double         d_decode_time_frac;


  unsigned char	 *d_buf_start;	// start of allocated buffer
  unsigned char  *d_msg_start;	// where the msg starts
  unsigned char  *d_msg_end;	// one beyond end of msg
  unsigned char  *d_buf_end;	// one beyond end of allocated buffer

  int             d_user_type;

  raw_message (long type, double arg1, double arg2, size_t length);

  friend BLOCKS_API raw_message_sptr
    raw_make_message (long type, double arg1, double arg2, size_t length);

  friend BLOCKS_API raw_message_sptr
    raw_make_message_from_string (const std::string s, long type, double arg1, double arg2);

  friend class raw_msg_queue;

  unsigned char *buf_data() const  { return d_buf_start; }
  size_t buf_len() const 	   { return d_buf_end - d_buf_start; }

public:
  ~raw_message ();

  long type() const   { return d_type; }
  double arg1() const { return d_arg1; }
  double arg2() const { return d_arg2; }
  bool timestamp_valid() const { return d_timestamp_valid; }
  uint64_t timestamp_sec() const { return d_timestamp_sec; }
  double timestamp_frac_sec() const { return d_timestamp_frac_sec; }
  double pctime_sec() const { return d_pc_time_secs; }
  double pctime_frac_sec() const { return d_pc_time_frac; }
  bool cfo_valid() const { return d_cfo_valid; }
  double snr_value() const { return d_snr; }
  std::vector<double> get_snr_values() const { return d_snr_list; }
  std::vector<double> power_list() const { return d_power_list; }
  std::vector<double> power_list2() const { return d_power_list2; }
  std::vector<double> cfo_values() const { return d_cfo_list; }

  void set_type(long type)   { d_type = type; }
  void set_arg1(double arg1) { d_arg1 = arg1; }
  void set_arg2(double arg2) { d_arg2 = arg2; }
  void set_cfo(std::vector<double> cfo_list) { d_cfo_list = cfo_list; d_cfo_valid = true; }
  void set_snr(double snr)    { d_snr = snr;}
  void set_snr_list(std::vector<double> snr_list) { d_snr_list = snr_list; };
  void set_power_list(std::vector<double> power_list) { d_power_list = power_list; }
  void set_power_list2(std::vector<double> power_list) { d_power_list2 = power_list; }
  void set_timestamp(uint64_t ps, double pfs) { d_timestamp_valid = true; d_timestamp_sec=ps; d_timestamp_frac_sec=pfs; }
  void set_pctime(double ps, double pfs) { d_pc_time_secs = ps; d_pc_time_frac = pfs; }

  void set_decode_time(uint64_t dec_secs, double dec_frac) { d_decode_time_valid=true; d_decode_time_secs=dec_secs; d_decode_time_frac=dec_frac;}
  double get_decode_time(void);

  void set_user_type(int user_type) { d_user_type = user_type; }
  int get_user_type(void) const { return d_user_type; }

  unsigned char *msg() const { return d_msg_start; }
  size_t length() const      { return d_msg_end - d_msg_start; }
  std::string to_string() const;

};

BLOCKS_API long raw_message_ncurrently_allocated ();

#endif /* INCLUDED_RAW_MESSAGE_H */
