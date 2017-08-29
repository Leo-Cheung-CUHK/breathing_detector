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

class raw_message;
typedef boost::shared_ptr<raw_message> raw_message_sptr;
%template(raw_message_sptr) boost::shared_ptr<raw_message>;

%rename(message_from_string2) raw_make_message_from_string;
raw_message_sptr
raw_make_message_from_string(const std::string s, long type = 0, double arg1 = 0, double arg2 = 0);

%rename(message2) raw_make_message;
raw_message_sptr
raw_make_message(long type = 0, double arg1 = 0, double arg2 = 0, size_t length = 0);

/*!
 * \brief Message.
 *
 * The ideas and method names for adjustable message length were
 * lifted from the click modular router "Packet" class.
 */
class raw_message {
  raw_message (long type, double arg1, double arg2, size_t length);

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

  double get_decode_time(void);

  int get_user_type(void) const { return d_user_type; }

  unsigned char *msg() const { return d_msg_start; }
  size_t length() const      { return d_msg_end - d_msg_start; }
  std::string to_string() const;
};

%rename(message_ncurrently_allocated) raw_message_ncurrently_allocated;
long raw_message_ncurrently_allocated();

