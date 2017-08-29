/* -*- c++ -*- */
/*
 * Copyright 2005,2009 Free Software Foundation, Inc.
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
#include <raw_msg_queue.h>
#include <stdexcept>

raw_msg_queue_sptr
raw_make_msg_queue(unsigned int limit)
{
  return raw_msg_queue_sptr (new raw_msg_queue(limit));
}

raw_msg_queue::raw_msg_queue(unsigned int limit)
  : d_not_empty(), d_not_full(),
    /*d_head(0), d_tail(0),*/ d_count(0), d_limit(limit)
{
}

raw_msg_queue::~raw_msg_queue()
{
  flush ();
}

void
raw_msg_queue::insert_tail(raw_message_sptr msg)
{
  if (msg->d_next)
    throw std::invalid_argument("raw_msg_queue::insert_tail: msg already in queue");

  gr::thread::scoped_lock guard(d_mutex);

  while (full_p())
    d_not_full.wait(guard);

  if (d_tail == 0){
    d_tail = d_head = msg;
    //msg->d_next = 0;
    msg->d_next.reset();
  }
  else {
    d_tail->d_next = msg;
    d_tail = msg;
    //msg->d_next = 0;
    msg->d_next.reset();
  }
  d_count++;
  d_not_empty.notify_one();
}

raw_message_sptr
raw_msg_queue::delete_head()
{
  gr::thread::scoped_lock guard(d_mutex);
  raw_message_sptr m;

  while ((m = d_head) == 0)
    d_not_empty.wait(guard);

  d_head = m->d_next;
  if (d_head == 0){
    //d_tail = 0;
    d_tail.reset();
  }

  d_count--;
  // m->d_next = 0;
  m->d_next.reset();
  d_not_full.notify_one();
  return m;
}

raw_message_sptr
raw_msg_queue::delete_head_nowait()
{
 gr::thread::scoped_lock guard(d_mutex);
  raw_message_sptr m;

  if ((m = d_head) == 0){
    //return 0;
    return raw_message_sptr();
  }

  d_head = m->d_next;
  if (d_head == 0){
    //d_tail = 0;
    d_tail.reset();
  }

  d_count--;
  //m->d_next = 0;
  m->d_next.reset();
  d_not_full.notify_one();
  return m;
}

void
raw_msg_queue::flush()
{
  raw_message_sptr	m;

  while ((m = delete_head_nowait ()) != 0)
    ;
}
