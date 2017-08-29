/* -*- c++ -*- */
/*
 * Copyright 2005,2009,2010,2011 Free Software Foundation, Inc.
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

class raw_msg_queue;
typedef boost::shared_ptr<raw_msg_queue> raw_msg_queue_sptr;
%template(raw_msg_queue_sptr) boost::shared_ptr<raw_msg_queue>;

%rename(msg_queue2) raw_make_msg_queue;
raw_msg_queue_sptr raw_make_msg_queue(unsigned limit=0);

/*!
 * \brief thread-safe message queue
 */
%ignore raw_msg_queue;
class raw_msg_queue {
public:
  raw_msg_queue(unsigned int limit);
  ~raw_msg_queue();

  //! Generic msg_handler method: insert the message.
  //void handle(gr_message_sptr msg) { insert_tail (msg); }

  /*!
   * \brief Insert message at tail of queue.
   * \param msg message
   *
   * Block if queue if full.
   */
  //void insert_tail(gr_message_sptr msg);

  /*!
   * \brief Delete message from head of queue and return it.
   * Block if no message is available.
   */
  //gr_message_sptr delete_head();

  /*!
   * \brief If there's a message in the q, delete it and return it.
   * If no message is available, return 0.
   */
  raw_message_sptr delete_head_nowait();

  //! is the queue empty?
  bool empty_p() const;

  //! is the queue full?
  bool full_p() const;

  //! return number of messages in queue
  unsigned int count() const;

  //! Delete all messages from the queue
  void flush();
};

/*
 * The following kludge-o-rama releases the Python global interpreter
 * lock around these potentially blocking calls.  We don't want
 * libgnuradio-core to be dependent on Python, thus we create these
 * functions that serve as replacements for the normal C++ delete_head
 * and insert_tail methods.  The %pythoncode smashes these new C++
 * functions into the gr.msg_queue wrapper class, so that everything
 * appears normal.  (An evil laugh is heard in the distance...)
 */
#ifdef SWIGPYTHON
%inline %{
  raw_message_sptr raw_py_msg_queue__delete_head(raw_msg_queue_sptr q) {
    raw_message_sptr msg;
    GR_PYTHON_BLOCKING_CODE(
        msg = q->delete_head();
    )
    return msg;
  }

  void raw_py_msg_queue__insert_tail(raw_msg_queue_sptr q, raw_message_sptr msg) {
    GR_PYTHON_BLOCKING_CODE(
        q->insert_tail(msg);
    )
  }
%}

// smash in new python delete_head and insert_tail methods...
%pythoncode %{
raw_msg_queue_sptr.delete_head = raw_py_msg_queue__delete_head
raw_msg_queue_sptr.insert_tail = raw_py_msg_queue__insert_tail
raw_msg_queue_sptr.handle = raw_py_msg_queue__insert_tail
%}
#endif	// SWIGPYTHON
