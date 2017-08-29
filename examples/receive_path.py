#!/usr/bin/env python
#
# Copyright 2014 Lizhao You
#
# 

import time

from gnuradio import gr, blocks, eng_notation
from gnuradio.eng_option import eng_option
import raw
import ofdm_rxtx, raw_qam
import gnuradio.gr.gr_threading as _threading

from phy_params import PNC_TYPE

# /////////////////////////////////////////////////////////////////////////////
#                              receive path
# /////////////////////////////////////////////////////////////////////////////
class ofdm_receive_path(gr.hier_block2):
  """
  Generic receive path:
    @note: input samples
    @note: provide PNC, RAW receive path, that depends on MODE
  """
  def __init__(self, options_rx, callback):
    if options_rx.mode == "PNC":
      ofdm = ofdm_rxtx.FPNC_RX(options_rx)
    elif options_rx.mode == "RAW":
      ofdm = ofdm_rxtx.OFDM_RX(options_rx)
    qam = raw_qam.qam_rx(options_rx.bitrate, ofdm.params.data_tones, ofdm.size, log=options_rx.log)
    framebytes = qam.framebytes

    gr.hier_block2.__init__(self, "ofdm_receive_path",
                            gr.io_signature(1, 1, gr.sizeof_gr_complex),
                            gr.io_signature(0, 0, 0))

    if options_rx.mode == "PNC":
      assert(options_rx.msg_type == PNC_TYPE)

    msgq_data = raw.msg_queue2() # use "unlimited" queue
    #msgq_data = gr.msg_queue() # use "unlimited" queue
    msg_sink = raw.message_sink(gr.sizeof_char * framebytes, options_rx.size, msgq_data, True)
    self.watcher_data = rx_queue_watcher_thread(msgq_data, callback)

    if options_rx.mode == "PNC":
      self.connect(self,
                   ofdm,
                   blocks.vector_to_stream(gr.sizeof_char, ofdm.params.data_tones),
                   qam,
                   blocks.stream_to_vector(gr.sizeof_char, qam.framebytes),
                   (msg_sink,0))
      #self.connect(self,
      #            ofdm)
      self.connect((ofdm, 1), blocks.stream_to_vector(gr.sizeof_char, ofdm.size), (msg_sink,1))   # control info
    elif options_rx.mode == "RAW":
      self.connect(self,
                   ofdm,
                   blocks.vector_to_stream(gr.sizeof_char, ofdm.params.data_tones),
                   qam,
                   blocks.stream_to_vector(gr.sizeof_char, qam.framebytes),
                   (msg_sink,0))
      #self.connect(self,
      #            ofdm)

      self.connect((ofdm, 1), blocks.stream_to_vector(gr.sizeof_char, ofdm.size), (msg_sink,1))   # control info
    #self.connect((ofdm, 1), blocks.null_sink(gr.sizeof_char))   # control info
    #self.connect((ofdm, 1), blocks.file_sink(gr.sizeof_char,'ofdm.datb'))   # control info

    self.ofdm = ofdm
    self.qam = qam
    self.sampler = ofdm.sampler
    self.sync = ofdm.sync
    self.fft = ofdm.fft
    self.framebytes = qam.framebytes

    if options_rx.mode == "PNC":
      print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ USE *ONE* DECODERS @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"

    if options_rx.log:
      # OFDM output
      if options_rx.mode == "PNC" or options_rx.mode == "RAW":
        self.connect(ofdm, blocks.file_sink(gr.sizeof_char*ofdm.params.data_tones, 'logs/rx-ofdmdemod.dat'))
        self.connect((ofdm,1), blocks.file_sink(gr.sizeof_char, 'logs/rx-ofdmdemod.datb'))

      # CD output
      if True:
        self.connect(qam, blocks.file_sink(gr.sizeof_char, 'logs/rx-qam.datb'))

  def carrier_sensed(self):
    """
    Return True if we think carrier is present.
    """
    pass

  def carrier_threshold(self):
    """
    Return current setting in dB.
    """
    pass

  def set_carrier_threshold(self, threshold_in_db):
    """
    Set carrier threshold.
    @param threshold_in_db: set detection threshold
    @type threshold_in_db:  float (dB)
    """
    pass
        
  def add_options(normal, expert):
    """
    Adds receiver-specific options to the Options Parser
    """
    expert.add_option("", "--bitrate", type="int", default=1,
                      help="set bitrate index [default=%default]")
    expert.add_option("","--sync-log", action="store_true", default=False,
                      help="log all samples after synchronizaiton")
    expert.add_option("", "--log", action="store_true", default=False,
                      help="log all source samples (CAUTION: lots of data)")
    ofdm_rxtx.OFDM_RX.add_options(normal, expert)
    ofdm_rxtx.FPNC_RX.add_options(normal, expert)
  # Make a static method to call before instantiation
  add_options = staticmethod(add_options)

  def _print_verbage(self):
    """
    Prints information about the receive path
    """
    pass

# /////////////////////////////////////////////////////////////////////////////
#                               Rx Queue Monitor
# /////////////////////////////////////////////////////////////////////////////
class rx_queue_watcher_thread(_threading.Thread):
  def __init__(self, q, callback=None):
    _threading.Thread.__init__(self)
    self.setDaemon(1)
    self.q = q
    self.callback = callback
    self.keep_running = True
    self.start()

  def done(self):
    self.keep_running = False

  def run(self):
    start = time.time()
    while self.keep_running:
      msg = self.q.delete_head()
      self.callback(msg)  # to ensure in-order delivery
      #_threading.Thread(target=self.callback, args=[msg]).start()

    finish = time.time()
    total = finish - start
    print "Queue: %d/%.2fs = %f" % (count, total, count / total)
    