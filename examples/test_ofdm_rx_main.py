#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
# Copyright 2014 Lizhao You
#

from gnuradio import gr, digital, blocks
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from optparse import OptionParser

from uhd_interface import uhd_receiver
import receive_path

import raw, time, struct

import mac_params
from mac_params import string_to_hex_list

#import os
#print 'Blocked waiting for GDB attach (pid = %d)' % (os.getpid(),)
#raw_input ('Press Enter to continue: ')

global f
f = None #open('p2p_end_time.dat','w')

class my_top_block(gr.top_block):
  """
    Three modes of operation:
      usrp -> outfile
      infile -> rxdata
      usrp -> rxdata
  """
  def __init__(self, options, callback=None):
    gr.top_block.__init__(self)
    if options.freq is not None:
      u = uhd_receiver(options.rx_args,
                       options.bandwidth,
                       options.rx_freq, options.rx_gain,
                       options.spec, options.antenna,
                       options.verbose, options.external)
      if options.bandwidth > 5e6:
        u.u.set_min_output_buffer(163840)

    elif options.infile is not None:
      u = blocks.file_source(gr.sizeof_gr_complex, options.infile)
    else:
      import sys
      sys.stderr.write("--freq or --infile must be specified\n")
      raise SystemExit

    rx = receive_path.ofdm_receive_path(options, self.phy_rx_callback)
    framebytes = rx.framebytes
    self.connect(u, rx)

    if options.profile:
      from gnuradio.ctrlport import monitor
      self.ctrlport_monitor = monitor()
      self.ctrlport_monitor_performance = monitor("gr-perf-monitorx")

    self.prt_msgq = mac_params.MyQueue()
    self.watcher_prt = mac_params.print_queue_watcher_thread(self.prt_msgq.queue)
    
  def phy_rx_callback(self, msg):
    """
    Invoked by thread associated with PHY to pass received packet up.

    @param ok: bool indicating whether payload CRC was OK
    @param payload: contents of the packet (string)
    """
    ts = msg.timestamp_sec() + msg.timestamp_frac_sec() if msg.timestamp_valid() else None
    (snr,) = msg.get_snr_values()
    (cfo,) = msg.cfo_values()
    global n_rcvd, n_right
    n_rcvd = n_rcvd + 1

    pkt = msg.to_string()
    (ok,payload)=digital.crc.check_crc32(pkt)
    (pkt_type,) = struct.unpack('!B', payload[0:1])
    (pktno,) = struct.unpack('!H', payload[1:3])
    #print "Receive payload is", map(ord,payload[0:50])

    decode_time = msg.get_decode_time()
    if ok :
      n_right = n_right+1
      #end[pktno]= time.time() 
      global f
      if f is not None:
        log = "%d %.06f %.06f\n" % (pktno, decode_time, time.time())
        f.write(log)

    ostr = "Rx: %r type:%s %d/%d ts=%.06f snr=%.01f cfo=%.06f decode_time=%.06fs\n    len=%d %s ... %s" \
          % (ok, chr(pkt_type), n_right, n_rcvd, ts, snr, cfo, decode_time, len(pkt), string_to_hex_list(pkt[0:16]), string_to_hex_list(pkt[-4:-1]))
    self.prt_msgq.put(ostr)

  def add_options(normal, expert):
    normal.add_option("", "--infile", type="string",
                      help="select input file")
    normal.add_option("", "--outfile", type="string",
                      help="select output file (raw)")
    normal.add_option("", "--rxdata", type="string",
                      help="data file (demodulated)")
    normal.add_option("-v", "--verbose", action="store_true", default=False)
    normal.add_option("-W", "--bandwidth", type="eng_float", default=1e6,
                      help="set symbol bandwidth [default=%default]")
    expert.add_option("", "--log", action="store_true", default=False,
                      help="Log all parts of flow graph to files (CAUTION: lots of data)")
    expert.add_option("", "--profile", action="store_true", default=False,
                      help="enable gr-ctrlport to monitor performance")
    uhd_receiver.add_options(normal)
    receive_path.ofdm_receive_path.add_options(normal, expert)
  # Make a static method to call before instantiation
  add_options = staticmethod(add_options)


# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////

def main():
  global n_rcvd, n_right, f
  n_rcvd = 0
  n_right = 0

  end = [0.0 for x in range(10000)]
  import signal
  def quit_gracefully(signum, frame):
    raise KeyboardInterrupt, "Signal handler"
  signal.signal(signal.SIGINT, quit_gracefully)



  parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
  expert_grp = parser.add_option_group("Expert")

  my_top_block.add_options(parser, expert_grp)

  (options, args) = parser.parse_args ()
  options.mode = "RAW"

  # build the graph
  tb = my_top_block(options)

  r = gr.enable_realtime_scheduling()
  if r != gr.RT_OK:
    print "Warning: failed to enable realtime scheduling"

  try:
    tb.start()                      # start flow graph

    if options.profile:
      tb.ctrlport_monitor_performance.start()
      tb.ctrlport_monitor.start()

  except KeyboardInterrupt:
    tb.stop()
  tb.wait()                       # wait for it to finish

if __name__ == '__main__':
  try:
    main()
    if f is not None:
      f.close()
  except KeyboardInterrupt:
    if f is not None:
      f.close()
    pass

