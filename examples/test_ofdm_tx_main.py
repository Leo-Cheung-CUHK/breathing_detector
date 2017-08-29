#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
# Copyright 2014 Lizhao You
#
# OFDM Tx Main 
#   * Mode 1: example_tx_sample_file
#   * Mode 2: example_tx_bits
#

from gnuradio import gr, blocks
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from gnuradio import digital
from optparse import OptionParser

from uhd_interface import uhd_transmitter
import transmit_path

import time, sys, struct, random

import raw

#import os
#print os.getpid()
#raw_input("Press Enter to continue")

global f
f = None #open('p2p_start_time.dat','w')

class my_top_block(gr.top_block):
  """
    Two modes of operation:
      txdata -> outfile
      txdata -> usrp
  """
  def __init__(self, options):
    gr.top_block.__init__(self)
    if options.freq is not None:
      u = uhd_transmitter(options.tx_args,
                          options.bandwidth,
                          options.tx_freq, options.tx_gain,
                          options.spec, options.antenna,
                          options.verbose, options.external)
    elif options.outfile is not None:
      u = blocks.file_sink(gr.sizeof_gr_complex, options.outfile)
    else:
      raise SystemExit("--freq or --outfile must be specified\n")

    tx = transmit_path.ofdm_transmit_path(options)
    amp = blocks.multiply_const_cc(options.tx_amplitude)

    self.connect(tx,amp,u)

    self.tx = tx
    self.u = u
    self.outfile = options.outfile

  # Provide general send_pkt API
  def send_pkt(self, payload='', time=None, eof=False, msgq_tx_enable=False):
    if eof and not msgq_tx_enable:
      self.tx.send_eof()
    elif eof and msgq_tx_enable:
      self.tx.send_eof2()

    if msgq_tx_enable:
      self.tx.send_samples(payload, time)
    else:
      self.tx.send_pkt(payload, time)
  
  def get_current_hwtime(self):
    if self.outfile is not None:
      return time.time()
    else:
      usrp_time, frac = self.u.get_usrp_time()
      curtime = usrp_time+frac
      return curtime

  def add_options(normal, expert):
    """
    Adds usrp-specific options to the Options Parser
    """
    normal.add_option("", "--outfile", type="string",
                      help="select output file to modulate to")
    normal.add_option("-N", "--numpkts", type="eng_float", default=0,
                      help="set number of packets to transmit [default=%default]")
    normal.add_option("","--pause", type="eng_float", default=0.005,
                      help="inter-burst pause [s]")
    normal.add_option("-W", "--bandwidth", type="eng_float", default=1e6, 
                      help="set symbol bandwidth [default=%default]")
    normal.add_option("", "--tx-amplitude", type="eng_float", default=0.1, metavar="AMPL",
                      help="set transmitter digital amplitude: 0 <= AMPL < 1.0 [default=%default]")
    expert.add_option("", "--time", action="store_true", default=False,
                      help="set timed tx mode [RawOFDM default=%default] ")
    uhd_transmitter.add_options(normal)
    transmit_path.ofdm_transmit_path.add_options(normal, expert)
  # Make a static method to call before instantiation
  add_options = staticmethod(add_options)

# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////

class MAC_TIMER:
  '''
  @Jan 27, 2014 by Lizhao
  Want to simulate a simple CSMA/CA protocol (only one user)
  But USRP could not support such tight timing
  In other words, the time that samples goes to USRP is larger than self.pause
  Here self.pause > 1ms but < 1.5ms. If I use 1.5ms, then most of time is okay.
  That means the transfer latency is about 0.5ms
  '''
  def __init__(self,start,GAP):
    self.pause = GAP
    self.time  = start
    print "%f %f \n" % (start, self.time)
    self.tSlot = 9 * 0.000001
    self.tPkt  = 260 * 4 * 0.000001  # assume 20MHz
    self.tDIFS = 34 * 0.000001
    self.cnt   = 0

  def get_next_txtime(self):
    self.cnt += 1
    if self.cnt > 1:
      t = random.randint(1,32)
      self.pause = self.tPkt + self.tDIFS + t*self.tSlot
    self.time = self.time + self.pause
    return [self.time, self.pause]

def main():
  global f

  def string_to_hex_list(s):
    return map(lambda x: hex(ord(x)), s)

  def example_tx_sample_file(tb, options):
    time.sleep(3)

    MAX_READ_BYTES = 1000000000    # max number of bytes to be read in a file
    file_object = open(options.data_file)
    rawdata = file_object.read(MAX_READ_BYTES)
    print "Length of payload = ", len(rawdata), " | MAX_READ = ", MAX_READ_BYTES

    #num_floats = len(rawdata)/4
    #payload=struct.unpack('f'*num_floats,rawdata)
    #txdata=struct.pack('f'*self.num_floats)
    txdata = rawdata
    file_object.close()   

    start = time.time()
    if options.time:
      usrp_time = tb.get_current_hwtime()
      print "Current USRP Time: ", usrp_time
      startTime = usrp_time
    GAP = options.pause
    framebytes = tb.tx.framebytes
    numpkts = 10000 if options.numpkts == 0 else abs(int(options.numpkts))
    start = [0.0 for x in range(numpkts+1)]

    # to use a MAC protocol
    #timerx = MAC_TIMER(startTime,GAP)

    pktno = 1
    while pktno <= numpkts:
      if options.time:
        #[tx_ts, pause_ts] = timerx.get_next_txtime()
        tx_ts = startTime+pktno*GAP
        pause_ts = GAP
        start[pktno]= tx_ts
        tb.send_pkt(txdata, tx_ts, eof=False, msgq_tx_enable=True)
        print "Send pkt# %d time:%f ..." % (pktno, startTime+pktno*GAP)
        time.sleep(0.5*GAP)
      else:
        tb.send_pkt(txdata, eof=False, msgq_tx_enable=True)
        print "Send pkt# %d" % (pktno)
        start[pktno]= time.time()
        if (GAP > 0.0):
          sys.stdout.flush()
          time.sleep(GAP*0.95)

      log = "%d %.6f \n" % (pktno, start[pktno])
      global f
      if f is not None:
        f.write(log)
      pktno = pktno + 1
      sys.stdout.write('.')
        
    tb.send_pkt(eof=True, msgq_tx_enable=True) # no tx.ofdm blocks, so disable eof for tx_samples
    elapsed = start[numpkts] - start[1]
    print "Sent %d packets of %d symbols in %.2f seconds" % (pktno-1, options.size, elapsed)
    if elapsed != 0:
      print pktno*options.size / elapsed
    time.sleep(3)

  def example_tx_bits(tb, options):
    time.sleep(3)

    start = time.time()
    if options.time:
      usrp_time = tb.get_current_hwtime()
      print "Current USRP Time: ", usrp_time
      startTime = usrp_time

    GAP = options.pause
    framebytes = tb.tx.framebytes
    numpkts = 10000 if options.numpkts == 0 else abs(int(options.numpkts))
    start = [0.0 for x in range(numpkts+1)]
    pktno = 1
    while pktno <= numpkts:
      pkt_type = struct.pack('!B', ord('B'))
      payload = pkt_type + struct.pack('!H', pktno)+ struct.pack('!H', 0)
      #random.seed()
      #for i in range(framebytes - len(payload) - 4):
      #  r = random.randint(0,255)
      #  payload += struct.pack('!B', r)
      for i in range(framebytes - len(payload) - 4):
        payload += struct.pack('!B', (i+1)%255 )
      #payload += struct.pack('!B', 0xC6) * (framebytes - len(payload) - 4)

      payload = digital.crc.gen_and_append_crc32(payload)

      if options.time:
        tb.send_pkt(payload, startTime+pktno*GAP)
        print "Send pkt# %d time:%f ..." % (pktno, startTime+pktno*GAP)
        start[pktno]= startTime+pktno*GAP
        time.sleep(GAP*0.5)
      else:
        tb.send_pkt(payload)
        print "Send pkt# %d ..." % (pktno)
        start[pktno]= time.time()
        if (GAP > 0.0):
          sys.stdout.flush()
          time.sleep(GAP*0.95)
        log = "%d %.6f \n" % (pktno, start[pktno])
        global f
        if f is not None:
          f.write(log)
      pktno = pktno + 1
      sys.stdout.write('.')
        
    tb.send_pkt(eof=True)
    elapsed = start[numpkts] - start[1]
    print "Sent %d packets of %d symbols in %.2f seconds" % (pktno-1, options.size, elapsed)
    if elapsed != 0:
        print pktno*options.size / elapsed
    time.sleep(3)

  parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
  expert_grp = parser.add_option_group("Expert")

  my_top_block.add_options(parser, expert_grp)

  (options, args) = parser.parse_args()
  options.mode = "RAW"  # for compatibility

  # build the graph
  tb = my_top_block(options)

  r = gr.enable_realtime_scheduling()
  if r != gr.RT_OK:
    print "Warning: failed to enable realtime scheduling"

  try:
    tb.start()                      # start flow graph
  
    if options.data_file:
      example_tx_sample_file(tb, options)
    else:
      example_tx_bits(tb, options)  
  except KeyboardInterrupt:
    tb.stop()

  print "Wait flowgraph to stop ..."
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

