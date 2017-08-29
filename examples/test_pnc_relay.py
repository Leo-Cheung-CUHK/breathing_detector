#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2014 Lizhao You
#
# Setup a Tx path and Rx path simutaneously
#  * Rx path for FPNC rx
#  * Tx path for RawOFDM tx
#

#import os
#print 'Blocked waiting for GDB attach (pid = %d)' % (os.getpid(),)
#raw_input ('Press Enter to continue: ')

import sys, time, copy, struct, operator, math, os
from gnuradio import eng_notation, gr, digital
from gnuradio.eng_option import eng_option
from optparse import OptionParser

import transceiver_phy
import phy_params, mac_params, mac_frame_handler
from phy_params import PNC_TYPE
from mac_params import MyQueue, string_to_hex_list, string_xor
from mac_params import CRC_LEN as CRCSIZE
from mac_params import TOTAL_ARQ_NSEQ as NSEQ
from mac_params import TOTAL_MAC_NSEQ as NSEQ_MAC

from mac_frame_handler import RelayParsePHYMsg, user_type_dict
import Queue

global timelog_writer, file_writer1, file_writer2
timelog_writer = None
file_writer1 = None
file_writer2 = None

# /////////////////////////////////////////////////////////////////////////////
#                                the mac protocol (MAC)
# /////////////////////////////////////////////////////////////////////////////
class relay(object):
  def __init__(self, verbose=False):
    self.verbose = verbose
    self.tb = None             # top block (access to PHY)

    self.beacon_no = 0
    self.xor_num = 0
    self.n_rcvd = 0
    self.n_right = 0
    self.x_right = 0
    self.a_right = 0
    self.b_right = 0
    self.tx_npkt = 0
    self.rx_npkt = 0

    self.pkt_relay_queue = Queue.Queue()

  def set_flow_graph(self, tb, options, fwriter):
    self.tb = tb
    self.options = copy.deepcopy(options)
    self.beacon_num = options.beacon_num
    self.beacon_pause = options.beacon_pause
    self.burst_num = options.burst_num
    self.burst_pause = options.burst_pause
    self.size = options.size                          # num of symbols
    self.framebytes = tb.framebytes
    self.beaconbytes = self.framebytes                # PKT_TYPE + BEACON# + CRC32 + PADDING # FIXME: make it general
    self.node = "R"
    self.mode = options.mode

    #if self.mode == "burst":
    #  self.beacon_num = 1

    self.bandwidth = options.bandwidth
    if options.tx_outfile is not None or options.tx_freq is not None: 
      self.tx_enable = True
    else:
      self.tx_enable = False
      
    self.init_pctime = 0
    self.init_hwtime = 0

    self.timelogger = fwriter[0]
    self.f1 = fwriter[1]
    self.f2 = fwriter[2]

    self.msg_type = options.msg_type
      
    payload = mac_params.make_binary_data(self.framebytes-CRCSIZE, ord('A'))
    tx_payload = digital.crc.gen_and_append_crc32(payload)
    self.txA = map(ord, tx_payload)
    
    payload = mac_params.make_binary_data(self.framebytes-CRCSIZE, ord('B'))
    tx_payload = digital.crc.gen_and_append_crc32(payload)
    self.txB = map(ord, tx_payload)
    
    self.ber = mac_frame_handler.calc_user_ber(self.txA, self.txB) # initialization
    self.berX = -1
    self.berA = -1
    self.berB = -1

    self.prt_msgq = MyQueue()
    self.watcher_prt = mac_params.print_queue_watcher_thread(self.prt_msgq.queue)
 
  def main_loop(self):
    self.prt_msgq.put("Tx: in main loop")
    time.sleep(3)
    [self.init_pctime, self.init_hwtime] = self.tb.get_init_time()
    ostr = "MAC GET INIT Time: init_hw_time = %.06f init_pctime = %.06f" % (self.init_hwtime, self.init_pctime)
    self.prt_msgq.put(ostr)

    if self.tx_enable:
      cur_pctime = time.time()
      cur_hwtime = self.init_hwtime + (cur_pctime-self.init_pctime)
      ts = cur_hwtime
      ostr = "[TX] ts = %.06f" % (ts)
      self.prt_msgq.put(ostr)

      tx_time = ts if ts is not None else None 

      if self.mode == "beacon":
        self.send_beacon(tx_time) 
        pass 
      if self.mode == "burst":
        assert(self.burst_pause is not None)
        self.beacon_pause = self.burst_pause * self.burst_num
        self.send_burst_beacon(tx_time)
        pass

  def send_burst_beacon(self,tx_time=None):
    # use timestamp to send
    assert(tx_time is not None)   
    numpkts = abs(int(self.beacon_num))
    #assert(numpkts == 1)  # default numpkts = 1

    tb = self.tb
    self.start_beacon_time = tx_time
    cur_pctime = time.time()
    tx_time = self.init_hwtime + (cur_pctime-self.init_pctime) + self.burst_pause

    while self.beacon_no < numpkts:
      self.beacon_no += 1

      pkt_type = struct.pack('!B', ord('B')) # beacon type 'B'
      no = self.beacon_no%NSEQ_MAC
      no = struct.pack('!B', no) # make beacon packet
      payload = pkt_type + no + struct.pack('!B', 0)
      payload += struct.pack('!B', ord('B'))*(self.beaconbytes - len(payload) - 4)
      payload = digital.crc.gen_and_append_crc32(payload)

      ostr = "[TX] beacon = %d %d %f" % (self.beacon_no%NSEQ_MAC, self.beacon_no, tx_time)
      self.prt_msgq.put(ostr)

      if tx_time is not None:
        tb.send_pkt(payload, tx_time)
      else:
        tb.send_pkt(payload)

      self.tx_npkt += 1

      WAIT_BURST_ST_TIME = self.burst_pause
      tx_time = tx_time + self.beacon_pause + WAIT_BURST_ST_TIME
      time.sleep(self.beacon_pause*0.98+WAIT_BURST_ST_TIME)

  def send_beacon(self, tx_time=None):
    # disable transmissions based on timestamp
    # to avoid conflict with XOR packets
    tb = self.tb
    if not self.options.time:
      tx_time = None

    self.start_beacon_time = tx_time

    # generate and send framing indicators
    numpkts = abs(int(self.beacon_num))

    # 'B'=0x42=b0100_0010 'X'=0x58=b0101_1000
    while self.beacon_no < numpkts:
      self.beacon_no += 1

      if tx_time is not None:
        tx_time = tx_time + self.beacon_pause
      else:
        tx_time = None

      try:
        pkt = self.pkt_relay_queue.get(False)  # do not block
        data = pkt[2:-4]
        pkt_type = pkt[0]
        no = self.beacon_no%NSEQ_MAC
        no = struct.pack('!B', no)
        payload = pkt_type + no + data
        payload = digital.crc.gen_and_append_crc32(payload)
        #print "A packet len=%d is in self.pkt_relay_queue qsize()=%d" %(len(payload), self.pkt_relay_queue.qsize())
      except:
        #print "none is in self.pkt_relay_queue"
        #pkt_type = struct.pack('!B', ord('B')) # beacon type 'B'   
        pkt_type = struct.pack('!B', ord('X')) # beacon type 'B'   
        no = self.beacon_no%NSEQ_MAC     
        no = struct.pack('!B', no) # make beacon packet
        payload = pkt_type + no + struct.pack('!B', 0)
        payload += struct.pack('!B', ord('B'))*(self.beaconbytes - len(payload) - CRCSIZE)
        payload = digital.crc.gen_and_append_crc32(payload)

      ostr = "[Tx] B#=%d %d ts=%.06f" % (self.beacon_no, (self.beacon_no%NSEQ), 0 if tx_time is None else tx_time)
      self.prt_msgq.put(ostr)

      # Beacon mode should not use timestamp, because downlink XOR does not have timestamps
      # They will conflict, and may degrade the performance
      if tx_time is not None:
        all_time = (tx_time - self.init_hwtime)
        cur_pctime = time.time()
        passed_time = cur_pctime - self.init_pctime
        sleep_time = (all_time-passed_time)
        #expire_time = (sleep_time + 0.7 * self.beacon_pause)

        tb.send_pkt(payload, tx_time)
        time_remain = (self.init_pctime + self.beacon_no*self.beacon_pause) - time.time()
        if sleep_time > 0:
          time.sleep(sleep_time*0.5)
      else:
        tb.send_pkt(payload)
        time.sleep(self.beacon_pause)
      self.tx_npkt += 1

  def phy_rx_callback(self, msg):  
    self.rx_npkt += 1
    now1 = time.time()
    pkt = RelayParsePHYMsg(msg,self.framebytes,self.size)

    pkt = pkt[0]
    self.n_rcvd += 1

    user_mode_fwd = 0

    if pkt.ok:
      self.n_right += 1
    
      #print "pkt.user_type = %d" % (pkt.user_type)
      if user_type_dict[pkt.user_type] == 'user_mode_X':
        self.x_right += 1
        user_mode_fwd = 3
      elif user_type_dict[pkt.user_type] == 'user_mode_A':
        self.a_right += 1
        user_mode_fwd = 1
      elif user_type_dict[pkt.user_type] == 'user_mode_B':
        self.b_right += 1
        user_mode_fwd = 2

    decoding_time = time.time()-self.init_pctime+self.init_hwtime-pkt.ts
    if self.msg_type == 0:
      #ostr = "[Rx] PILOT: ok=%s ts=%f|recv=%d xor=%d|snrA=%f cfoA=%f|snrB=%f cfoB=%f|time=%f:%f" \
      #  %(pkt.ok,pkt.ts,self.n_rcvd,self.n_right,pkt.snr_a,pkt.cfo_a,pkt.snr_b,pkt.cfo_b,
      #    decoding_time,time.time()-pkt.pc_ts)
      ostr = "[Rx] PILOT: ok=%s ts=%f|recv=%d %d=[%d %d %d]|snrA=%.02f cfoA=%.06f|snrB=%.02f cfoB=%.06f|time=%.06f:%.06f" \
        %(pkt.ok,pkt.ts,self.n_rcvd, self.n_right, self.x_right,self.a_right,self.b_right,pkt.snr_a,pkt.cfo_a,pkt.snr_b,pkt.cfo_b,
          decoding_time,time.time()-pkt.pc_ts)
    elif self.msg_type == 1:
      ostr = "[Rx] PILOT: ok=[%s %s %s] ts=%f|recv=%d xor=[%d %d %d]|snrA=%f cfoA=%f|snrB=%f cfoB=%f|time=%f:%f" \
        %(pkt.ok,pkt1.ok,pkt2.ok,pkt.ts,self.n_rcvd,self.n_right,self.a_right,self.b_right,pkt.snr_a,pkt.cfo_a,pkt.snr_b,pkt.cfo_b,
          decoding_time,time.time()-pkt.pc_ts)
    self.prt_msgq.put(ostr)

    if pkt.ok:
        pkt_type = struct.pack('!B', (ord('X') & 0xFC) | (user_mode_fwd & 0x3))
        payload = pkt_type + pkt.payload[1:]
        payload = digital.crc.gen_and_append_crc32(payload)
        #print "insert into queue pkt_type = 0x%02X  len=%d %s" %( ord(pkt_type), len(payload), string_to_hex_list(payload[0:16]) )
        self.pkt_relay_queue.put(payload)


    if self.timelogger is not None:
      now = time.time()
      fstr = "%f %f %f %f %f %f %f %d %d\n" % (now-pkt.pc_ts, now, pkt.pc_ts, self.tb.get_current_hwtime()-pkt.ts,self.tb.get_current_hwtime(),pkt.ts,now1,self.n_rcvd,self.n_right)
      self.timelogger.write(fstr)

    # reply xor pkt returned by PNC PHY
    start = time.time()
    if False:
      npkt = self.rx_npkt

      if self.mode == "burst" and npkt > 0 and npkt % self.burst_num == 0:
        pkt_type = struct.pack('!B', ord('X'))
      else:
        pkt_type = struct.pack('!B', ord('X'))

      if pkt.ok:
        payload = pkt_type + pkt.payload[1:]
        payload = digital.crc.gen_and_append_crc32(payload)
        self.xor_num += 1
        self.tb.send_pkt(payload)
        ostr = "[Tx] xor packet= %d tx_npkt= %d rx_npkt= %d " % (self.xor_num, self.tx_npkt, self.rx_npkt)
        self.prt_msgq.put(ostr)        
      else:
        if self.mode == "burst" and npkt > 0 and npkt % self.burst_num == 0:
          # send a beacon
          self.beacon_no += 1
          pkt_type = struct.pack('!B', ord('B')) # beacon type 'B'   
          no = self.beacon_no%NSEQ_MAC     
          no = struct.pack('!B', no) # make beacon packet
          payload = pkt_type + no + struct.pack('!B', 0)
          payload += struct.pack('!B', ord('B'))*(self.beaconbytes - len(payload) - CRCSIZE)
          payload = digital.crc.gen_and_append_crc32(payload)
          ostr = "Tx: beacon = %6d %6d =====" % ((self.beacon_no%NSEQ), self.beacon_no)
          self.prt_msgq.put(ostr)
          self.tb.send_pkt(payload)
 
      self.tx_npkt += 1

  def add_options(normal, expert_grp):
    expert_grp.add_option("","--log-time", action="store_true", default=False,
                          help="log all time infomation, defult is False")   
    expert_grp.add_option("", "--beacon-num", type="eng_float", default=1000,
                          help="set number of beacons to transmit [default=%default]")
    expert_grp.add_option("", "--beacon-pause", type="eng_float", default=0.3,
                          help="time interval between beacons [default=%default]")
    expert_grp.add_option("", "--burst-num", type="eng_float", default=1000,
                          help="set number of burst packets to be transmitted (for endnode) [default=%default]")    
    expert_grp.add_option("", "--burst-pause", type="eng_float", default=None,
                          help="time interval between regular pkts [default=%default]")
    expert_grp.add_option("", "--msg-type", type="eng_float", default="0",
                          help="set msg type [default=%default]")
    expert_grp.add_option("", "--mode", type="string", default="beacon",
                          help="set mode (beacon or burst) [default=%default]")
    expert_grp.add_option("", "--time", action="store_true", default=False,
                      help="set hw timed tx mode [PNC Beacon default=%default] ")
  # Make a static method to call before instantiation
  add_options = staticmethod(add_options)


# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////

def main():
  import signal
  def quit_gracefully(signum, frame):
    raise KeyboardInterrupt, "Signal handler"
  signal.signal(signal.SIGINT, quit_gracefully)

  parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
  expert_grp = parser.add_option_group("Expert")
  relay.add_options(parser, expert_grp)
  transceiver_phy.TXRX.add_options(parser, expert_grp)
  (options, args) = parser.parse_args ()

  # instantiate the MAC
  mac = relay(verbose=True)

  global timelog_writer, file_writer1, file_writer2
  if options.log_time:
    if os.path.isfile('timelogR.dat'):
      os.remove('timelogR.dat')
    timelog_writer=open('timelogR.dat','w')

  # build the graph
  import copy
  options_tx = copy.deepcopy(options)
  options_tx.mode = "RAW"
  options_tx.precoding = "NO"
  options_rx = copy.deepcopy(options)
  options_rx.mode = "PNC"
  tb = transceiver_phy.TXRX(options_tx, options_rx, mac.phy_rx_callback)
  fwriter = [timelog_writer, file_writer1, file_writer2]
  mac.set_flow_graph(tb, options, fwriter)  # give the MAC a handler for the PHY

  #r = gr.enable_realtime_scheduling()
  #if r != gr.RT_OK:
  #  print "Warning: failed to enable realtime scheduling"

  try:
    if options_tx.mode == "RAW":
      print "INIT: set RAW tx option"
    elif options_tx.mode == "PNC":
      print "INIT: set PNC tx option"
    if options_rx.mode == "RAW":
      print "INIT: set RAW rx option"
    elif options_rx.mode == "PNC":
      print "INIT: set PNC rx option"
    assert(options.node == None)

    tb.start()                       # start flow graph

    if options.profile:
      tb.ctrlport_monitor_performance.start()
      tb.ctrlport_monitor.start()

    mac.main_loop()                  # don't expect this to return...
  except KeyboardInterrupt:
    tb.stop()
    os.system('mv -fv /tmp/*.dat ./')


  tb.wait()                          # wait for it to finish
  os.system('mv -fv /tmp/*.dat ./')

if __name__ == '__main__':
  try:
    main()
    if file_writer1 is not None:
      file_writer1.close()
      file_writer2.close()
    if timelog_writer is not None:
      timelog_writer.close()
  except KeyboardInterrupt:
    if file_writer1 is not None:
      file_writer1.close()
      file_writer2.close()
    if timelog_writer is not None:
      timelog_writer.close()
    pass
