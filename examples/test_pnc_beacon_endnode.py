#!/usr/bin/env python
#
# Copyright 2014 Lizhao You
#
# PNC Endnode Beacon Transmission Mode 
#
# Setup a Tx path and Rx path simutaneously
#   * Rx path for OFDM rx (based on rawofdm)
#   * Tx path for FPNC tx
# BEACON MSGQ TX mode: one beacon triggers one transmission.

#import os
#print 'Blocked waiting for GDB attach (pid = %d)' % (os.getpid(),)
#raw_input ('Press Enter to continue: ')

from gnuradio import gr, eng_notation, digital
from gnuradio.eng_option import eng_option
import sys, time, copy, struct, random, threading, os
from optparse import OptionParser

import transceiver_phy, phy_params, mac_frame_handler, mac_params
from mac_params import MAC_HEADER_LEN, CRC_LEN
from mac_params import MyQueue, string_to_hex_list, string_to_ord_list
from mac_frame_handler import EndNodeParsePHYMsg, node_type_dict

import Queue

MAX_READ_BYTES = 1000000000    # max number of bytes to be read in a file
MAX_NUM_PKTS = 100000          # max number of tx/rx pkts in this system
MAX_TOLERABLE_TIME = 100       # max tolerable time to guarantee within CP 

# /////////////////////////////////////////////////////////////////////////////
#                                The MAC Protocol
# /////////////////////////////////////////////////////////////////////////////
class endnode(object):    
  def __init__(self, verbose=False):
    self.verbose = verbose
    self.tb = None                            # top block (access to PHY)

    self.init_pctime = 0
    self.init_hwtime = 0

    # CFO precoding relevant
    self.cfo_pkt_thre = 1
    self.alpha = 0.1
    self.accu_cfo = 0

    # statistic data
    self.num_rx_all = 0
    self.num_rx_bcn = 0
    self.num_rx_xor = 0
    self.num_tx     = 0

    self.start_time = 0

    # printer info
    self.prt_msgq = MyQueue()
    self.watcher_prt = mac_params.print_queue_watcher_thread(self.prt_msgq.queue)

    self.f = None #open('pnc_end_time.dat','w')

  def set_flow_graph(self, tb, options, fwriter):
    self.tb = tb
    self.framesize = tb.framebytes                            # FIXME: make beacon length more general
    self.pktsize = self.framesize-MAC_HEADER_LEN-CRC_LEN
    self.pause = options.pause                                # trigger GAP
    self.burst_num = options.burst_num
    self.node  = options.node
    self.data_file = options.data_file

    self.f1 = fwriter[0]
    self.f2 = fwriter[1]
    self.timelogger = fwriter[2]

    self.rate_start_time = 0
    self.rate_total_bits = 0
    self.rate_avg = 0;
    self.rate_instant = 0;
    self.rate_instant_time_queue = Queue.Queue()
    self.rate_instant_time_max = 20


    # -------------------------------------------------------------------------- #
    # debug mode with msgtx - random bits or raw samples
    if options.data_file is not None:
      file_object = open(options.data_file)
      rawdata = file_object.read(MAX_READ_BYTES)
      print "Length of payload = ", len(rawdata), " | MAX_READ = ", MAX_READ_BYTES

      self.num_floats = len(rawdata)/4
      self.test_frame=struct.unpack('f'*self.num_floats,rawdata)
      file_object.close()
    else:
      payload = mac_params.make_binary_data(self.framesize-CRC_LEN, ord(self.node))
      payload = digital.crc.gen_and_append_crc32(payload)
      self.test_frame = payload
      print "TEST FRAME", len(payload), string_to_hex_list(payload[-32:]), string_to_ord_list(payload[-32:])
    # -------------------------------------------------------------------------- #

  def main_loop(self):
    print "INIT: in main loop"
    print "TEST: send a frame", len(self.test_frame)
    if self.data_file is None:
      self.tb.send_pkt(self.test_frame)  
    else:
      time.sleep(3)
      [self.init_pctime, self.init_hwtime] = self.tb.get_init_time()
      cur_pctime = time.time()
      cur_hwtime = self.init_hwtime + (cur_pctime-self.init_pctime)  
      self.msgq_tx(cur_hwtime)

  def msgq_tx(self, timestamp):
    random_scale_data = [p * 1 for p in self.test_frame]
    txdata=struct.pack('f'*self.num_floats, *random_scale_data)  # keep *
    for cnt in range(1):
        tx_ts = timestamp+(cnt+1)*self.pause
        all_time = (tx_ts - self.init_hwtime)
        cur_pctime = time.time()
        passed_time = cur_pctime - self.init_pctime
        sleep_time = 0.5*(all_time-passed_time)
        if sleep_time > 0:
          time.sleep(sleep_time)
        self.tb.send_pkt(txdata, tx_ts, eof=False, msgq_tx_enable=True)
        print "[TEST] Send frame#", cnt, " | ts=", tx_ts
  
  def phy_rx_callback_msgq_tx(self, msg):
    """
    Invoked by thread associated with PHY to pass received packet up.
    Used with MSGQ TX mode: insert modulated samples, and transmit them with random amplitude.
    
    @param msg: received frame with crc32 
    """
    frame = EndNodeParsePHYMsg(msg,self.framesize,self.node)

    self.num_rx_all += 1    
    self.accu_cfo = (1 - self.alpha)*self.accu_cfo + self.alpha*frame.cfo
    if self.num_rx_all <= self.cfo_pkt_thre: # ignore 1st frame, do nothing
      [self.init_pctime, self.init_hwtime] = self.tb.get_init_time()
      print "INIT Time = ", self.init_hwtime, self.init_pctime
      pass 
    else:
      self.tb.set_nco(self.accu_cfo)           # CFO compensation
    
    random_scale_data = [p for p in self.test_frame]
    txdata=struct.pack('f'*self.num_floats, *random_scale_data)  # keep *

    if self.timelogger is not None and frame.ok:
      now = time.time()
      fstr = "%f %f %f %f %f %f\n" % (now-frame.pc_ts, now, frame.pc_ts, self.tb.get_current_hwtime()-frame.ts,self.tb.get_current_hwtime(),frame.ts)
      self.timelogger.write(fstr)

    if frame.ok and frame.type == ord('B'):
      self.num_rx_bcn += 1
      for cnt in range(self.burst_num):
        tx_ts = frame.ts+(cnt+1)*self.pause
        self.tb.send_pkt(txdata, tx_ts, eof=False, msgq_tx_enable=True) 
        print "Send pkt#", cnt, " | ts=", tx_ts
    elif frame.ok and frame.type == ord('X'):
      self.num_rx_xor += 1

    ostr = "[Rx] ok=%r rx=%d(B%d X%d) ts=%f cfo=%6f snr=%f" % (frame.ok,self.num_rx_all, self.num_rx_bcn, self.num_rx_xor, frame.ts, frame.cfo, frame.snr)
    self.prt_msgq.put(ostr)

  def phy_rx_callback_bits_tx(self, msg):
    """
    Invoked by thread associated with PHY to pass received packet up.
    Used with NORMAL TX mode: insert bits, modulate, and transmit.
    
    @param msg: received frame with crc32 and other info
    """
    frame = EndNodeParsePHYMsg(msg,self.framesize,self.node)

    self.num_rx_all += 1
    self.accu_cfo = (1 - self.alpha)*self.accu_cfo + self.alpha*frame.cfo
    if self.num_rx_all <= self.cfo_pkt_thre: # ignore 1st frame, do nothing
      [self.init_pctime, self.init_hwtime] = self.tb.get_init_time()
      print "INIT Time = ", self.init_hwtime, self.init_pctime
      pass 
    else:
      self.tb.set_nco(self.accu_cfo)           # CFO compensation
    
    now = time.time()
    if self.timelogger is not None:
      now = time.time()
      fstr = "%f %f %f %f %f %f\n" % (now-frame.pc_ts, now, frame.pc_ts, self.tb.get_current_hwtime()-frame.ts,self.tb.get_current_hwtime(),frame.ts)
      self.timelogger.write(fstr)

    if self.f is not None:
      log = "%.06f %.06f %.06f\n" % (frame.ts, frame.decode_itv, time.time())
      self.f.write(log)

    pkt_type = frame.type & 0xFC
    if frame.ok and pkt_type == ord('X'): # type 'X' is for BEACON
      self.num_rx_bcn += 1

      if self.num_rx_bcn == 1:
        self.start_time = time.time()

      tx_payload = self.test_frame
      tx_frame_ts = frame.ts+self.pause      
      self.tb.send_pkt(tx_payload, tx_frame_ts)
      self.num_tx += 1
      #print "len()=%d %s" % (len(tx_payload), string_to_hex_list(tx_payload))
      
      rate_cur_time = time.time()  
      user_type_indict = frame.user_type & 0x3
      if user_type_indict > 0 and user_type_indict!=node_type_dict[self.node]:
        self.rate_total_bits += self.framesize*8  #convert to bits
      if self.rate_start_time == 0:
        self.rate_start_time = rate_cur_time
      else:
        rate_time_interval = rate_cur_time - self.rate_start_time
        self.rate_avg = self.rate_total_bits * 1.0 / rate_time_interval

      if user_type_indict > 0 and user_type_indict!=node_type_dict[self.node]:
        self.rate_instant_time_queue.put(rate_cur_time)
      if self.rate_instant_time_queue.qsize() < self.rate_instant_time_max:
        self.rate_instant = 0
      else:
        rate_time_interval = rate_cur_time - self.rate_instant_time_queue.get()
        self.rate_instant = self.framesize*8.0 * self.rate_instant_time_max/rate_time_interval


      ostr = "[Rx] B#=%d rx=%d(B%d X%d) ts=%.06f cfo=%.06f snr=%.01f decitv=%.06f rate=%.03f/%.03fkbps" % \
      (frame.beacon_no,self.num_rx_all, self.num_rx_bcn, self.num_rx_xor, frame.ts, frame.cfo, frame.snr, frame.decode_itv, self.rate_avg/1000.0, self.rate_instant/1000.0)
      self.prt_msgq.put(ostr)

      ostr = "[Tx] P#=%d ts=%.06f" % (self.num_tx,tx_frame_ts)
      self.prt_msgq.put(ostr)

    #elif frame.ok and frame.type == ord('X'):
    #  self.num_rx_xor += 1
    #
    #  tx_beacon_no = frame.x_beacon_no
    #  self_data = self.test_frame
    #  frame_data = frame.payload
    #  dcod_data = ''
    #
    #  for i in range(len(frame_data)):
    #    dcod_data += chr(ord(frame_data[i])^ord(self_data[i]))
    #
    #  ostr1 = "[RX] X#=%d " % (frame.beacon_no)
    #  ostr2 = "header of dcod_data is: " + str(map(ord,dcod_data[3:11])) + " " + str(map(ord,frame.payload[3:11]))
    #  self.prt_msgq.put(ostr1+ostr2)

    #  ostr = "[Rx] X#=%d rx=%d(B%d X%d) ts=%f cfo=%f snr=%f" % \
    #  (frame.beacon_no, self.num_rx_all, self.num_rx_bcn, self.num_rx_xor, frame.ts, frame.cfo, frame.snr)
    #  self.prt_msgq.put(ostr)

    else:
      ostr = "[Rx] False %s#=%d rx=%d(B%d X%d) ts=%f cfo=%6f snr=%f" % \
      (chr(frame.type), frame.beacon_no, self.num_rx_all, self.num_rx_bcn, self.num_rx_xor, frame.ts, frame.cfo, frame.snr)
      self.prt_msgq.put(ostr)

  def add_options(normal, expert_grp):
    normal.add_option("", "--pause", type="eng_float", default=0.1,
                          help="time interval between regular frames [default=%default]")
    expert_grp.add_option("", "--burst-num", type="eng_float", default=1,
                          help="set number of burst packets per Transmission [default=%default]")
    expert_grp.add_option("","--log-time", action="store_true", default=False,
                          help="log all timing infomation, defult is False")      
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
  endnode.add_options(parser, expert_grp)
  transceiver_phy.TXRX.add_options(parser, expert_grp)
  (options, args) = parser.parse_args ()

  # instantiate the MAC
  mac = endnode(verbose=True)

  timelog_writer = None
  if options.log_time:
    if options.node == 'A':
      if os.path.isfile('timelogA.dat'):
        os.remove('timelogA.dat')
      timelog_writer=open('timelogA.dat','w')
    if options.node == 'B':
      if os.path.isfile('timelogB.dat'):
        os.remove('timelogB.dat')
      timelog_writer=open('timelogB.dat','w')
  
  file_writer1 = None
  file_writer2 = None
  if options.data_file is not None:
    callback = mac.phy_rx_callback_msgq_tx
  else:
    callback = mac.phy_rx_callback_bits_tx
  
    # check node type
    if options.node is None:
      print "ERROR: please specify node type, A or B \n"
      exit(-1)

  # build the graph (PHY)
  # TODO: remove mode option
  options_tx = copy.deepcopy(options)
  options_tx.mode = "PNC"
  options_rx = copy.deepcopy(options)
  options_rx.mode = "RAW"
  options_rx.msg_type = phy_params.PNC_TYPE
  tb = transceiver_phy.TXRX(options_tx, options_rx, callback)
  fwriter = [file_writer1, file_writer2, timelog_writer]
  mac.set_flow_graph(tb, options, fwriter)            # give the MAC a handle for the PHY

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

    # GR3.6.5 max_noutput_items=100000
    # GR3.7.8 max_noutput_items=100000000
    tb.start(1000000000)                  # start flow graph
    if options.profile:
      tb.ctrlport_monitor_performance.start()
      tb.ctrlport_monitor.start()

    mac.main_loop()
  except KeyboardInterrupt:
    print "***************** FUNCTION CTRL INTERUPT *****************"
    tb.stop()
    os.system('mv -fv /tmp/*.dat ./')#mv -fv /tmp/*.dat ./')
    if file_writer1 is not None:
      file_writer1.close()
      file_writer2.close()


  tb.wait()                     # wait for it to finish
  os.system('mv -fv /tmp/*.dat ./')

if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    print "****************** MAIN CTRL INTERUPT *****************"
    os.system('mv -fv /tmp/*.dat ./')
    pass