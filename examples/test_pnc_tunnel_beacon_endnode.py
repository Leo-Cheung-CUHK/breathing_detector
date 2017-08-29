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

import TunnelARQ

import Queue

MAX_READ_BYTES = 1000000000    # max number of bytes to be read in a file
MAX_NUM_PKTS = 100000          # max number of tx/rx pkts in this system
MAX_TOLERABLE_TIME = 100       # max tolerable time to guarantee within CP 

# /////////////////////////////////////////////////////////////////////////////
#
#   Use the Universal TUN/TAP device driver to move packets to/from kernel
#
#   See /usr/src/linux/Documentation/networking/tuntap.txt
#
# /////////////////////////////////////////////////////////////////////////////

# Linux specific...
# TUNSETIFF ifr flags from <linux/tun_if.h>

IFF_TUN   = 0x0001   # tunnel IP packets
IFF_TAP   = 0x0002   # tunnel ethernet frames
IFF_NO_PI = 0x1000   # don't pass extra packet info
IFF_ONE_QUEUE = 0x2000   # beats me ;)

def open_tun_interface(tun_device_filename):
    from fcntl import ioctl
    
    mode = IFF_TAP | IFF_NO_PI
    TUNSETIFF = 0x400454ca

    tun = os.open(tun_device_filename, os.O_RDWR)
    ifs = ioctl(tun, TUNSETIFF, struct.pack("16sH", "gr%d", mode))
    ifname = ifs[:16].strip("\x00")
    return (tun, ifname)

# /////////////////////////////////////////////////////////////////////////////
#                                The MAC Protocol
# /////////////////////////////////////////////////////////////////////////////
class cs_mac(object):
  """
    Prototype carrier sense MAC

    Reads packets from the TUN/TAP interface, and sends them to the PHY.
    Receives packets from the PHY via phy_rx_callback, and sends them
    into the TUN/TAP interface.

    Of course, we're not restricted to getting packets via TUN/TAP, this
    is just an example.
  """
  def __init__(self, tun_fd, verbose=False):
    self.tun_fd = tun_fd       # file descriptor for TUN/TAP interface
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

    self.arq_mode = options.arq_mode
    self.buffer = TunnelARQ.NullARQBuffer("",self.node,self.framesize,arq_mode=self.arq_mode,prt_msgq=self.prt_msgq)
    self.buffer.setTunnelMode()

    if self.node == 'B':
      self.pause += 160.0/options.bandwidth

  def phy_rx_callback(self, msg):
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

      tx_payload = self.buffer.extractTxFrame(beacon_seq=frame.beacon_no)
      tx_frame_ts = frame.ts+self.pause
      self.tb.send_pkt(tx_payload, tx_frame_ts)
      self.num_tx += 1
      #print "len()=%d %s" % (len(tx_payload), string_to_hex_list(tx_payload))

      ostr = "[Tx] P#=%d ts=%.06f" % (self.num_tx,tx_frame_ts)
      self.prt_msgq.put(ostr)
    
      user_type_indict = frame.user_type & 0x3

      if user_type_indict == node_type_dict['X']:
        self.num_rx_xor += 1
        tx_beacon_no = frame.x_beacon_no
        self_data = self.buffer.getTxFrame(tx_beacon_no)
        frame_data = frame.payload
        dcod_data = ""

        #print "Tunnel callback  len(frame_data)=%d/len(self_data)=%d beacon_no=%d" %(len(frame_data), len(self_data), tx_beacon_no)
        try:
          for i in range( min(len(frame_data), len(self_data))):
            dcod_data += chr(ord(frame_data[i])^ord(self_data[i]))

          rx = self.buffer.updateRxQueue(dcod_data,frame.x_beacon_no)
          if rx is not None and len(rx) > 0:
            #ostr = "[DEBUG] type=%s len=%d %s" % (type(rx), len(rx), string_to_hex_list(rx[:20]))
            #self.prt_msgq.put(ostr)
            os.write(self.tun_fd, rx)
        except:
          pass

      elif user_type_indict > 0 and user_type_indict != node_type_dict[self.node]:
        dcod_data = frame.payload
        rx = self.buffer.updateRxQueue(dcod_data,frame.x_beacon_no)
        if rx is not None and len(rx) > 0:
          os.write(self.tun_fd, rx)

    #  ostr1 = "[RX] X#=%d " % (frame.beacon_no)
    #  ostr2 = "header of dcod_data is: " + str(map(ord,dcod_data[3:11])) + " " + str(map(ord,frame.payload[3:11]))
    #  self.prt_msgq.put(ostr1+ostr2)

      rate_cur_time = time.time()

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

  def main_loop(self):
    """
        Main loop for MAC.
        Only returns if we get an error reading from TUN.
    """
    while 1:
      payload = os.read(self.tun_fd, 10*1024)
      if not payload:
        print ">>> Error: End of Tx"
        break

      self.buffer.bufferUpdate(payload)

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
  cs_mac.add_options(parser, expert_grp)
  parser.add_option("-v","--verbose", action="store_true", default=False)
  parser.add_option("", "--pause", type="eng_float", default=0.1,
                      help="time interval between regular frames [default=%default]")
  parser.add_option("", "--arq-mode", type="int", default=1,
                      help="set ARQ mode (NULL (1), RTT (6)) [default=%default]")
  expert_grp.add_option("","--tun-device-filename", default="/dev/net/tun",
                      help="path to tun device file [default=%default]")
  transceiver_phy.TXRX.add_options(parser, expert_grp)
  (options, args) = parser.parse_args ()

  # open the TUN/TAP interface
  (tun_fd, tun_ifname) = open_tun_interface(options.tun_device_filename)
  if options.node == "A":
    os.system('ifconfig gr0 192.168.200.1')
    #os.system('ifconfig gr0 hw ether b2:c8:41:27:56:a0')
    #os.system('arp -s 192.168.200.2 6e:77:48:12:90:6a')
    print
    #print "[A] Allocated virtual ethernet interface: %s with MAC b2:c8:41:27:56:a0" % (tun_ifname,)
    print "[A] Allocated virtual ethernet interface: %s with IP 192.168.200.1"  % (tun_ifname,)
    print
  if options.node == "B":
    os.system('ifconfig gr0 192.168.200.2')
    #os.system('ifconfig gr0 hw ether 6e:77:48:12:90:6a')
    #os.system('arp -s 192.168.200.1 b2:c8:41:27:56:a0')
    print
    #print "[B] Allocated virtual ethernet interface: %s with MAC 6e:77:48:12:90:6a" % (tun_ifname,)
    print "[B] Allocated virtual ethernet interface: %s with IP 192.168.200.2"  % (tun_ifname,)
    print

  # instantiate the MAC
  mac = cs_mac(tun_fd, verbose=True)

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
    callback = None
  else:
    callback = mac.phy_rx_callback
  
  # check node type
  if options.node is None:
    print "ERROR: please specify node type, A or B \n"
    exit(-1)

  # build the graph (PHY)
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


