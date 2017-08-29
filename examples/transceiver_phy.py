#!/usr/bin/env python
#
# Copyright 2012 Lizhao You
#


from gnuradio import gr, blocks
from uhd_interface import uhd_transmitter, uhd_receiver

import raw

import transmit_path
import receive_path


# /////////////////////////////////////////////////////////////////////////////
#                             the flow graph (PHY)
# /////////////////////////////////////////////////////////////////////////////
class TXRX(gr.top_block):
  """
    Tx path:
      1. RAW: use RAW mode
      2. PNC: use PNC mode (diff preamble and pilot)
    Rx path:
      1. RAW: use RAW mode
      2. PNC: use PNC mode (diff sync and MUD-XOR-CD)
  """
  def __init__(self, options_tx, options_rx, callback):
    gr.top_block.__init__(self)

    self.init_pctime = 0
    self.init_hwtime = 0

    self.tx_outfile    = options_tx.tx_outfile
    self._tx_amplitude = options_tx.tx_amplitude 
    #######################################################################
    #                              USRP Config                            #
    #######################################################################
    tx_slave=False
    rx_slave=False
    if options_tx.tx_args != options_rx.rx_args:
      print "===Transceiver: enable MIMO mode, RX as slave, TX as master==="
      rx_slave=True
    if options_rx.mode == "PNC":
      rx_slave=False
      
    # lzyou: it seems that you MUST put rx before tx
    # otherwise something strange will happen (eg, not synced even the program tells you synced)
    d_rx_enable = True
    if options_rx.rx_freq is not None:
      source = uhd_receiver(options_rx.rx_args,
                            options_rx.bandwidth,
                            options_rx.rx_freq, options_rx.rx_gain,
                            options_rx.spec, options_rx.antenna,
                            options_rx.verbose, options_rx.external, rx_slave)
      #source.u.set_min_output_buffer(163840)
    elif options_rx.rx_infile is not None:
      source = blocks.file_source(gr.sizeof_gr_complex, options_rx.rx_infile)
    else:
      source = None
      d_rx_enable = False
    self.d_rx_enable = d_rx_enable
    self.source = source
    
    d_tx_enable = True
    if options_tx.tx_freq is not None:
      sink = uhd_transmitter(options_tx.tx_args,
                             options_tx.bandwidth,
                             options_tx.tx_freq, options_tx.tx_gain,
                             options_tx.spec, options_tx.antenna,
                             options_tx.verbose, options_tx.external, tx_slave)
    elif options_tx.tx_outfile is not None:
      sink = blocks.file_sink(gr.sizeof_gr_complex, options_tx.tx_outfile)
    else:
      sink = None
      d_tx_enable = False
    self.d_tx_enable = d_tx_enable
    self.sink = sink
    #######################################################################

    # FIXME: support diff (tx/rx) framebytes   
    self.framebytes = None
    #######################################################################
    #                              Tx Path                                #
    #######################################################################
    if d_tx_enable:
      self.tx = transmit_path.ofdm_transmit_path(options_tx)
      self.amp = blocks.multiply_const_cc(self._tx_amplitude)
      #self.amp.set_min_output_buffer(163840)
      self.connect(self.tx, self.amp, self.sink)
      self.framebytes = self.tx.framebytes
    #######################################################################
    
    #######################################################################
    #                              Rx Path                                #
    #######################################################################
    # log usrp raw samples
    #if options_rx.mode == 'PNC':
    #  self.connect(source, blocks.file_sink(gr.sizeof_gr_complex, '/tmp/PNCRXdiag.dat'))
    if options_rx.logfile:
      if options_rx.mode == 'PNC':
        self.connect(source, blocks.file_sink(gr.sizeof_gr_complex, '/tmp/PNCRXdiag.dat'))
      else:
        assert(options_rx.node != None)

      if options_rx.node == 'A':
        self.connect(source, blocks.file_sink(gr.sizeof_gr_complex, '/tmp/NodeARXdiag.dat'))
      elif options_rx.node == 'B':
        self.connect(source, blocks.file_sink(gr.sizeof_gr_complex, '/tmp/NodeBRXdiag.dat'))

    # setup receive_path
    if d_rx_enable:
      if options_rx.rx_outfile is not None:
        self.rx = blocks.file_sink(gr.sizeof_gr_complex, options_rx.rx_outfile)
        self.connect(self.source, self.rx)
        return
      
      self.rx = receive_path.ofdm_receive_path(options_rx, callback)
      #if self.framebytes is not None:
      #  assert(self.framebytes == self.rx.framebytes)
      self.framebytes = self.rx.framebytes
      self.connect(self.source, self.rx)
    else:
      self.rx = None

    #######################################################################
    if options_rx.profile:
      from gnuradio.ctrlport import monitor
      self.ctrlport_monitor = monitor()
      self.ctrlport_monitor_performance = monitor("gr-perf-monitorx")


  def set_tx_amplitude(self, ampl):
    """
    Sets the transmit amplitude sent to the USRP
    @param: ampl 0 <= ampl < 1.0.  Try 0.10
    """
    self._tx_amplitude = max(0.0, min(ampl, 1))
    self.amp.set_k(self._tx_amplitude)
      
  def set_nco(self, nco):
    if self.d_tx_enable:
      self.tx.set_nco(nco)    
        
  # Provide general send_pkt API
  def send_pkt(self, payload='', time=None, eof=False, msgq_tx_enable=False):
    if self.d_tx_enable: 
      if eof:
        self.tx.send_eof()
      elif msgq_tx_enable:
        self.tx.send_samples(payload, time)
      else:
        self.tx.send_pkt(payload, time)

  def get_current_hwtime(self):
    if self.tx_outfile is not None:
      return time.time()
    else:
      usrp_time, frac = self.sink.get_usrp_time()
      curtime = usrp_time+frac
      #curtime = self.rx.sampler.get_current_hwtime()
      #curtime = time.time()-self.init_pctime+self.init_hwtime
      return curtime

  def get_init_time(self):
    # not for transmitter only
    if self.rx is not None:
      self.init_pctime = self.rx.sampler.get_init_pctime()
      self.init_hwtime = self.rx.sampler.get_init_hwtime()
    return [self.init_pctime, self.init_hwtime]
  
  def _print_verbage(self):
    """
    Prints information about the UHD transceiver and Tx/Rx path
    """
    self.source._print_verbage()
    self.sink._print_verbage()
    self.tx._print_verbage()
    self.rx._print_verbage()

  def add_options(normal, expert):
    normal.add_option("", "--rx-infile", type="string",
                      help="select RX input file (raw)")
    normal.add_option("", "--rx-outfile", type="string",
                      help="select RX output file (raw)")
    normal.add_option("", "--tx-outfile", type="string", default=None,
                      help="select TX output file (raw)")
    normal.add_option("-W", "--bandwidth", type="eng_float", default=1e6,
                      help="set symbol bandwidth [default=%default]")  
    normal.add_option("", "--tx-amplitude", type="eng_float", default=0.1, metavar="AMPL",
                      help="set transmitter digital amplitude: 0 <= AMPL < 1.0 [default=%default]")
    expert.add_option("-v", "--verbose", action="store_true", default=False)
    expert.add_option("", "--profile", action="store_true", default=False,
                      help="enable ctrlport_monitor and ctrlport_monitor_performance")
    expert.add_option("", "--logfile", action="store_true", default=False,
                      help="log all usrp samples")
    uhd_transmitter.add_options(normal)
    uhd_receiver.add_options(normal)
    transmit_path.ofdm_transmit_path.add_options(normal, expert)
    receive_path.ofdm_receive_path.add_options(normal, expert)
    
  # Make a static method to call before instantiation
  add_options = staticmethod(add_options)
