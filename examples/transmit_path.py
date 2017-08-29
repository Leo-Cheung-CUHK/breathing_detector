#!/usr/bin/env python
#
# Copyright 2014 Lizhao You
#

from gnuradio import gr, digital, eng_notation, blocks
from gnuradio.eng_option import eng_option
import raw
import ofdm_rxtx, raw_qam

# /////////////////////////////////////////////////////////////////////////////
#                              transmit path
# /////////////////////////////////////////////////////////////////////////////
class ofdm_transmit_path(gr.hier_block2): 
  def __init__(self, options):
    gr.hier_block2.__init__(self, "ofdm_transmit_path",
                            gr.io_signature(0, 0, 0),
                            gr.io_signature(1, 1, gr.sizeof_gr_complex))
    if options.data_file is not None:
      data = gr.message_source(gr.sizeof_gr_complex)
      ofdm = data
      framebytes = 0
    else:
      ofdm = ofdm_rxtx.TX(options)
      qam = raw_qam.qam_tx(options.bitrate, ofdm.params.data_tones, ofdm.size, log=options.log)
      framebytes = qam.framebytes
      data = blocks.message_source(gr.sizeof_char * framebytes, 16)
      data = blocks.message_burst_source(gr.sizeof_char * framebytes, 16)
      self.connect(data,
                   blocks.vector_to_stream(gr.sizeof_char, qam.framebytes),
                   qam,
                   blocks.stream_to_vector(gr.sizeof_gr_complex, ofdm.params.data_tones),
                   ofdm)

    # precoding module
    nco_sensitivity = 2.0 / options.fft_length  # correct for fine frequency
    nco = raw.pnc_frequency_modulator_fc(nco_sensitivity)
    print "===========>>>>>>>>>"
    print "precoding: ", options.precoding
    if options.precoding == "YES":
      self.connect(ofdm, nco, self)
      self.nco = nco
    else:
      self.connect(ofdm, self)
      self.nco = None

    self.data = data
    self.tx = ofdm
    self.framebytes = framebytes
    
    if options.log:
      self.connect(self.tx,  blocks.file_sink(gr.sizeof_gr_complex, 'logs/tx.dat'))

  def send_samples(self, payload='', time=None):
    msg = gr.message_from_string(payload)
    if time is not None:
      secs = long(time)
      frac_secs = time - long(time)
      msg.set_timestamp(secs, frac_secs)
    else:
      #print "[TRANSMIT_PATH] WARNING: no timestamp!"
      pass
    self.tx.msgq().insert_tail(msg)
  
  def send_pkt(self, payload='', time=None):
    msg = gr.message_from_string(payload)       
    self.data.msgq().insert_tail(msg)           # insert data
    self.tx.send_packet(time)              # insert control data
    
  def send_eof(self):
    self.data.msgq().insert_tail(gr.message(1))
    self.tx.send_eof()

  def send_eof2(self):
    self.data.msgq().insert_tail(gr.message(1))
    
  def set_nco(self, cfo):
    if self.nco is not None:
      return self.nco.set_value(cfo)

  def string_to_hex_list(self, s):
    return map(lambda x: hex(ord(x)), s)
        
  def add_options(normal, expert):
    """Adds transmitter-specific options to the Options Parser"""
    normal.add_option("", "--data-file", type="string", default=None,
                      help="use complex input file for transmission")
    expert.add_option("-p", "--precoding", type="string", default="NO", 
                      help="precoding option (for PNC endnode) [default=%default]")
    expert.add_option("", "--bitrate", type="int", default=1,
                      help="set bitrate index [default=%default]")
    expert.add_option("-v", "--verbose", action="store_true", default=False)
    expert.add_option("", "--log", action="store_true", default=False,
                      help="log all parts of flow graph to files (CAUTION: lots of data)")
    ofdm_rxtx.TX.add_options(normal, expert)
  # Make a static method to call before instantiation
  add_options = staticmethod(add_options)

  def _print_verbage(self):
    """
    Prints information about the transmit path
    """
    print "Tx amplitude     %s" % (self._tx_amplitude)
