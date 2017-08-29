#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
# Copyright 2014 Lizhao You
#
import gnuradio
from gnuradio import gr, digital, blocks, analog
import raw
import array, struct, time, sys, math

# import from local
from phy_params import ofdm_params  
from phy_params import PNC_TYPE
from ofdm_sync import ofdm_sync, ofdm_sync_fir

bitrates = {
  1 : (1, (4, 4)),
  2 : (1, (6, 4)), # DONT USE THIS BITRATE
  3 : (2, (4, 4)),
  4 : (2, (6, 4)),
  5 : (4, (4, 4)),
  6 : (4, (6, 4)),
  7 : (6, (4, 3)),
  8 : (6, (6, 4))
  }

class OFDM_RX(gr.hier_block2):
  def __init__(self, options):
    p = ofdm_params(options)
    nsym = options.size+len(p.preambles)

    gr.hier_block2.__init__(self, "OFDM_RX",
      gr.io_signature(1,1, gr.sizeof_gr_complex),
      gr.io_signature2(2,2, p.data_tones*gr.sizeof_char, gr.sizeof_char))
      #gr.io_signature(0,0,0))

    ################################################################################
    #                        Block Definition and Connection
    ################################################################################
    # to the synchronization algorithm
    #sync = ofdm_sync(p.fft_length, p.cp_length, p.cross_corr_preambles, 'RAW', options.log)
    sync = ofdm_sync_fir(p.fft_length, p.cp_length, p.cross_corr_preambles, options.bandwidth, 'RAW', options.log)

    print "============================"
    #if options.node == "A":
    #  sync.peak.set_file_mode(0)
    #elif options.node == "B":
    #  sync.peak.set_file_mode(1)

    # correct for fine frequency offset computed in sync (up to +-pi/fft_length)
    print "============================"
    nco_sensitivity = 2.0/p.fft_length

    nco = analog.frequency_modulator_fc(nco_sensitivity)
    sigmix = blocks.multiply_cc()

    # sample at symbol boundaries
    # NOTE: (sync,2) indicates the first sample of the symbol!
    sampler = raw.ofdm_sampler(p.fft_length, p.fft_length+p.cp_length, long(options.bandwidth), timeout=nsym, debug=False)

    # fft on the symbols
    # see gr_fft_vcc_fftw that it works differently if win = []
    win = [1 for i in range(p.fft_length)]
    fft = gnuradio.fft.fft_vcc(p.fft_length, True, win, True)

    # use the preamble to correct the coarse frequency offset and initial equalizer
    (qambits, (nc, np)) = bitrates[options.bitrate]
    demod = raw.ofdm_demod(p.fft_length, p.data_tones, p.cp_length, p.preambles, p.padded_carriers, options.size, qambits)

    #self.connect(self, sync)

    #self.connect(blocks.null_source(p.data_tones*gr.sizeof_char), (self,0))
    #self.connect(blocks.null_source(gr.sizeof_char), (self,1))
    
    self.connect(self, sync)
    #self.connect((sync,0), (sigmix,0))  # see the reason below (FPNC_RX)
    #self.connect((sync,1), nco, (sigmix,1))
    #self.connect(sigmix, (sampler,0))
    self.connect((sync,0), (sampler,0))
    self.connect((sync,1), (sampler,2))
    self.connect((sync,2), (sampler,1))            # timing signal to sample at
    self.connect((sampler,0), fft)
    self.connect(fft, demod, self)
    self.connect((sampler,1), (demod,1), (self,1))

    #sampler.set_max_noutput_items(4)
    #demod.set_max_noutput_items(4)

    #self.connect((sync, 2), blocks.tag_debug(gr.sizeof_char, "(sync,2)"))
    #self.connect((sampler,1), blocks.tag_debug(gr.sizeof_char, "(sampler,1)"))
    #self.connect((demod,1), blocks.tag_debug(gr.sizeof_char, "demod_flag"))

    ################################################################################

    self.params = p
    self.size = options.size
    self.sampler = sampler
    self.fft = fft
    self.sync = sync

    if options.log:
      self.connect((sync,0),
                   blocks.file_sink(gr.sizeof_gr_complex, 'logs/rx-sync.dat'))
      self.connect((sync,2),
                   blocks.file_sink(gr.sizeof_char, 'logs/rx-sync.datb'))
      #self.connect(nco,
      #             blocks.file_sink(gr.sizeof_gr_complex, "logs/rx-nco.dat"))
      #self.connect(sigmix,
      #             blocks.file_sink(gr.sizeof_gr_complex, "logs/rx-sigmix.dat"))
      self.connect((sampler,0),
                   blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, "logs/rx-sampler.dat"))
      self.connect((sampler,1),
                   blocks.file_sink(gr.sizeof_char, "logs/rx-sampler.datb"))
      self.connect(fft,
                   blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, "logs/rx-fft.dat"))

  def add_options(normal, expert):
    normal.add_option("-s", "--size", type="int", default=16,
                      help="set number of symbols per packet [default=%default]")
    normal.add_option("", "--node", type="string", default=None,
                      help="select pnc endnode: A or B [default=None]")
    ofdm_params.add_options(normal, expert)
  add_options = staticmethod(add_options)

class FPNC_RX(gr.hier_block2):
  """
  Demodulates a received OFDM stream.
  This block performs synchronization, FFT, and demodulation of incoming OFDM
  symbols and passes packets up the a higher layer.

  input[0]: complex base band (from USRP)

  output[0]: demodulation results (hard or soft)
  output[1]: start-of-frame indicator (one per symbol)
  """
  def __init__(self, options):
    """
    @param options: parsed raw.ofdm_params
    """
    assert(options.mode == "PNC")
    p = ofdm_params(options)
    nsym = options.size+len(p.preambles)

    if options.msg_type == PNC_TYPE:
      output_signature = gr.io_signature2(2,2, p.data_tones*gr.sizeof_char, gr.sizeof_char)
      #output_signature = gr.io_signature(0,0,0)
    
    gr.hier_block2.__init__(self, "FPNC_OFDM_RX",gr.io_signature(1,1, gr.sizeof_gr_complex),output_signature)

    ################################################################################
    #                        Block Definition and Connection
    ################################################################################
    # to the synchronization algorithm
    #sync = ofdm_sync(p.fft_length, p.cp_length, p.cross_corr_preambles, 'PNC', options.log)
    sync = ofdm_sync_fir(p.fft_length, p.cp_length, p.cross_corr_preambles, options.bandwidth,'PNC', options.log)

    # correct for fine frequency offset computed in sync (up to +-pi/fft_length)
    nco_sensitivity = 2.0/p.fft_length
    #nco_sensitivity = 0
    nco = analog.frequency_modulator_fc(nco_sensitivity)  # FIXME: we disable CFO compenstation for PNC
    sigmix = blocks.multiply_cc()

    # sample at symbol boundaries
    sampler = raw.ofdm_sampler(p.fft_length, p.fft_length+p.cp_length, long(options.bandwidth), timeout=nsym)
    sampler.set_relay_flag(True)

    # fft on the symbols
    # see gr_fft_vcc_fftw that it works differently if win = []
    win = [1 for i in range(p.fft_length)]
    fft = gnuradio.fft.fft_vcc(p.fft_length, True, win, True)

    functions = {'hard':  0, 'soft1': 1}
    mode = functions[options.demod_type]
    print "===== >>>>> Demod Mode = ", mode, options.demod_type
    (qambits, (nc, np)) = bitrates[options.bitrate]
    self.demod = raw.pnc_demod(p.fft_length, p.data_tones, p.cp_length, p.preambles, p.padded_carriers, options.size, options.alpha, mode, qambits)

    # we delay two more symbols to allow signal sampling
    symbol_length = p.fft_length+p.cp_length
    STS_LEN = 2
    sts_delay = blocks.delay(gr.sizeof_gr_complex, STS_LEN*symbol_length)

    self.connect(self, sync)

    #self.connect((sync,0), blocks.null_sink(gr.sizeof_gr_complex))
    #self.connect((sync,1), blocks.null_sink(gr.sizeof_float))
    #self.connect((sync,2), blocks.null_sink(gr.sizeof_char))

    #self.connect(blocks.null_source(p.data_tones*gr.sizeof_char), (self,0))
    #self.connect(blocks.null_source(gr.sizeof_char), (self,1))

    # not using cfo compensate
    self.connect((sync,0), (sampler, 0))
    self.connect((sync,1), (sampler, 2))
    self.connect((sync,2), (sampler,1))               # timing signal to sample at

    self.connect((sampler,0), fft, (self.demod, 0))
    self.connect((sampler,0), (self.demod,2))         # time-domain signal
    self.connect((sampler,1), (self.demod,1))

    self.connect((self.demod,1), (self,1))            # flag output
    self.connect((self.demod,0), (self,0))            # XOR bits output
    #self.connect((self.demod,1), blocks.null_sink(gr.sizeof_char))
    #self.connect((self.demod,0), blocks.null_sink(p.data_tones*gr.sizeof_char))

    if False:
      self.connect((sampler,0),
                  blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, "logs/rx-sampler.dat"))
      self.connect((sampler,1),
                  blocks.file_sink(gr.sizeof_char, "logs/rx-sampler.datb"))
      self.connect((sync,2),
                  blocks.file_sink(gr.sizeof_char, 'logs/rx-sync.datb'))
      self.connect((sync,0),
                   blocks.file_sink(gr.sizeof_gr_complex, 'logs/rx-sync.dat'))
      self.connect((self.demod,1),
                   blocks.file_sink(gr.sizeof_char, 'logs/rx-demod.datb'))

    ################################################################################
    self.params = p
    self.size = options.size
    self.sampler = sampler
    self.fft = fft
    self.sync = sync
    
    if options.log:
      self.connect((sync,2),
                   blocks.file_sink(gr.sizeof_char, 'logs/rx-sync.datb'))
      self.connect((sync,0),
                   blocks.file_sink(gr.sizeof_gr_complex, 'logs/rx-sync.dat'))
      self.connect((sampler,0),
                   blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, "logs/rx-sampler.dat"))
      self.connect((sampler,1),
                   blocks.file_sink(gr.sizeof_char, "logs/rx-sampler.datb"))

      self.connect((sync,2),
                   blocks.file_sink(gr.sizeof_char, 'logs/rx-sync.datb'))
      self.connect(sigmix,
                   blocks.file_sink(gr.sizeof_gr_complex, "logs/rx-sigmix.dat"))
    
    if options.log:
      self.connect((sync,0),
                   blocks.file_sink(gr.sizeof_gr_complex, 'logs/rx-sync.dat'))
      self.connect((sync,1),
                   blocks.file_sink(gr.sizeof_float, 'logs/rx-sync-cfo.dat'))
      self.connect(nco,
                   blocks.file_sink(gr.sizeof_gr_complex, "logs/rx-nco.dat"))
      self.connect((sampler,0),
                   blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, "logs/rx-sampler.dat"))
      self.connect((sampler,1),
                   blocks.file_sink(gr.sizeof_char, "logs/rx-sampler.datb"))
      self.connect(fft,
                   blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, "logs/rx-fft.dat"))
      self.connect((self.demod,1), 
                   blocks.file_sink(gr.sizeof_char, 'logs/pnc-flag.dat'))

      if True:
        self.connect((self.demod,0), 
                     blocks.file_sink(gr.sizeof_char*p.data_tones, 'logs/pnc-bits.dat'))

  def add_options(normal, expert):
    normal.add_option("-s", "--size", type="int", default=16,
                      help="set number of symbols per packet [default=%default]")
    expert.add_option("", "--demod-type", type="string", default="soft1",
                      help="select demodulator mode: hard:0, soft1:1 [default=%default]")
    expert.add_option("", "--alpha", type="eng_float", default=0.5, metavar="ALPHA",
                      help="divide range(2H_max*alpha) into 255 intervals [default=%default]")
    ofdm_params.add_options(normal, expert)
  add_options = staticmethod(add_options)

# /////////////////////////////////////////////////////////////////////////////
#                   mod/demod with packets as i/o
# /////////////////////////////////////////////////////////////////////////////

class TX(gr.hier_block2):
  """
  Modulates an OFDM stream. Creates OFDM symbols from RAW bits.

  input[0]: data bits
  input[1]: start-of-frame indicator (one per symbol)

  output[0]: baseband-modulated signal
  """
  def __init__(self, options):
    #self.tx = raw.ofdm_mod(options)
    #self.params = self.tx.params
    self.params = ofdm_params(options)
    symbol_size = self.params.data_tones * gr.sizeof_gr_complex

    gr.hier_block2.__init__(self, "OFDM_TX",
      gr.io_signature(1,1, symbol_size),
      gr.io_signature(1,1, gr.sizeof_gr_complex))

    ############################################################################################
    if options.node == "A":
      node_type = 1
    elif options.node == "B":
      node_type = 2
    elif options.node == None:
      node_type = 0
    else:
      print"pilot setup error, you mush specify the pilot type for endnode",
      raise SystemExit("The node type must be specified\n")

    if options.mode == "RAW":  # For compatability
      node_type = 0
    ###########################################################################################
    win = []
    self.msg = raw.message_source(gr.sizeof_char, 16, True)

    params = self.params
    self.mapper = raw.ofdm_mapper(params.padded_carriers, node_type)
    #self.preambles = raw.ofdm_insert_preamble(params.fft_length, params.preambles_td)
    self.preambles = raw.ofdm_insert_amble(params.fft_length, options.size, params.preambles_td, params.padding_td)
    self.ifft = gnuradio.fft.fft_vcc(params.fft_length, False, win, True)
    self.cp_adder = raw.ofdm_cyclic_prefixer(params.fft_length, params.fft_length + params.cp_length)
    self.scale = blocks.multiply_const_cc(1.0 / math.sqrt(params.fft_length), params.fft_length)

    self.connect(self, self.mapper, self.ifft, self.scale, (self.preambles,0))
    self.connect(self.msg, (self.preambles,1))
    self.connect((self.preambles,0), (self.cp_adder,0), self)
    self.connect((self.preambles,1), (self.cp_adder,1))

    self.size = options.size

    #self.connect((self.preambles,1), blocks.file_sink(gr.sizeof_char, 'tx_preamble_flag.datb'))

  # TODO: add burst mode
  def send_packet(self, time=None): # NOTE: we keep this to allow discontinuous operation
    """ send a packet of options.size symbols from the datafile """
    payload = [0] * self.size
    payload[self.size-1] = 9
    if time is not None:
      payload[0] = 2   # indicator of packet ending (using TS/EOB)
    else:
      payload[0] = 1   # indicator of packet ending (not using TS/EOB)
    payload = array.array('B', payload)

    msg = raw.message_from_string2(payload.tostring())
    if time is not None:
      secs = long(time)
      frac = time - secs
      msg.set_timestamp(secs, frac)

    msgq = self.msg.msgq()
    msgq.insert_tail(msg)
  
  def send_eof(self):
    msgq = self.msg.msgq()
    msgq.insert_tail(raw.message2(1))

  def add_options(normal, expert):
    normal.add_option("-s", "--size", type="int", default=16,
                      help="set number of symbols per packet [default=%default]")
    expert.add_option("", "--node", type="string", default=None,
                      help="select pnc endnode: A or B [default=None]")
    ofdm_params.add_options(normal, expert)
  add_options = staticmethod(add_options)
