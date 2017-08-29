#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
#

import array, struct, time, sys
from gnuradio import gr, blocks

import raw


"""
  fixed size RX/TX blocks

  qam + conv + punc
"""

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

def framebytes(bitrate, size):
  """ size = number of complex dimensions in frame """
  (qambits, (nc, np)) = bitrate
  # remember, 1/2-rate conv code!
  np*= 2
  assert (size * qambits * nc) % (np * 8) == 0
  bytes = size * qambits * nc / (np * 8)
  return bytes

class qam_rx(gr.hier_block2):
  """ wraps conv demodulator """
  def __init__(self, bitrate, ncarriers, nsymbols, log=False):

    (qambits, (nc, np)) = bitrates[bitrate]
    frametones = ncarriers * nsymbols
    self.framebytes = framebytes((qambits, (nc, np)), frametones)
    self.framebytes -= 1 # room for padding

    gr.hier_block2.__init__(self, "qam_rx",
      gr.io_signature(1,1, gr.sizeof_char),
      gr.io_signature(1,1, gr.sizeof_char))

    conv = raw.conv_dec((self.framebytes+1)*8) # decoded length in bits including padding
    punc = raw.conv_punc(np, nc, 128)
    intrlv = raw.intrlv_bit(ncarriers, qambits, True)
    pad = raw.conv_punc(self.framebytes+1, self.framebytes)

    self.connect(self,
                 intrlv,
                 punc,
                 conv, # output is stream of bytes... includes padding
                 pad,
                 self)

    if log:
      self.connect(intrlv,
                   blocks.file_sink(gr.sizeof_char, "logs/rx-punc.datb"))
      self.connect(punc,
                   blocks.file_sink(gr.sizeof_char, "logs/rx-conv.datb"))
      self.connect(conv,
                   blocks.file_sink(gr.sizeof_char, "logs/rx-pad.datb"))
      self.connect(pad,
                   blocks.file_sink(gr.sizeof_char, 'logs/rx-final.datb'))

class qam_tx(gr.hier_block2):
  """ wraps QAM+conv modulator """
  def __init__(self, bitrate, ncarriers, nsymbols, log=False):

    (qambits, (nc, np)) = bitrates[bitrate]
    frametones = ncarriers * nsymbols
    self.framebytes = framebytes((qambits, (nc, np)), frametones)
    self.framebytes -= 1 # room for padding

    gr.hier_block2.__init__(self, "qam_tx",
      gr.io_signature(1,1, gr.sizeof_char),
      gr.io_signature(1,1, gr.sizeof_gr_complex))

    conv = raw.conv_enc()
    punc = raw.conv_punc(nc, np)
    intrlv = raw.intrlv_bit(ncarriers, qambits, False)
    qam = raw.qam_enc(qambits)
    pad = raw.conv_punc(self.framebytes, self.framebytes+1)

    # main source accepts whole frames!
    self.connect(self,
                 pad,
                 conv,
                 punc,
                 intrlv,
                 qam,
                 self)

    if log:
      self.connect(intrlv,
                   blocks.file_sink(gr.sizeof_char, "logs/tx-intrlv.datb"))
      self.connect(punc,
                   blocks.file_sink(gr.sizeof_char, "logs/tx-punc.datb"))
      self.connect(conv,
                   blocks.file_sink(gr.sizeof_char, "logs/tx-conv.datb"))
      self.connect(pad,
                   blocks.file_sink(gr.sizeof_char, "logs/tx-pad.datb"))