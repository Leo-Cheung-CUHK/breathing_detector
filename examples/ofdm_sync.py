#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
# Copyright 2014 Lizhao You
#

import math
import gnuradio
from gnuradio import gr, blocks


import raw

class ofdm_sync(gr.hier_block2):
  """
  OFDM synchronization using pseudo-noise correlation.
  T. M. Schmidl and D. C. Cox, "Robust Frequency and Timing
  Synchonization for OFDM," IEEE Trans. Communications, vol. 45,
  no. 12, 1997.

  Assumes that the signal preamble is two identical copies of pseudo-noise
  half-symbols. Equivalently in frequency domain, the odd freqencies are 0.

  The two half-symbols are expected to be offset by pi T deltaf.
  This method can detect the timing and the frequency offset but only within
  +- 1/T.

  Let L = T/2
  P(d) = sum_{i=1..L} x[d+i] x[d+i+L]*
  R(d) = sum_{i=1..L} |x[d+i+L]|^2
  M(d) = |P(d)|^2/R(d)^2
  d_opt = argmax_d M(d)
  timing_start := d_opt
  fine_freq := angle(P(d_opt)) / (2 pi L)

  Observe: angle(P(d_opt)) is in [-pi, pi]
    Therefore max CFO that can be detected is 1/(2L) * fft_len

  Outputs:
    Output 0: delayed signal
    Output 1: fine frequency correction value (to drive NCO)
    Output 2: timing signal (indicates first sample of preamble)

  -------------------------------------------------------------------------------
  RPNC Note:
  * this is similar to ofdm_sync_pn.py in gnuradio.

  Mode:
  * PNC: with cross-corr, seperate cross-corr and mode identification
       - support USER_MODE, USER_A_MODE, USER_B_MODE and PNC_MODE
  * RAW: with cross-corr, but integrate everything in raw_regenerate_peak2
       - support USER_MODE
  """
  def __init__(self, fft_length, cp_length, preambles, mode='RAW', logging=False):
    gr.hier_block2.__init__(self, "ofdm_sync_pn",
      gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
      gr.io_signature3(3, 3,    # Output signature
        gr.sizeof_gr_complex,   # delayed input
        gr.sizeof_float,        # fine frequency offset
        gr.sizeof_char          # timing indicator
      ))

    period = fft_length/2
    window = fft_length/2

    # Calculate the frequency offset from the correlation of the preamble
    x_corr = blocks.multiply_cc()
    self.connect(self, blocks.conjugate_cc(), (x_corr, 0))
    self.connect(self, blocks.delay(gr.sizeof_gr_complex, period), (x_corr, 1))
    P_d = blocks.moving_average_cc(window, 1.0)
    self.connect(x_corr, P_d)

    P_d_angle = blocks.complex_to_arg()
    self.connect(P_d, P_d_angle)

    # Get the power of the input signal to normalize the output of the correlation
    R_d = blocks.moving_average_ff(window, 1.0)
    self.connect(self, blocks.complex_to_mag_squared(), R_d)
    R_d_squared = blocks.multiply_ff() # this is retarded
    self.connect(R_d, (R_d_squared, 0))
    self.connect(R_d, (R_d_squared, 1))
    M_d = blocks.divide_ff()
    self.connect(P_d, blocks.complex_to_mag_squared(), (M_d, 0))
    self.connect(R_d_squared, (M_d, 1))

    # Now we need to detect peak of M_d
    # the peak is up to cp_length long, but noisy, so average it out
    matched_filter = blocks.moving_average_ff(cp_length, 1.0/cp_length)

    # NOTE: the look_ahead parameter doesn't do anything
    # these parameters are kind of magic, increase 1 and 2 (==) to be more tolerant
    peak_detect = raw.peak_detector_fb(0.25, 0.55, 30, 0.001)
    
    # offset by -1
    self.connect(M_d, matched_filter, blocks.add_const_ff(-1), peak_detect)

    # peak_detect indicates the time M_d is highest, which is the end of the symbol.
    # nco(t) = P_d_angle(t-offset) sampled at peak_detect(t)
    # modulate input(t - fft_length) by nco(t)
    # signal to sample input(t) at t-offset
    #
    ##########################################################
    # IMPORTANT NOTES:
    # We can't delay by < 0 so instead:
    # input is delayed by some_length
    # signal to sample is delayed by some_length - offset
    ##########################################################
    # peak_delay and offset are for cutting CP
    # FIXME: until we figure out how to do this, just offset by 6 or cp_length/2
    symbol_length = fft_length+cp_length
    peak_delay = cp_length
    offset = 6 #cp_length/2
    self.offset = offset

    # regenerate peak using cross-correlation
    # * RAW mode: use raw_generate_peak2 block to filter peaks
    # * PNC mode: use raw_generate_peak3 block to identify the proper peak for two users
    # PNC mode delays all input peak_delay samples
    # * cp_length: delay for cross-correlation since auto-correlation is not so accurate
    # * 3 symbol_length: delay for identifying mode for PNC
    # * -offset: for cp cut
    if mode == 'PNC':
      # The block structure is
      # signal -> 3*symbol_length+cp_length-offset     -> (self,0) [signal]
      # signal -> cp_length delay -> auto  -> (peak,0) -> (self,2) [peak]
      # signal -> cp_length delay          -> (peak,1) -> (self,1) [cfo]
      # cfo    -> cp_length delay          -> (peak,2)
      #
      # we let cross's signal go faster to identify the beginning
      # we use cross's peak output to find cfo, so the delay of cfo should = cross delay
      # we use raw_regenerate_peak to do cross-corr, and symbols energy after STS to identify mode 
      # so the signal is further delayed by three symbols more
      self.signal_cross_delay = blocks.delay(gr.sizeof_gr_complex, peak_delay)
      self.cfo_cross_delay = blocks.delay(gr.sizeof_float, peak_delay)

      LOOK_AHEAD_NSYM = 3
      self.connect(self, self.signal_cross_delay)
      self.connect(P_d_angle, self.cfo_cross_delay)

      peak = raw.regenerate_peak3(fft_length, fft_length+cp_length, LOOK_AHEAD_NSYM, peak_delay, preambles, False)
      self.connect(peak_detect,             (peak,0))
      self.connect(self.signal_cross_delay, (peak,1))
      self.connect(self.cfo_cross_delay,    (peak,2))

      self.signal_delay = blocks.delay(gr.sizeof_gr_complex, LOOK_AHEAD_NSYM*symbol_length+peak_delay-offset)
      self.connect(self, self.signal_delay, (self,0))  # signal output
      self.connect((peak,1), (self,1))                 # cfo output
      self.connect((peak,0), (self,2))                 # peak output

      if logging:
        self.connect(self.cfo_cross_delay, blocks.file_sink(gr.sizeof_float, 'logs/input-cfo.dat'))
        #self.connect(cross, blocks.file_sink(gr.sizeof_float, 'logs/cross.dat'))
        self.connect(self.signal_cross_delay, blocks.file_sink(gr.sizeof_gr_complex, 'logs/cross-signal.dat'))

    if mode == 'RAW':
      LOOK_AHEAD_NSYM = 0
      peak = raw.regenerate_peak2(fft_length, fft_length+cp_length, LOOK_AHEAD_NSYM, preambles)
      self.signal_delay1 = blocks.delay(gr.sizeof_gr_complex, peak_delay-offset)
      self.signal_delay2 = blocks.delay(gr.sizeof_gr_complex, offset)
      self.cfo_delay = blocks.delay(gr.sizeof_float, peak_delay) # we generate the same delay with signal
      self.connect(peak_detect, (peak,0))
      self.connect(P_d_angle, self.cfo_delay, (peak,1))
      self.connect(self, self.signal_delay1, self.signal_delay2, (peak,2))

      self.connect(self.signal_delay1, (self,0))      # signal output
      self.connect((peak,1), (self,1))                # cfo output
      self.connect((peak,0), (self,2))                # raw peak output

      if logging:
        self.connect(self.signal_delay1, blocks.file_sink(gr.sizeof_gr_complex, 'logs/test-out-signal.dat'))
        self.connect((peak,0), blocks.file_sink(gr.sizeof_char, 'logs/test-out-peak.datb'))    
        self.connect(self.cfo_delay, blocks.file_sink(gr.sizeof_float, 'logs/test-cfo.dat'))
        self.connect(peak_detect, blocks.file_sink(gr.sizeof_char, 'logs/test-out-auto-peak.datb'))  

    if logging:
      #self.connect(self.signal_delay1, blocks.file_sink(gr.sizeof_gr_complex, 'logs/test-signal.dat'))
      self.connect((peak,0), blocks.file_sink(gr.sizeof_char, 'logs/peak.datb'))
      self.connect((peak,1), blocks.file_sink(gr.sizeof_float, 'logs/peak-cfo.dat'))
      self.connect(matched_filter, blocks.file_sink(gr.sizeof_float, 'logs/sync-mf.dat'))
      self.connect(M_d, blocks.file_sink(gr.sizeof_float, 'logs/sync-M.dat'))
      self.connect(P_d, blocks.file_sink(gr.sizeof_gr_complex, 'logs/sync-pd.dat'))
      self.connect(R_d, blocks.file_sink(gr.sizeof_float, 'logs/sync-rd.dat'))
      self.connect(R_d_squared, blocks.file_sink(gr.sizeof_float, 'logs/sync-rd-squared.dat'))
      self.connect(P_d_angle, blocks.file_sink(gr.sizeof_float, 'logs/sync-angle.dat'))
      self.connect(peak_detect, blocks.file_sink(gr.sizeof_char, 'logs/sync-peaks.datb'))



################################################################################
class ofdm_sync_fir(gr.hier_block2):
  """ An IEEE 802.11a/g/p OFDM Receiver for GNU Radio
      Bastian Bloessl, Michele Segata, Christoph Sommer and Falko Dressler
      SRIF’13, August 12, 2013, Hong Kong, China.
  """
  def __init__(self, fft_length, cp_length, preambles, sample_rate=10e6, mode='RAW', logging=False):
    gr.hier_block2.__init__(self, "ofdm_sync_fir",
      gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
      gr.io_signature3(3, 3,    # Output signature
        gr.sizeof_gr_complex,   # delayed input
        gr.sizeof_float,        # fine frequency offset
        gr.sizeof_char          # timing indicator
      )
    )

    period = 16
    window = 48
    symbol_length = fft_length+cp_length

    # Calculate the frequency offset from the correlation of the preamble
    x_corr = blocks.multiply_cc()
    P_d = blocks.moving_average_cc(window, 1.0)
    #P_d = gnuradio.filter.fir_filter_ccf(1, [1.0]*window)
    P_d_angle = blocks.complex_to_arg()
    period_delay = blocks.delay(gr.sizeof_gr_complex, period)
    conj = blocks.conjugate_cc()

    self.connect(self, period_delay, (x_corr, 0))
    self.connect(self, conj, (x_corr, 1))
    self.connect(x_corr, P_d, P_d_angle)

    # Get the power of the input signal to normalize the output of the correlation
    R_d = blocks.moving_average_ff(window, 1.0)
    #R_d = gnuradio.filter.fir_filter_fff(1, [1.0]*window)
    self.connect(self, blocks.complex_to_mag_squared(), R_d)

    ## using volk divide
    #M_d = blocks.divide_ff()
    M_d = raw.divide_ff()
    self.connect(P_d, blocks.complex_to_mag(), (M_d, 0))
    self.connect(R_d, (M_d, 1))

    peak_detect = raw.peak_detector2_fb(0.75)
    # NOTE: Trainning symbol format: STS STS LTS1 LTS2
    # because of the moving_average with parameter "window",  M_d outputs pulse which indicates the last sample of STS, but need to be adjusted
    # Cross correlation operation generate peaks only at the last sample of LTS
    # so, in order to reduce the cross-corr computation, 
    # we delay the peak flag with some delay(less than symbol_length to get some tolerance)
    # the threshold setted in peak_detector will intruduce some samples 
    if mode == 'PNC':
      autocorr_flag_delay = symbol_length*2-cp_length
    elif mode == 'RAW':
      autocorr_flag_delay = symbol_length
    autoflag_delay = blocks.delay(gr.sizeof_float, autocorr_flag_delay)

    self.connect(self, (peak_detect,0))
    self.connect(M_d,  autoflag_delay, (peak_detect,1))

    if sample_rate > 15e6:
      period_delay.set_min_output_buffer(96000)
      conj.set_min_output_buffer(96000)
      x_corr.set_min_output_buffer(96000)
      P_d.set_min_output_buffer(96000)
      P_d_angle.set_min_output_buffer(96000)
      M_d.set_min_output_buffer(96000)
      peak_detect.set_min_output_buffer(96000)
      autoflag_delay.set_min_output_buffer(96000)


    if False:
      self.connect(M_d, blocks.file_sink(gr.sizeof_float, 'logs/rx-sync-M_d.dat'))
      self.connect(autoflag_delay, blocks.file_sink(gr.sizeof_float, 'logs/rx-autoflag_delay.dat'))
      self.connect(peak_detect, blocks.file_sink(gr.sizeof_char, 'logs/rx-sync-peak.datb'))
      self.connect((peak_detect,1), blocks.file_sink(gr.sizeof_float, 'logs/rx-sync-peak.datf'))

    ##########################################################
    # peak_delay and offset are for cutting CP
    # FIXME: until we figure out how to do this, just offset by 6 or cp_length/2
    #symbol_length = fft_length+cp_length
    peak_delay = cp_length
    offset = 8  #cp_length/2 
    self.offset = offset

    # regenerate peak using cross-correlation
    # * RAW mode: use raw_generate_peak2 block to filter peaks
    # * PNC mode: use raw_generate_peak3 block to identify the proper peak for two users
    # PNC mode delays all input peak_delay samples
    # * cp_length: delay for cross-correlation since auto-correlation is not so accurate
    # * 3 symbol_length: delay for identifying mode for PNC
    # * -offset: for cp cut
    if mode == 'PNC':
      # The block structure is
      # signal -> 3*symbol_length+cp_length-offset     -> (self,0) [signal]
      # signal -> cp_length delay -> auto  -> (peak,0) -> (self,2) [peak]
      # signal -> cp_length delay          -> (peak,1) -> (self,1) [cfo]
      # cfo    -> cp_length delay          -> (peak,2)
      # 
      # we let cross's signal go faster to identify the beginning
      # we use cross's peak output to find cfo, so the delay of cfo should = cross delay
      # we use raw_regenerate_peak to do cross-corr, and symbols energy after STS to identify mode 
      # so the signal is further delayed by three symbols more
      self.cfo_cross_delay = blocks.delay(gr.sizeof_float, peak_delay)

      LOOK_AHEAD_NSYM = 0
      peak_cross_range = cp_length*2

      #self.connect(autoflag_delay, blocks.file_sink(gr.sizeof_float, 'logs/rx-autoflag_delay.dat'))

      peak = raw.regenerate_peak3(fft_length, fft_length+cp_length, LOOK_AHEAD_NSYM, peak_cross_range, preambles, False)
      self.connect(peak_detect,   (peak,0))
      self.connect(self,          (peak,1))
      self.connect(P_d_angle, self.cfo_cross_delay,    (peak,2))

      self.signal_delay = blocks.delay(gr.sizeof_gr_complex, autocorr_flag_delay+cp_length + peak_cross_range-offset)
      self.connect(self, self.signal_delay, (self,0))  # signal output
      self.connect((peak,1), (self,1))                 # cfo output
      self.connect((peak,0), (self,2))                 # peak output
      #self.connect((peak,1), blocks.null_sink(gr.sizeof_float))
      #self.connect((peak,0), blocks.null_sink(gr.sizeof_char))
      #self.connect(blocks.null_source(gr.sizeof_float), (self,1))
      #self.connect(blocks.null_source(gr.sizeof_char), (self,2))

      if logging:
        self.connect(self.cfo_cross_delay, blocks.file_sink(gr.sizeof_float, 'logs/input-cfo.dat'))
        #self.connect(cross, blocks.file_sink(gr.sizeof_float, 'logs/cross.dat'))
        self.connect(self.signal_cross_delay, blocks.file_sink(gr.sizeof_gr_complex, 'logs/cross-signal.dat'))

    if mode == 'RAW':
      LOOK_AHEAD_NSYM = 0
      peak = raw.regenerate_peak2(fft_length, fft_length+cp_length, \
                                              LOOK_AHEAD_NSYM, preambles, False)
      # do LTS cross correlation will set a flag at the last sample of LTS
      # signal delay is the same as the path(cross signal), but delay to pos(in 1st cp of LTS)
      # so as the sample cut the data from LTS to end;
      self.signal_delay = blocks.delay(gr.sizeof_gr_complex, symbol_length - offset)  #self.cross_delay + symbol_length - offset

      if sample_rate > 15e6:
        self.signal_delay.set_min_output_buffer(96000)
        #peak.set_max_noutput_items(2048)
        #peak.set_max_noutput_items(96000)


      self.cfo_delay = autocorr_flag_delay + peak_delay

      self.connect(peak_detect, (peak,0))
      self.connect(P_d_angle, blocks.delay(gr.sizeof_float, self.cfo_delay), (peak,1))
      self.connect(   self,   (peak,2))

      self.connect(self, self.signal_delay, (self,0))      # signal output
      self.connect((peak,1), (self,1))                # cfo output
      self.connect((peak,0), (self,2))                # raw peak output

      self.peak = peak

