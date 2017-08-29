#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2016 Lihao
#
from gnuradio import gr, digital, blocks
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from optparse import OptionParser
from phy_params import PNC_TYPE
from phy_params import ofdm_params  
from phy_params import PNC_TYPE
from uhd_interface import uhd_receiver
from Tkinter import *
from scipy import signal
from mac_params import string_to_hex_list
import array, struct, time, sys, math
import gnuradio.gr.gr_threading as _threading
import mac_params
import os
import raw
import struct
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio  
import random,time
import threading 
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from numpy import arange, sin, pi
import tkFont
#import os
#print 'Blocked waiting for GDB attach (pid = %d)' % (os.getpid(),)
#raw_input ('Press Enter to continue: ')

global csi,LTS,pkts_count,pkts_list,running_flag,ready_flag,plotting_data,result,plotting_flag,holdon_flag,f,breath_fre_filtered
f = None
holdon_flag = False
plotting_flag = True
plotting_data = list()
running_flag =False
ready_flag = False
csi = list()
pkts_list = list()
breath_fre_filtered=list()
a  = np.random.rand(1190,64)
a = a/10000
#a[:] = 0
#a = np.zeros((1190,64),np.float32)
pkts_list = a.tolist()
pkts_count = 0
LTS =[
      0.0000 - 0.0000j,
      0.0000 + 0.0000j,
      0.0000 + 0.0000j,
      0.0000 + 0.0000j,
      0.0000 + 0.0000j,
     -0.0000 + 0.0000j,
     -6.2952 + 4.9366j,
      2.4255 + 7.6235j,
      5.7135 - 5.5996j,
     -7.5109 + 2.7543j,
     -7.6708 - 2.2714j,
      3.0858 - 7.3809j,
      7.4767 - 2.8459j,
     -7.8102 + 1.7322j,
      5.6839 - 5.6297j,
      1.7752 - 7.8006j,
     -7.4013 - 3.0366j,
      4.3934 + 6.6856j,
      7.5909 - 2.5254j,
     -7.6760 - 2.2537j,
     -0.0898 - 7.9995j,
     -7.6591 + 2.3105j,
     -7.9456 + 0.9312j,
     -7.9993 - 0.1043j,
      7.9965 - 0.2350j,
     -4.0937 - 6.8732j,
     -7.3674 - 3.1179j,
     -7.8306 + 1.6375j,
     -4.7677 - 6.4241j,
      7.1099 + 3.6673j,
     -4.7978 - 6.4016j,
     -2.4849 + 7.6043j,
     -0.0000 + 0.0000j,
      4.8656 - 6.3503j,
      7.0433 - 3.7937j,
      7.9698 - 0.6945j,
     -5.3350 - 5.9614j,
     -2.3271 - 7.6540j,
      7.3225 - 3.2219j,
      4.8572 + 6.3567j,
     -1.2351 + 7.9041j,
      6.3683 + 4.8420j,
      7.0268 - 3.8241j,
      3.8621 + 7.0060j,
      4.6707 + 6.4950j,
      6.2755 - 4.9617j,
     -5.5160 - 5.7943j,
      3.7751 + 7.0533j,
     -6.6011 + 4.5195j,
      0.3198 - 7.9936j,
     -7.9144 + 1.1671j,
     -7.9225 + 1.1105j,
      7.7123 + 2.1263j,
     -0.9759 + 7.9402j,
     -3.9915 - 6.9331j,
     -5.5539 + 5.7580j,
     -0.5757 - 7.9793j,
     -6.1940 - 5.0630j,
      7.4395 - 2.9418j,
      0.0000 - 0.0000j,
     -0.0000 - 0.0000j,
     -0.0000 + 0.0000j,
     -0.0000 + 0.0000j,
      0.0000 - 0.0000j];

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


class OFDM_RX(gr.hier_block2):
  def __init__(self, options,callback=None):
    p = ofdm_params(options)
    nsym = options.size+len(p.preambles)

    gr.hier_block2.__init__(self, "OFDM_RX",
      gr.io_signature(1,1, gr.sizeof_gr_complex),
      gr.io_signature(0,0,0))

    ################################################################################
    #                        Block Definition and Connection
    ################################################################################
    # to the synchronization algorithm
    #sync = ofdm_sync(p.fft_length, p.cp_length, p.cross_corr_preambles, 'RAW', options.log)
    sync = ofdm_sync_fir(p.fft_length, p.cp_length, p.cross_corr_preambles, options.bandwidth, 'RAW', options.log)
    sampler = raw.ofdm_sampler(p.fft_length, p.fft_length+p.cp_length, long(options.bandwidth), timeout=nsym, debug=False)
    self.connect(self, sync)
    self.params = p
    self.size = options.size
    self.callback = callback
    self._rcvd_pktq = gr.msg_queue()
    message_sink = blocks.message_sink(gr.sizeof_gr_complex*p.fft_length,self._rcvd_pktq,True)
    self.connect((sync,0), (sampler,0))
    self.connect((sync,1), (sampler,2))
    self.connect((sync,2), (sampler,1)) 
    self.connect(sampler,message_sink)
    if options.log:
      self.connect((sync,0),
                   blocks.file_sink(gr.sizeof_gr_complex, 'logs/rx-sync_0.dat'))  #signal output 
      self.connect((sync,1),
                   blocks.file_sink(gr.sizeof_float, 'logs/rx-sync_1.datf'))  #cfo output
      self.connect((sync,2),
                   blocks.file_sink(gr.sizeof_char, 'logs/rx-sync_2.datc'))  #peak out
      self.connect(sampler,blocks.file_sink(gr.sizeof_gr_complex*p.fft_length, 'sampler.dat'))  #LTS output 
    self._watcher = _queue_watcher_thread(self._rcvd_pktq, self.callback)
  def add_options(normal, expert):
    normal.add_option("-s", "--size", type="int", default=16,
                      help="set number of symbols per packet [default=%default]")
    normal.add_option("", "--node", type="string", default=None,
                      help="select pnc endnode: A or B [default=None]")
    ofdm_params.add_options(normal, expert)
  add_options = staticmethod(add_options)

class _queue_watcher_thread(_threading.Thread):
    def __init__(self, rcvd_pktq, callback=None):
        _threading.Thread.__init__(self)
        self.setDaemon(1)
        self.rcvd_pktq = rcvd_pktq
        self.callback = callback
        self.keep_running = True
        self.start()


    def run(self):
        while self.keep_running:
            time.sleep(0.01)
            msg = self.rcvd_pktq.delete_head()
            if self.callback:
                self.callback(msg)


class ofdm_receive_path(gr.hier_block2):
  """
  Generic receive path:
    @note: input samples
    @note: provide PNC, RAW receive path, that depends on MODE
  """
  def __init__(self, options_rx,callback=None):
    if options_rx.mode == "RAW":
      ofdm = OFDM_RX(options_rx,callback)
    gr.hier_block2.__init__(self, "ofdm_receive_path",
                            gr.io_signature(1, 1, gr.sizeof_gr_complex),
                            gr.io_signature(0, 0, 0))

    if options_rx.mode == "RAW":
      self.connect(self,ofdm)
  def add_options(normal, expert):
    """
    Adds receiver-specific options to the Options Parser
    """
    expert.add_option("", "--bitrate", type="int", default=1,
                      help="set bitrate index [default=%default]")
    expert.add_option("","--sync-log", action="store_true", default=False,
                      help="log all samples after synchronizaiton")
    expert.add_option("", "--log", action="store_true", default=False,
                      help="log all source samples (CAUTION: lots of data)")
    OFDM_RX.add_options(normal, expert)
  add_options = staticmethod(add_options)


class my_top_block(gr.top_block):
  """
    Three modes of operation:
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
    self.callback = callback
    rx = ofdm_receive_path(options,self.callback)
    self.connect(u, rx)

    if options.profile:
      from gnuradio.ctrlport import monitor
      self.ctrlport_monitor = monitor()
      self.ctrlport_monitor_performance = monitor("gr-perf-monitorx")

    
  
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
    ofdm_receive_path.add_options(normal, expert)
  add_options = staticmethod(add_options)

class CSI_GUI(Frame):
    def __init__(self, parent,my_top_block):
      self.fre_breathe_value = StringVar()
      self.COUNT = 1e5
      self.spilt_total=0
      self.i = 0
      self.tb = my_top_block
      self.f = Figure(figsize=(5, 4), dpi=100)
      self.a = self.f.add_subplot(111)
 #     self.t = arange(0, 60, 0.05)
 #     self.s = self.t  
      myfont = matplotlib.font_manager.FontProperties(fname='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
      self.a.set_title(u"实时呼吸波形",fontproperties=myfont)
      self.p = Figure(figsize=(5, 4), dpi=100)
      self.b = self.p.add_subplot(111)
 #     self.t = arange(0, 60, 0.05)
 #     self.s = self.t  
      self.b.set_title(u'呼吸频率统计',fontproperties=myfont)
      Frame.__init__(self, parent)
      self.parent = parent
      self.initUI()


    def initUI(self):
      #client side
      self.parent.title("呼吸检测系统")
      self.pack(fill=BOTH, expand=1)
      self.receiveButton = Button(self, text=u'开始检测呼吸频率', width = 20, height = 2, command=self.start_function)
      self.receiveButton.place(x=600,y=250)
      self.receiveButton = Button(self, text=u'停止检测', width = 20, height = 2, command=self.stop_function)
      self.receiveButton.place(x=220,y=250)
      ft1 = tkFont.Font(family = 'Fixdsys',size = 30,weight = tkFont.BOLD)
      ft2 = tkFont.Font(family = 'Fixdsys',size = 25,weight = tkFont.BOLD)
      self.label_fre_breathe = Label(self, text="呼吸频率  (每分钟) :",borderwidth=15,font = ft1 )
      self.label_fre_breathe.place(x=400,y=100)
      self.fre_breathe_text = Label(self, textvariable=self.fre_breathe_value,font = ft2)
      self.fre_breathe_text.place(x=800,y=115)
      self.canvas = FigureCanvasTkAgg(self.f, master=self.parent)
      self.canvas.show()
      self.canvas.get_tk_widget().pack(side=LEFT, fill=X, expand=1)
      self.canvas1 = FigureCanvasTkAgg(self.p, master=self.parent)
      self.canvas1.show()
      self.canvas1.get_tk_widget().pack(side=LEFT,fill=X ,expand=1)

    def csi_getting(self,pkts=None):
      global csi,LTS
      LTS = np.array(LTS)
      i = 0
      while i<=(len(pkts)-1):
        pkt_a = np.array(pkts[i])
        fft_pkt = np.fft.fftshift(np.fft.fft(pkt_a))
        csi_pkt = fft_pkt/LTS
        csi_pkt[0:6] = 0
        csi_pkt[32] = 0
        csi_pkt[59:64] = 0
        RSS= sum(abs(csi_pkt**2))/len(pkt_a)
        csi.append(RSS)
        i +=1
      print 'csi list appending successfully'

    def hampel(self,X, Y, DX, T,Threshold):
      X.shape = len(X),1
      Y.shape = len(Y),1
      SortX   = np.sort(X)
      IdxNaN  = np.isnan(X)|np.isnan(Y)
      X = X[~np.isnan(X)]  
      Y = Y[~np.isnan(Y)] 
      YY  = Y;
      I   = np.zeros(Y.shape,dtype=bool)
      S0 = np.zeros(YY.shape,dtype=float)
      S0[:] = np.nan
      Y0 = np.zeros(YY.shape,dtype=float)
      Y0[:] = np.nan 
      ADX = np.tile(DX, Y.shape) 
      # Preallocate  :adaptive
      Y0Tmp = np.zeros(YY.shape,dtype=float)
      Y0Tmp[:] = np.nan
      S0Tmp = np.zeros(YY.shape,dtype=float)
      S0Tmp[:] = np.nan
      tmp = np.arange(1,S0.size+1)
      DXTmp   = tmp.conj().T*DX  # Integer variation of Window Half Size
      #Calculate Initial Guess of Optimal Parameters Y0, S0, ADX
      i = 0
      while i<Y.size:
          j     = 0;
          S0Rel = np.inf
          while S0Rel > Threshold:
          # Calculate Local Nominal Data Reference value
          # and Local Scale of Natural Variation using DXTmp window
              result= self.localwindow(X, Y, DXTmp[j], i)
              Y0Tmp[j] = float(result[0])
              S0Tmp[j] = float(result[1])
          #Calculate percent difference relative to previous value
              if j > 0:
                  S0Rel   = abs((S0Tmp[j-1] - S0Tmp[j])/(S0Tmp[j-1] + S0Tmp[j])/2.0)
              j = j+1
          Y0[i]   = float(Y0Tmp[j-2])     #Local Nominal Data Reference value
          S0[i]   = float(S0Tmp[j-2])     #Local Scale of Natural Variation
          ADX[i]  = float(DXTmp[j-2]/DX)  #Local Adapted Window size relative to DX
          i = i+1;
      # Gaussian smoothing of relevant parameters
      DX  = 2*self.median(SortX[1:] - SortX[0:-1])
      ADX = self.smgauss(X, ADX, DX)
      S0  = self.smgauss(X, S0, DX)
      Y0  = self.smgauss(X, Y0, DX)  
      Y.shape = len(Y),1
      # Prepare Output 
      Idx     = abs(Y - Y0) > T*S0   # Index of possible outlier  
      YY = Y0[Idx]              # Replace outliers with local median value
      #Reinsert NaN values detected at error checking stage
      if any(IdxNaN):
          YY   = self.rescale(IdxNaN, YY);
      return YY  

    def localwindow(self,X, Y, DX, i):
        # Built-in functions
        Idx = (X[i]-DX <= X) & (X <= X[i]+ DX)
        #Index relevant to Local Window
        Y0  = self.median(Y[Idx])
        S0  = 1.4826*self.median(abs(Y[Idx] - Y0));
        return (Y0, S0)    

    def median(self,YM):
        # Isolate relevant values in Y
        YM  = np.sort(YM)
        NYM = YM.size        
        #Calculate median
        if NYM%2:   #Uneven
            M   = YM[(NYM+1)/2-1]
        else:           # Even
            M   = (YM[NYM/2]+YM[NYM/2-1])/2.0
        return M    

    def smgauss(self,X, V, DX):
        #Prepare Xj and Xk
        X_tmp = X.reshape(len(X),1)
        V.shape = len(V),1
        Xj =  np.tile(X_tmp.T, (X_tmp.size, 1))
        Xk  = np.tile(X_tmp, (1, X_tmp.size))
        #Calculate Gaussian weight
        Wjk = np.exp(-(((Xj-Xk)/float(2*DX))**2))
        #Calculate Gaussian Filter
        tmp = np.array(sum(Wjk,0))
        tmp.shape = len(tmp),1
        G= ((Wjk).dot(V))/tmp
        return G
        
    def rescale(self,IdxNaN, YY):
        #Output Rescaled Elements
        Element =YY
        if is_numeric(YY):
            Element =np.zeros(Y.shape,dtype=bool)
        elif is_logical(YY):
            Element =np.zeros(YY.shape,dtype=float)
            Element[:] = np.nan
            X[~np.isnan(X)] = Element  
        return X
    def smooth(self,signal,count):
        D = signal
        k = count
        nn = len(signal)
        b=(k-1)/2
        Z = list()
        if(nn>k):
            i=0
            while i<nn:
                if(i<b):
                    Z.append(0)
                    j = -i
                    while j<i+1:
                        Z[i]+=D[i+j]
                        j = j+1
                    Z[i]=Z[i]/(2*i+1.0) 
                elif(((i>b)|(i==b))&((nn-i)>b)):
                    Z.append(0)
                    j = -b
                    while j<b+1:
                        Z[i]+=D[i+j]
                        j = j+1
                    Z[i]=Z[i]/float(k)
                else:
                    Z.append(0)
                    i1=(nn-1)-i
                    j=-i1
                    while j<i1+1:
                        Z[i]+=D[i+j]
                        j = j+1
                    Z[i]=Z[i]/(2*i1+1.0)
                i=i+1
        return np.array(Z)     
    def csi_main(self):
      global pkts_list,running_flag,ready_flag,csi,f,breath_fre_filtered
      running_flag = True
      fft_size = 1024
      fft_step = 5*20
      size_w = 5
      bpf = np.zeros(fft_size,dtype=float)
      bpf[4:26] = 1
      q = 10
      alpha = 1/4
      breath_fre = list()
      count = 1
      while running_flag ==True:
        if ready_flag ==True:
          self.csi_getting(pkts_list)
          pkts_list = pkts_list[-1100:]
          csi = np.array(csi)
          RSS =csi

          OA = RSS     #get the CSI 
          OT = arange(1.,len(OA)+1)
          DX          =    0.01     #Window Half size
          T           =    0.01     #Threshold
          Threshold   =    0.01     #AdaptiveThreshold
          y = self.hampel(OT,OA, DX, T,Threshold)
          yy = self.smooth(y, 30)
          RSS = yy

          rss_removed = np.zeros(len(RSS)-size_w*20,dtype=float)

          i = size_w*10
          while i<len(RSS)-size_w*10:
              rss_removed[i-size_w*10]=RSS[i]-np.mean(RSS[i-size_w*10:size_w*10+i-1])
              i = i+1

#          OA = rss_removed     #get the CSI 
#          OT = arange(1.,len(OA)+1)
#          DX          =    0.01     #Window Half size
#          T           =    0.01     #Threshold
#          Threshold   =    0.01     #AdaptiveThreshold
#          y = self.hampel(OT,OA, DX, T,Threshold)
#          yy = self.smooth(y, 30)
#          rss_removed = yy

          i = 0
          fd  = np.fft.fft(rss_removed[i:i+fft_size])
          fd =fd*bpf
          tmp_fd = abs((fd*bpf)**2).tolist()
          ind = tmp_fd.index(max(tmp_fd))+1
          breath_fre.append(round(60*ind*20.0/float(fft_size)))
          fd[fd<fd[ind-1]/4.0] = 0
          recover_bre_siganl=np.fft.ifft(fd).real
          count = count+1
          result = recover_bre_siganl
          self.f.clf()
          self.a = self.f.add_subplot(111)
#          self.t = arange(0, 60, 0.05859375)
#          self.a.plot(self.t, rss_removed)
          self.a.set_ylim([-0.00070, 0.00070])
          self.a.plot(rss_removed)
          myfont = matplotlib.font_manager.FontProperties(fname='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
          self.a.set_title(u"实时呼吸波形",fontproperties=myfont)
          self.canvas.show()
          self.canvas.get_tk_widget().pack(side=LEFT, fill=X, expand=1)
          self.update()
          if count ==q:
              breath_fre_array = np.array(breath_fre)
              A = np.sort(breath_fre_array)
              frequency = round(np.mean(A[np.ceil(q*alpha)+1: q-np.ceil(q*alpha)]))
              breath_fre_filtered.append(frequency)
      #        f.write(str(frequency))
              count  =q-1
              self.fre_breathe_value.set(str(frequency))
              self.p.clf()
              self.b = self.p.add_subplot(111)
              self.b.set_ylim([4, 28])
              self.b.set_xlim([1, 20])
              self.b.plot(breath_fre_filtered[-20:])
              self.b.set_title(u'呼吸频率统计',fontproperties=myfont)
              self.canvas1.show()
              self.canvas1.get_tk_widget().pack(side=LEFT, fill=X, expand=1)
              self.update()
        #      f.write(str(frequency))
              breath_fre = breath_fre[-(q-1):]
              self.update()
          ready_flag = False
          csi = list()

    def start_function(self):
      t=threading.Thread(target=self.GUI_main)
      t.start()

    def stop_function(self):
      global running_flag
      running_flag =False
      plotting_flag = False
      self.tb.stop()


    def GUI_main(self):
        print time.clock()
        self.tb.start()                      # start flow graph
        self.csi_main()
        self.tb.wait()

# /////////////////////////////////////////////////////////////////////////////
#                                   main
# /////////////////////////////////////////////////////////////////////////////
def main():
    global f
    f = file('log.txt','a+')
    parser = OptionParser(option_class=eng_option, conflict_handler="resolve")
    expert_grp = parser.add_option_group("Expert")  
    my_top_block.add_options(parser, expert_grp)  
    (options, args) = parser.parse_args ()
    options.mode = "RAW"
    # build the graph 
    def callback(msg):
      global pkts_list,ready_flag
      binary_pkt = msg.to_string()
      tmp = np.fromstring(binary_pkt,dtype=np.float32)
      tmp.shape = len(tmp)/2,2
      pkt = np.vectorize(complex)(tmp[:,0], tmp[:,1])
      if len(pkt)==64:
        pkts_list.append(pkt)
      elif len(pkt)%128==0:
        count = len(pkt)/128
        for i in range(count):
          pkt_a = np.array(pkt[128*i:64+128*i])
          pkt_b = np.array(pkt[64+128*i:128+128*i])
          CFO = np.angle(sum((pkt_b*pkt_a)))/80
          pkt_a = pkt_a*np.exp(CFO*(-1j))
          pkts_list.append(pkt_a)
      else:
        pass
      print len(pkts_list)
      if len(pkts_list)==1200:
        ready_flag =True

    tb = my_top_block(options,callback)
    r = gr.enable_realtime_scheduling()
    if r != gr.RT_OK:
      print "Warning: failed to enable realtime scheduling"
    root = Tk()
    root.geometry("1000x800+300+300")
    app = CSI_GUI(root,tb)
    root.mainloop() 
if __name__ == '__main__':
    try:
      main()
      if f!=None:
        f.close()
    except KeyboardInterrupt:
      pass


#kill -9 $(ps auxf | grep "get_csi_version3.py" | awk '{print $2}')
