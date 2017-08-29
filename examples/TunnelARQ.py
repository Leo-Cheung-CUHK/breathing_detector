#!/usr/bin/env python
#
# Copyright 2013 Yue Li
# Copyright 2014 Lizhao You
#
import math,random,time,struct
import socket,sys,threading
import Image, numpy, argparse
import collections

import mac_frame_handler
from mac_params import string_to_hex_list
from mac_params import ENDNODE_FRAME_FORMAT, MAC_HEADER_LEN, CRC_LEN
from mac_params import TOTAL_ARQ_NSEQ as NSEQ
from mac_params import TOTAL_MAC_NSEQ as NSEQ_MAC

import collections

from gnuradio import digital

global WSIZE
WSIZE = 0
global fwriter
fwriter = None

NULL_MODE = 1
RTT_MODE = 6

PRINT_PING_DEBUG = 0

# =============================================================================

# /////////////////////////////////////////////////////////////////////////////
#                            Buffer Controller for MAC
# /////////////////////////////////////////////////////////////////////////////
# 1) Manage tx buffer from APP and to PHY;
# 2) Manage rx buffer from PHY and to APP;
# 3) Choose ARQ mode: NULL;

PRINT_BUFFER_FLAG = 0

class NullARQBuffer:
  def __init__(self,content,node,framebytes,windowsize=8,arq_mode=NULL_MODE,dack=False,infile=None,prt_msgq=None):
    self.c = content
    if type(self.c) == 'str':
      self.c = map(ord,self.c)

    self.frame = mac_frame_handler.EndnodeFrame(framebytes,node)

    # reset the MAC header size
    # it seems that the TAP interface (Ethernet) could return 1514 bytes
    self.frame.set_header_size(15)

    self.FORMAT = self.frame.f
    self.pktsize = self.frame.pktsize
    self.headersize = self.frame.headersize
    self.npkt = float(len(self.c))/self.pktsize
    self.npkt = int(math.ceil(self.npkt))

    # A queue to store the Send time of each packet -- for RTT calculation
    self.start_time = time.time()
    self.tx_time = [0] * NSEQ
    self.ack_time = [0] * NSEQ
    self.avg_rtt = 0
    self.alpha = 0.05
    self.first_rtt = True
    self.rtt_file = None
    #if node=='A':
    #  self.rtt_file = open("RTT_Result_A.dat",'w')
    #else:
    #  self.rtt_file = open("RTT_Result_B.dat",'w')
    #self.rtt_file.write("\n=============================\n")

    self.node = node
    self.arq_mode = arq_mode
    self.NULL = 129

    self.tunnelMode = False  # default no tunnel
    self.warpMode   = False  # default no warp transmission

    global fwriter
    #if self.node == 'A':
    #  fwriter=open('ARQ_TXLOG_A.dat', 'w')
    #if self.node == 'B':
    #  fwriter=open('ARQ_TXLOG_B.dat', 'w')

    #self.tx_buffer = []             # as a queue
    self.tx_buffer = collections.deque(maxlen=30)
    self.tx_oqueue = [[]]*NSEQ_MAC  # tx's output queue to PHY layer, implemented using list
    self.rx_buffer = [[]]*NSEQ      # rx's input queue from PHY layer, implemented using list
    #self.rx_oqueue = []             # rx's output queue to APP layer

    self.f = infile
    self.prt_msgq = prt_msgq

    if self.arq_mode == NULL_MODE:
      self.arq = NULL_ARQ(node)
    elif self.arq_mode == RTT_MODE:
      self.arq = RTT_ARQ(node,self.prt_msgq)

    global WSIZE
    WSIZE = windowsize 

    self.index = 0  # current index of the tx buffer

    self.lock = threading.Lock()

    # update tx queue during initialization
    for i in range(self.npkt):
      if type(self.c[0]) == str:
        buff = self.c[self.pktsize*i:self.pktsize*(i+1)]
      elif type(self.c[0]) == int:
        buff = map(chr,self.c[self.pktsize*i:self.pktsize*(i+1)])

      #if len(buff) != self.pktsize:
      #  if type(buff) == list:
      #    buff = buff + [chr(self.NULL)] * (self.pktsize-len(buff))
        #if type(buff) == str:  # XXX: believe it is list; we convert it in line 35
        #  buff = buff + struct.pack('!B',self.NULL) * (self.pktsize-len(buff))
      self.tx_buffer.append(buff)
      self.index = (self.index+1)%NSEQ

    self.recstring = ""

    # for statistics
    self.last_pktno = 0
    self.lost_npkts = 0
    self.recv_npkts  = 0

  def setTunnelMode(self):
    # Tunnel mode: extract the payload from rx pkt (indicated by pktlen)
    self.tunnelMode = True

  def setWrapMode(self):
    # Wrap mode: transmit a file repeatly
    # wrap tx is implemented using pop and insert, and the insert position is related to self.npkt
    # XXX: conflict with the bufferUpdate function
    # XXX: a better strategy is to realize warp tx in upper layer, and do flow control
    self.warpMode = True

  def bufferUpdate(self,content):
    # Func: put content into the tx_buffer (usually used with TAP interface)
    # - put an packet into the tx_buffer, to the next position
    # - the position index is maintained by self.index
    # XXX: the bufferUpdate function conflicts with the warp tx mode

    clen = len(content)

    # to make sure 
    # (a) it is an valid update; 
    # (b) each Ethernet frame is smaller than PHY-layer frame size
    assert(clen > 0)
    #assert(clen <= self.pktsize)
      
    while len(content) > 0:
      buff = content[:self.pktsize]
      content = content[self.pktsize:]

      # the standard format is char list
      if type(buff) == str:
        buff = list(buff)
      #if len(buff) < self.pktsize:
      #  if type(buff) == list:
      #    buff = buff + [chr(self.NULL)] * (self.pktsize-len(buff))
      #  if type(buff) == str:
      #    buff = buff + struct.pack('!B',self.NULL) * (self.pktsize-len(buff))

      self.lock.acquire()
      self.tx_buffer.append(buff)
      self.index = (self.index+1)%NSEQ
      self.lock.release()
      self.npkt += 1

      if len(buff) > 41 and PRINT_PING_DEBUG:
        ostr1 =  "[TX] Get index=%d len=%d/%d | ###icmp_seq=%d### time=%f " %(self.index, len(buff), clen, ord(buff[41]), time.time())
      else:
        ostr1 =  "[TX] Get index=%d len=%d/%d queue_len=%d | time=%f " %(self.index, len(buff), clen, len(self.tx_buffer), time.time())
      self.prt_msgq.put(ostr1)

  def deliverRxFrames(self,packet):
    # TODO: deliver rx frames to APP layer
    self.recstring = self.recstring + packet
    pass

  def calculate_per(self, pktno, nright=0):
    # sometimes some packets are missed because the receiver is not ready
    # this should not be counted
    if nright == 1:
      self.lost_npkts = 0
      self.recv_npkts = 1
      self.last_pktno = pktno
      return self.lost_npkts * 1.0 / self.recv_npkts

    tno = pktno
    if self.last_pktno > pktno:
        # assume: gap larger than 64 is impossible
        #assert(self.last_pktno > NSEQ/2+NSEQ/4 and pktno < NSEQ/2-NSEQ/4)
        if not (self.last_pktno > NSEQ/2+NSEQ/4 and pktno < NSEQ/2-NSEQ/4):
            print "########## FATAL Error: lpktno=%d pktno=%d " % (self.last_pktno, pktno)
        # Modified for RTT
        tno = tno + NSEQ - 1  # seq starts from 1
        ####

    #print "[PER] lpktno=%d pktno=%d lnpkts=%d rnpkts=%d %d/%d" % (self.last_pktno, pktno, tno-self.last_pktno-1, tno-self.last_pktno, self.lost_npkts, self.recv_npkts)        
    self.lost_npkts += (tno-self.last_pktno-1)
    self.recv_npkts += (tno-self.last_pktno) 
    self.last_pktno = pktno
    return self.lost_npkts * 1.0 / self.recv_npkts

  def updateRxQueue(self,rxframe,beacon_seq=0):
    # Func: receive a frame from PHY, update RxQueue and RxARQ
    # Input:
    #   @rxframe: received frame, including MAC header and payload
    #   @beacon_seq: for debug only
    # Output: received packets or None
    #   - None: already received packet, out of order
    #   - For SR: probably several packets
    (rec_seq,) = struct.unpack('!B',rxframe[self.FORMAT.RECV_SEQ_POS])
    rec_packet = rxframe[self.headersize:self.headersize+self.pktsize]
    (rec_ack,) = struct.unpack('!B',rxframe[self.FORMAT.RECV_ACK_POS])
    (rec_len,) = struct.unpack('!H',rxframe[self.FORMAT.PKTLEN_POS:self.FORMAT.PKTLEN_POS+2])

    if self.prt_msgq is not None:
      if rec_len>20+41 and PRINT_PING_DEBUG:
        ostr = "[RX] UpdateRxQueue: beacon#=%d rec_seq=%d rec_ack=%d rec_len=%d | ###icmp_seq=%d###" % (beacon_seq,rec_seq,rec_ack,rec_len,ord(rxframe[20+41]))
      else:
        ostr = "[RX] UpdateRxQueue: beacon#=%d rec_seq=%d rec_ack=%d rec_len=%d" % (beacon_seq,rec_seq,rec_ack,rec_len)
      self.prt_msgq.put(ostr)

    if self.tunnelMode:
      rec_packet = rec_packet[:rec_len]


    ###########################################################################
    # Calculate packet error rate
    per = 0
    if rec_seq > 0:
      per = self.calculate_per(rec_seq)
      ostr = "[RX] ##### RESULT: seq=%d per=%f" % (rec_seq,per)
      self.prt_msgq.put(ostr)

    # Calculate round trip time
    if rec_seq > 1:
      if rec_ack != 0:
        self.ack_time[rec_ack] = time.time()
        rtt = (self.ack_time[rec_ack]-self.tx_time[rec_ack])*1000
        #calculate average RTT
        if self.avg_rtt == 0:
          self.avg_rtt = rtt
          ostr = "[RX] ##### RESULT: seq=%d per=%f ack=%d cur=%6.3fms avg=%6.3fms" % (rec_seq,per,rec_ack,rtt,self.avg_rtt)
          self.prt_msgq.put(ostr)
        else:
          self.avg_rtt = (1-self.alpha)*self.avg_rtt + self.alpha*rtt
          ostr = "[RX] ##### RESULT: seq=%d per=%f ack=%d cur=%6.3fms avg=%6.3fms" % (rec_seq,per,rec_ack,rtt,self.avg_rtt)
          self.prt_msgq.put(ostr)
        if self.rtt_file is not None:
          self.rtt_file.write(ostr)
      else:
        rtt = 0
    ###########################################################################

    if self.node == "A" and PRINT_BUFFER_FLAG:
      print "[RX] Node A RX ", len(rec_packet), string_to_ord_list(rec_packet[30:50])
    elif self.node == "B" and PRINT_BUFFER_FLAG:
      print "\t\t\t\t\t[RX] Node B RX ", len(rec_packet), string_to_ord_list(rec_packet[30:50])

    #print "self.arq_mode=%d, NULL_MODE=%d" %(self.arq_mode, NULL_MODE)
    if self.arq_mode == NULL_MODE:
      # for null ARQ mode
      return rec_packet
    else:
      # for ARQ mode beyond NULL ARQ
      r = self.arq.update(rec_seq,rec_ack)
      if r:
        return rec_packet
      else:
        return None

  def extractTxFrame(self,beacon_seq=0,burst_seq=None):
    # Function: 
    #   - extract a frame for transmission: get self_seq from TxARQ, and the data; also self_ack
    #   - update TxQueue: if beacon_seq valid, use beacon_seq; ow., use burst_seq
    # Input:
    #   @beacon_seq: cur beacon seq num
    #   @burst_seq:  cur burst seq num
    #   @data:       assume already in the self.tx_buffer
    # Output:
    #   tx_frame:    the frame to be transmitted  
    self_sack = None
    if self.arq_mode == NULL_MODE:
      self_seq, self_ack = self.arq.retrieve()
    elif self.arq_mode == RTT_MODE:
      self_seq, self_ack = self.arq.retrieve(beacon_seq)
      #add the send starting time in queue
      self.tx_time[self_seq] = time.time()

    if self.prt_msgq is not None:
      if burst_seq is None:
        ostr = "[TX] InsertTxQueue: beacon#=%d burst#=None self_seq=%d self_ack=%d " % (beacon_seq,self_seq,self_ack)
      else:
        ostr = "[TX] InsertTxQueue: beacon#=%d burst#=%d self_seq=%d self_ack=%d " % (beacon_seq,burst_seq,self_seq,self_ack)
      #print ostr
      self.prt_msgq.put(ostr)

    # extract tx_pkt, depending re-tx mode or tx mode
    tx_frame_list = self.tx_buffer
    pktlen = 0
    if len(tx_frame_list) == 0:  # if no pkts from APP layer, generate a dummy pkt
      #return None
      ostr1 = "[Tx] Dummy packet | queue_len=%d" % (len(tx_frame_list))
      self.prt_msgq.put(ostr1)
      tx_pkt = [chr(self.NULL)] * self.pktsize
    else:                        # if not found in output queue, get it from APP layer
      self.lock.acquire()
      tx_pkt = tx_frame_list[0]

      # clean up tx data for NULL_ARQ
      if self.arq_mode == NULL_MODE or self.arq_mode == RTT_MODE:
        tx_frame_list.popleft()
      self.lock.release()

      pktlen = len(tx_pkt)
      if pktlen>41 and PRINT_PING_DEBUG:
        ostr1 = "[Tx] Valid packet | pktlen=%d ###icmp_seq=%d### queue_len=%d " % (pktlen,ord(tx_pkt[41]),len(tx_frame_list))
      else:
        ostr1 = "[Tx] Valid packet | pktlen=%d queue_len=%d " % (pktlen,len(tx_frame_list))
        #ostr2 = str(map(ord,tx_pkt[26:50]))
      self.prt_msgq.put(ostr1)

      if pktlen != self.pktsize:
         if type(tx_pkt) == list:
           tx_pkt = tx_pkt + [chr(self.NULL)] * (self.pktsize-pktlen)
         if type(tx_pkt) == str:  # XXX: believe it is list; we convert it in line 35
           tx_pkt = tx_pkt + struct.pack('!B',self.NULL) * (self.pktsize-pktlen)
           tx_pkt = list(tx_pkt)

    #ostr = "[RX] {nARQ}: pktlen=%d" % (pktlen)
    #self.prt_msgq.put(ostr)

    if self.node == "A" and PRINT_BUFFER_FLAG:
      print "[TX] Node A PRE_RECV_ACK:", self.arq.pre_recv_ack 
      print "[TX] Node A TX_SEQ:", self_seq, string_to_ord_list(tx_pkt)
    if self.node == "B" and PRINT_BUFFER_FLAG:
      print "\t\t\t\t\t[TX] Node B PRE_RECV_ACK:", self.arq.pre_recv_ack 
      print "\t\t\t\t\t[TX] Node B TX_SEQ:", self_seq, string_to_ord_list(tx_pkt)

    # update self_seq and tx_frame
    tx_frame_data = self.frame.generate_frame_data(beacon_seq,self_seq,self_ack,tx_pkt,burst_seq,self_sack,pktlen)

    if fwriter:
      x = map(ord,tx_pkt[:10])
      s = "[ARQ_TX] %d %d %d %d %d \n" % (self_seq, burst_seq, x[0], x[5], x[9])
      fwriter.write(s)
    
    # update tx_oqueue for XORing
    if burst_seq is not None:  # for burst mode
      self.tx_oqueue[burst_seq]  = tx_frame_data
    else:                      # for beacon mode
      self.tx_oqueue[beacon_seq] = tx_frame_data
      
    return tx_frame_data

  def getTxFrame(self,txseq):
    # Func: get a transmitted frame of txseq for XORing
    assert(txseq < NSEQ_MAC)
    return self.tx_oqueue[txseq]

  def getRxBufferSize(self):
    # Func: return the current received packets
    return len(self.recstring)*1.0 / self.pktsize

# ////////////////////////////////////////////////////// #
#                    ARQ Protocols                       #
# ////////////////////////////////////////////////////// #

def IsSeqSmaller(a_seq,b_seq):
  if a_seq == None or b_seq == None:
    return False
  if abs(a_seq-b_seq) > WSIZE: # we are on the seq boundary
    if a_seq > b_seq:  # e.g., a_seq=127 b_seq=1
      return True
    else:
      return False
  else:
    return a_seq < b_seq

def GetNextSeqNum(seq,recv_ack):
  # If recv_ack is larger than seq, use recv_ack
  # Since we use warp seq, we should compare them carefully
  # Depend on whether seq/recv_ack is small (less than 8)

  #print seq, recv_ack
  seqAr = (seq+1)%NSEQ
  seqAd = (seq+1)/NSEQ

  if recv_ack is None:
    return seqAr

  if IsSeqSmaller(seqAr,recv_ack):
    return recv_ack
  else:
    return seqAr

  # TODO: remove the following codes
  # TODO: refractor following programs with IsSeqSmaller function
  #if recv_ack <= WSIZE:
  #  if seqAr <= WSIZE:
  #    return max(seqAr, recv_ack)
  #  else:  # no matter seqAd==0 or seqAd==1
  #    return seqAr
  #elif seqAd == 0 and recv_ack > WSIZE:
  #  return max(seqAr,recv_ack)
  #elif seqAd == 1 and recv_ack > WSIZE:
  #  return seqAr

class RTT_ARQ:
  ###################################
  # Functions:
  # Only use ACK to compute Round-Trip Time
  # No repeat ACK even the packet lost during transmission
  ###################################

  def __init__(self,node,prt_msgq,infile=None):
    self.pre_recv_ack = 0  # should transmit first packet

    self.seq = 0 # cur tx seq no
    self.ack = 0 # cur tx ack no

    self.TX_STATE = ''
    self.RX_STATE = ''

    self.node = node
    self.infile = infile
    self.prt_msgq = prt_msgq

    self.lock = threading.Lock()

    self.retrieve_count = 0

    self.ack_queue = []

  def retrieve(self,beacon_nu):
    if self.retrieve_count == 0:
      # Modified for RTT
      self.seq=1
    else:
      self.seq = (self.seq+1)%NSEQ
      # Modified for RTT
      if self.seq == 0:
        self.seq = self.seq + 1
      #####
    self.retrieve_count = self.retrieve_count + 1    

    # self.ack is used in both retrieve and update, so we need lock
    ack_pkt_seq = 0
    self.lock.acquire()
    if len(self.ack_queue) > 0:
      ack_pkt_seq = self.ack_queue.pop(0)
    # Midified for RTT
    r = (self.seq, ack_pkt_seq)
    #####
    self.lock.release()

    if self.node=="A":
      ostr = "Node A sending pkt: %d \nNode A sending ack: %d" % (r[0], r[1])
      if self.infile is not None:
        self.infile.write(ostr)
    elif self.node=="B":
      ostr = "\t\t\t\t\t\tNode B sending pkt: %d \t\t\t\t\t\tNode B sending ack: %d\n" % (r[0], r[1])
      if self.infile is not None:
        self.infile.write(ostr)
    #print ostr

    return r

  def update(self,rec_seq,rec_ack):
    '''Ret: bool isNewPkt'''

    #isNewPkt = False
    #if rec_seq == self.ack:           
    #  self.ack = (rec_seq+1)%NSEQ
    #  isNewPkt = True
    # elif rec_seq > self.ack:
    #  self.ack = (rec_seq+1)%NSEQ 
    #  isNewPkt = True

    # self.ack is used in both retrieve and update, so we need lock    
    self.lock.acquire()
    # Modification: In order to avoid missed ACK affecting the RTT computations
    # self.ack=rec_seq%NSEQ
    self.ack_queue.append(rec_seq)
    self.lock.release()
    
    # assumption: you will never receive an old ACK (FIFO Queue)
    isNewPkt = True   
    return isNewPkt

class NULL_ARQ:
  def __init__(self,node,infile=None):
    self.pre_recv_ack = 0
    self.seq = 0
    self.ack = 0
    self.n = node
    self.f = infile
    self.lock = threading.Lock()

  def retrieve(self):
    self.lock.acquire()
    r = (self.seq,self.ack)

    if self.n=="A":
      ostr = "Node A sending pkt: %d \nNode A sending ack: %d" % (self.seq, self.ack)
      if self.f is not None:
        self.f.write(ostr)
    elif self.n=="B":
      ostr = "\t\t\t\t\t\t\t\tNode B sending pkt: %d \n\t\t\t\t\t\t\t\tNode B sending ack: %d" % (self.seq, self.ack)
      if self.f is not None:
        self.f.write(ostr)

    self.seq = (self.seq+1)%NSEQ
    self.lock.release()
    return r

  def update(self,rec_seq,rec_ack):
    if self.n=="A":
      ostr = "Node A receive pkt: %d \nNode A receive ack: %d" % (rec_seq, rec_ack)
      if self.f is not None:
        self.f.write(ostr)
    elif self.n=="B":
      ostr = "\t\t\t\t\t\t\t\tNode B receive pkt: %d\n\t\t\t\t\t\t\t\tNode B receive ack: %d" % (rec_seq, rec_ack)
      if self.f is not None:
        self.f.write(ostr)
    return True

# ////////////////////////////////////////////////////// #
#                    MAIN FUNCTION                       #
# ////////////////////////////////////////////////////// #
import os, glob
def ReadImage(imagePath):
  imgloc = imagePath
  #open the img
  img = Image.open(imgloc)
  mode = img.mode
  if mode == 'RGB':
    greyLevel = 3
  elif mode == 'L':
    greyLevel = 1
  img_data = numpy.array(list(img.getdata()), numpy.uint8)
  img_data.shape = len(img_data)*greyLevel,1

  width,height = img.size
  width1 = width / 128
  width2 = width % 128
  height1 = height / 128
  height2 = height % 128

  m = [width1,width2,height1,height2,greyLevel]#1 indicates grey-level

  for i in range(len(img_data)):
    m.append(img_data[i][0])
  return m

def string_to_ord_list(s):
  return map(lambda x: ord(x), s)

if __name__ == '__main__':
  from optparse import OptionParser
  parser = OptionParser()
  parser.add_option("", "--play", type="int", default=1, help="Mode: txt or GUI")
  parser.add_option("-m", "--arq-mode", type="int", default=4, help="ARQ mode")
  parser.add_option("-t", "--times", type="int", default=100, help="time slots")
  parser.add_option("-w", "--windowsize", type="int", default=8, help="window size")  
  parser.add_option("-d", "--downlink", type="float", default=1, help="downlink PRR")
  parser.add_option("-u", "--uplink", type="float", default=1, help="uplink PRR")
  (options, args) = parser.parse_args()

  play = options.play
  assert(play == 1 or play == 2)

  mode = options.arq_mode
  run_times = options.times
  uplink = options.uplink
  downlink = options.downlink
  windowsize = options.windowsize

  framesize = 1535 #757 #404  #74
  pktsize = framesize - 20 - 4

  SEQ_NUM = 256

  if play == 1:
    data_raw1 = []
    data_raw2 = []
    for i in range(run_times):
      x = [i] * pktsize
      for j in range(pktsize):
        data_raw1.append((i+j)%SEQ_NUM)
        data_raw2.append((i+2*j)%SEQ_NUM)
  elif play == 2:
    data_raw1 = ReadImage('img_data/INC.bmp')
    data_raw2 = ReadImage('img_data/CUHK.bmp')

  seed_num = random.randint(0, sys.maxint) # 9000541137569755104
  print "LEN: ", seed_num, len(data_raw1), len(data_raw2)
  max_pic_len = max(len(data_raw1), len(data_raw2))

  import os, sys
  f = None
  #f = sys.stdout
  #f = open('SR_ARQ.dat','w')

  node11 = BufferController(data_raw1,'A',framesize,windowsize,mode,True,f)
  node21 = BufferController(data_raw2,'B',framesize,windowsize,mode,True,f)

  random.seed(seed_num)
  for i in range(run_times):

    if play == 2 and len(node11.recstring) >= max_pic_len \
                 and len(node21.recstring) >= max_pic_len:
      pass #break

    # beacon mode
    beacon_seq = i%NSEQ
    f11 = node11.extractTxFrame(beacon_seq)
    f21 = node21.extractTxFrame(beacon_seq)
    
    # burst mode
    #f11 = node11.extractTxFrame(1,beacon_seq)
    #f21 = node21.extractTxFrame(1,beacon_seq)

    #print "A: ", i, ": ", string_to_hex_list(f11[:10]), string_to_hex_list(f11[-4:])
    #print "B: ", i, ": ", string_to_hex_list(f21[:10]), string_to_hex_list(f21[-4:])

    if PRINT_BUFFER_FLAG:
      print "INFO: ", i, len(node11.recstring), len(node21.recstring), "f21=", string_to_ord_list(f21[-10:]), "f11=", string_to_ord_list(f11[-10:])

    # uplink phase
    if random.randint(1,100) <= uplink*100:
      fxor = ''
      for xx in range(len(f11)):
        fxor += chr(ord(f11[xx])^ord(f21[xx]))
      #print "R: ", i, ": ", len(fxor),string_to_hex_list(fxor[0:10])#, string_to_hex_list(fxor[-14:])

      rx_pkt = mac_frame_handler.HandleXorFrame(fxor,512)
      from gnuradio import digital
      (ok, payload) = digital.crc.check_crc32(rx_pkt)
      assert(ok)

      # downlink phase of node A
      if random.randint(1,100) <= downlink*100:
        self_seq, = struct.unpack('!B',fxor[3])
        #print "A:", self_seq, beacon_seq
        assert(self_seq == beacon_seq)
        self_pkt = node11.getTxFrame(self_seq)
        #self_pkt = node11.getTxFrame(beacon_seq)
        rxframe = ''
        for xx in range(len(fxor)):
          rxframe += chr(ord(fxor[xx])^ord(self_pkt[xx]))
        #print "Node A: ", (rxframe == f21)
        #print "f21: ", len(f21), string_to_ord_list(f21)

        rxpkt = node11.updateRxQueue(rxframe)
        if rxpkt is not None:
          #print string_to_ord_list(rxpkt)        
          node11.deliverRxFrames(rxpkt)

        if play == 1 or play == 2:
          rs = node11.recstring
          clen = min(len(rs),len(data_raw2))
          rs = rs[:clen]
          d = data_raw2[:clen]

          flag = (string_to_ord_list(rs[-pktsize:]) == d[-pktsize:])
          if not flag and mode != 1:
            print "===================================================================="
            print "ERROR", i, len(rs), len(data_raw2), "f21=", string_to_ord_list(f21[-10:]), "f11=", string_to_ord_list(f11[-10:])
            print "Node B R:", string_to_ord_list(rs[-(2*pktsize):])
            print "Node B T:", d[-(2*pktsize):]
            break

      # downlink phase of node B
      if random.randint(1,100) <= downlink*100:  
        self_seq, = struct.unpack('!B',fxor[5])
        #print "B:", self_seq, beacon_seq
        assert(self_seq == beacon_seq)
        self_pkt = node21.getTxFrame(self_seq)
        #self_pkt = node21.getTxFrame(beacon_seq)
        rxframe = ''
        for xx in range(len(fxor)):
          rxframe += chr(ord(fxor[xx])^ord(self_pkt[xx]))
        #print "Node B: ", (rxframe == f11)

        rxpkt = node21.updateRxQueue(rxframe)
        if rxpkt is not None:
          node21.deliverRxFrames(rxpkt)

        if play == 1 or play == 2:
          rs = node21.recstring
          clen = min(len(rs),len(data_raw1))
          rs = rs[:clen]
          d = data_raw1[:clen]

          flag = (string_to_ord_list(rs[-pktsize:]) == d[-pktsize:])
          if not flag and mode != NULL_MODE:
            print "===================================================================="
            print "ERROR", i, len(rs), len(data_raw1), "f21=", string_to_ord_list(f21[-10:]), "f11=", string_to_ord_list(f11[-10:])
            print "Node A R:", string_to_ord_list(rs[-(2*pktsize):])
            print "Node A T:", d[-(2*pktsize):]
            break

  if f is not None and f is not sys.stdout:
    f.close()

  print len(node11.recstring)/pktsize, len(node21.recstring)/pktsize
