#
# Copyright 2014 Lizhao You
#
# Functions for parsing MAC frame
#

import time, operator, struct, Queue, math
from gnuradio import digital
from mac_params import FRAME_FORMAT, ENDNODE_FRAME_FORMAT, RELAY_RX_FRAME_FORMAT
from mac_params import string_to_hex_list, string_xor
from mac_params import MAC_HEADER_LEN, CRC_LEN

user_type_dict = {0:'user_none', 1:'user_mode', 2:'user_mode_A', 3:'user_mode_B', 4:'user_mode_X'}
# NULL(Beacon); Node A; Node B; Xor;
node_type_dict = {'N':0, 'A':1, 'B':2, 'X':3} 

# /////////////////////////////////////////////////////////////////////////////
# FIXME: For MAC_HEADER_LEN >= 12 (for new version, ARQ in MAC layer)
# /////////////////////////////////////////////////////////////////////////////
def EndNodeParsePHYMsg(msg,frame_len,node):
  cfo = msg.cfo_values()
  snr = msg.get_snr_values()
  ts = msg.timestamp_sec() + msg.timestamp_frac_sec() #if msg.timestamp_valid() else None
  pc_ts = msg.pctime_sec() + msg.pctime_frac_sec()
  nsamples = 0
  power = msg.power_list()
  decode_itv = msg.get_decode_time()

  # get payload
  rx_pkt = msg.to_string()
  assert(len(rx_pkt) == frame_len)

  #print "EndNodeParsePHYMsg  rx_pkt=%s" %(string_to_hex_list(rx_pkt[0:16]))

  (ok, payload) = digital.crc.check_crc32(rx_pkt)
  p = EndnodeFrame(frame_len,node)
  p.set_meta_info(cfo,snr,ts,power,nsamples,pc_ts)
  p.set_frame_info(ok,payload,rx_pkt[-4:])
  p.set_mac_info(payload)
  p.set_decode_itv(decode_itv)
  return p

def RelayParsePHYMsg(msg,frame_len,sym_len=0):
  cfo = msg.cfo_values()
  snr = msg.get_snr_values()
  ts = msg.timestamp_sec() + msg.timestamp_frac_sec() #if msg.timestamp_valid() else None
  pc_ts = msg.pctime_sec() + msg.pctime_frac_sec()

  nsamples = 0
  power = msg.power_list()

  # get payload
  s = msg.to_string()
  user_type = msg.get_user_type()

  ret = []
  #for j in range(len(s)/frame_len):
  #  rr1 = frame_len*j
  #  rr2 = frame_len*(j+1)
  #  if j == 0:
  #    rx_pkt = HandleXorFrame(s[rr1:rr2],sym_len)
  #  else:
  #    rx_pkt = s[rr1:rr2]

  for j in range(len(s)/frame_len):
    rr1 = frame_len*j
    rr2 = frame_len*(j+1)

    if user_type_dict[user_type]=='user_mode_X':
      rx_pkt = HandleXorFrame(s[rr1:rr2],sym_len)
    else:
      rx_pkt = s[rr1:rr2]
    #print "NO CRCCOMPENSATE len(s)=%d user_type=%d %s" %(len(s), user_type, string_to_hex_list(s[0:16]))
    #x = digital.crc.gen_and_append_crc32(rx_pkt[:-4])
    #print "STRING HEX LIST", len(x), string_to_hex_list(x[-4:])

    (ok, payload) = digital.crc.check_crc32(rx_pkt)

    p = RelayFrame(frame_len)
    p.set_meta_info(cfo,snr,ts,power,nsamples,pc_ts, user_type)
    p.set_frame_info(ok,payload,rx_pkt[-4:])
    p.set_mac_info(payload)
    ret.append(p)
    # only one packet
    break
  return ret

# private function, used within this file
def HandleXorFrame(message,sym_len):
  poly = {16:struct.pack('!4B',0x8c,0xd5,0x91,0x1f),
          32:struct.pack('!4B',0x59,0x0c,0xcd,0xb6),
          64:struct.pack('!4B',0xa6,0x2a,0x4e,0xba),
          128:struct.pack('!4B',0x6e,0xaf,0x13,0x3a),
          256:struct.pack('!4B',0x52,0xfa,0x50,0xe2),
          512:struct.pack('!4B',0xea,0xb0,0xe2,0x44)}

  #x = string_to_hex_list(poly[sym_len])
  #print "poly=", x, len(x)
  #x = string_to_hex_list(message[-4:])
  #print "pkt =", x, len(x)
  #print " real = ", string_to_hex_list(real_crc), len(real_crc)

  real_crc = string_xor(message[-4:], poly[sym_len])
  message=message[0:-4]+real_crc
  return message

# /////////////////////////////////////////////////////////////////////////////
#                               Frame Class
# /////////////////////////////////////////////////////////////////////////////
class Frame(object):
  def __init__(self,framesize):
    """
    @ret: received frame with all information
      * meta-information: ts, snr, cfo, power
      * frame info: ok, payload, crc32
      * mac header info: packet_type, beacon_no
        - For node A/B: + xor_beacon_no, self_seq_no
    """
    self.ts    = None                   # FPGA time
    self.pc_ts = None                   # PC time
    self.snr   = None                   # SNR
    self.cfo   = None                   # CFO
    self.power = None                   # absoluate power
    self.nsamples = None
    self.decode_itv = None;

    self.ok = None                      # crc pass or fail    
    self.payload = None                 # payload
    self.crc32 = None                   # crc

    self.type  = None                   # Frame type

    self.framesize  = framesize
    self.headersize = MAC_HEADER_LEN
    self.crcsize    = CRC_LEN
    self.pktsize    = framesize - self.headersize - self.crcsize

    self.f = FRAME_FORMAT()

  def set_header_size(self, size):
    # provide API to modify frame format
    self.headersize = size
    self.pktsize    = self.framesize - self.headersize - self.crcsize
    print "@@@@@@@@@ Reset header size successfully headersize=%d pktsize=%d" % (self.headersize, self.pktsize)

  def set_meta_info(self,cfo,snr,ts,power,nsamples,pc_ts=None):
    # snr format: list with snr[0] (time domain)
    # the results should be the same as power_list
    self.cfo = cfo
    self.power = power
    self.ts = ts
    self.pc_ts = pc_ts
    self.nsamples = nsamples
    self.snr = snr

  def set_mac_info(self,rxframe):
    f = self.f
    (type_byte,) = struct.unpack('!B', rxframe[f.TYPE_POS])
    self.type =  type_byte & 0xFC
    (self.beacon_no,) = struct.unpack('!B', rxframe[f.BEACON_POS])
    (self.bursts_no,) = struct.unpack('!B', rxframe[f.BURSTS_POS])

  def set_frame_info(self,ok,payload,crc32):
    self.ok = ok
    self.payload = payload
    self.crc32 = crc32

class EndnodeFrame(Frame):
  """
    @param node: node type
  """
  def __init__(self,frame_len,node):
    Frame.__init__(self,frame_len)
    self.node = node
    self.f = ENDNODE_FRAME_FORMAT(self.node)

  def set_mac_info(self,rxframe):
    f = self.f
    self.user_type = struct.unpack('!B', rxframe[f.TYPE_POS])[0] & 0x3
    #print "EndnodeFrame TYPE: 0x%02x %d" %(struct.unpack('!B', rxframe[f.TYPE_POS])[0], self.user_type)
    
    Frame.set_mac_info(self,rxframe)

    #print self.type
    if self.type == ord('B'):  # Beacon packet
      # TODO: add more field
      pass
    elif self.type == ord('X') or self.type== ord('S'):  # Xor packet
      (self.self_seq_no,) = struct.unpack('!B', rxframe[f.SELF_SEQ_POS])
      (self.x_beacon_no,) = struct.unpack('!B', rxframe[f.SELF_BEACON_POS])
      (self.x_bursts_no,) = struct.unpack('!B', rxframe[f.SELF_BURSTS_POS])  
      (self.rec_seq,) = struct.unpack('!B',rxframe[f.RECV_SEQ_POS])  
      (self.rec_ack,) = struct.unpack('!B',rxframe[f.RECV_ACK_POS])
      (self.s_seq,) = struct.unpack('!B',rxframe[f.SELF_SEQ_POS])  
      (self.s_ack,) = struct.unpack('!B',rxframe[f.SELF_ACK_POS])
    #print "#############", self.type, self.beacon_no, self.self_seq_no

  def set_meta_info(self,cfo,snr,ts,power,nsamples,pc_ts=None):
    Frame.set_meta_info(self,cfo,snr,ts,power,nsamples,pc_ts)
    self.snr = snr[0]
    self.cfo = cfo[0]

  def set_decode_itv(self, decode_itv):
    self.decode_itv = decode_itv;

  def generate_frame_data(self,beacon_seq,txseq,ackseq,payload,burst_seq=None,sack=None,pktlen=0):
    f = self.f

    buff = [struct.pack('!B',0)] * self.headersize
    buff[f.TYPE_POS] = struct.pack('!B',ord('E'))
    buff[f.SELF_BEACON_POS] = struct.pack('!B',beacon_seq)
    buff[f.SELF_SEQ_POS] = struct.pack('!B',txseq)
    buff[f.SELF_ACK_POS] = struct.pack('!B',ackseq)
    buff[f.PKTLEN_POS:f.PKTLEN_POS+2] = struct.pack('!H',pktlen)

    if burst_seq is not None:
      buff[f.SELF_BURSTS_POS] = struct.pack('!B',burst_seq)

    if sack is not None:
      l = len(sack)
      from mac_params import MAC_AVAILABLE_LEN
      #print "[MAC_FRAME_HANDLER]", l, self.headersize, f.SACK_ST_POS, MAC_AVAILABLE_LEN
      assert(l <= MAC_AVAILABLE_LEN)
      buff[f.SACK_ST_POS] = struct.pack('!B',l)
      for i in range(l):
        buff[f.SACK_ST_POS+i+1] = struct.pack('!B',sack[i])

    if type(payload) == list:
      buff = buff + payload
      string = "".join(buff)
    if type(payload) == str:
      string = "".join(buff)+payload

    string = digital.crc.gen_and_append_crc32(string)
    #print "generate_frame_data, %s" %(string_to_hex_list(string[0:64]))
    return string

class RelayFrame(Frame):
  def __init__(self,frame_len):
    Frame.__init__(self,frame_len)

  def set_mac_info(self,rxframe):
    Frame.set_mac_info(self,rxframe)
    # TODO: add more filed

  def set_meta_info(self,cfo,snr,ts,power,nsamples,pc_ts=None, user_type=0):
    Frame.set_meta_info(self,cfo,snr,ts,power,nsamples,pc_ts)
    #assert(len(cfo)==2)  # relay must have two CFOs, but sometimes only SU is detected
    assert(len(snr)==2)  # relay must have two SNRs. If only SU is detected, one must be snf

    self.snr_a = snr[0] if len(snr)>0 else 0
    self.snr_b = snr[1] if len(snr)>1 else 0
    
    # FIXME: to determine SU CFO be A or B
    self.cfo_a = cfo[0] if len(cfo)>0 else 0
    self.cfo_b = cfo[1] if len(cfo)>1 else 0
    
    # use pilots to calculate SNR
    try:
      self.snr_a1 = 10*math.log10(power[0]/power[1]) if len(power) >= 2 else 0
      self.snr_b1 = 10*math.log10(power[2]/power[3]) if len(power) >= 4 else 0
    except (ValueError, ZeroDivisionError):
      self.snr_a1 = self.snr_b1 = 0
      print " ValueError: ", power[0], power[1], power[2], power[3]

    self.user_type = user_type

    #print self.snr_a, self.snr_a1, self.snr_b, self.snr_b1
    self.snr = (self.snr_a+self.snr_b)/2

# /////////////////////////////////////////////////////////////////////////////
#                              BER Calculation
# /////////////////////////////////////////////////////////////////////////////
class calc_user_ber:
  # PNC Bits parameters
  #   self.txA    A's raw bits
  #   self.txB    B's raw bits
  
  def listXor(self, A, B):
    return map(operator.xor, A, B)
  
  def countBER(self, tx, rx):
    try:
      assert(len(tx)==len(rx))
    except AssertionError:
      print len(tx), len(rx)

    v = 0.0
    for i in range(len(tx)):
      a = tx[i] ^ rx[i]
      t = bin(a).count("1")
    #print a, t
      v += t
    #print v, len(tx)
    return v/(len(tx)*8)
  
  def __init__(self, A, B):
    # Note: A/B is a list
    # element is ord(unsigned char)
    # e.g. A = [65, 65, 65, ...]
    self.txA = A
    self.txB = B
    self.txX = self.listXor(self.txA, self.txB)

  def xBER(self, rx):
    if type(rx) == str:
      rx = map(ord,rx)
    berX = self.countBER(self.txX, rx) if rx is not None else None
    return berX

  def aBER(self, rx):
    if type(rx) == str:
      rx = map(ord,rx)
    berA = self.countBER(self.txA, rx) if rx is not None else None
    return berA

  def bBER(self, rx):
    if type(rx) == str:
      rx = map(ord,rx)
    berB = self.countBER(self.txB, rx) if rx is not None else None
    return berB

  def BER(self, rxX=None, rxA=None, rxB=None):
    berX = self.countBER(self.txX, rxX) if rxX is not None else None
    berA = self.countBER(self.txA, rxA) if rxA is not None else None
    berB = self.countBER(self.txB, rxB) if rxB is not None else None
    return [berX, berA, berB]
  
  def strBER(self, userStr):
    # Note: userStr is a string, containing three users
    bits = map(ord, userStr)
    size = len(bits)/3
    r = self.BER(bits[0:size], bits[size:2*size], bits[2*size:])
    return r
