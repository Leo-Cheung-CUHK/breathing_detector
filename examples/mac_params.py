#
# Copyright 2014 Lizhao You
#
# Shared parameters for MAC protocol
#

import operator, struct, Queue, time

import gnuradio.gr.gr_threading as _threading

# /////////////////////////////////////////////////////////////////////////////
#                             Print Queue Monitor
# /////////////////////////////////////////////////////////////////////////////   
class MyQueue():
  def __init__(self):
    self.queue = Queue.Queue()

  def put(self,istr):
    #print my_time(), istr
    #ostr = str(my_time())+": "+istr
    ostr = "%.06f: " %(time.time()) + istr
    self.queue.put(ostr)

class print_queue_watcher_thread(_threading.Thread):
  def __init__(self, q):
    _threading.Thread.__init__(self)
    self.setDaemon(1)
    self.q = q
    self.start()

  def run(self):
    while True:
      msg = self.q.get()
      if self.q.qsize() > 10240:
        pass
      else:
        print msg

# /////////////////////////////////////////////////////////////////////////////
#                              Frame Format
# /////////////////////////////////////////////////////////////////////////////
# Frame: PHY HEADER (PREAMBLE) | MAC HEADER | DATA | CRC32
# Note: we do not have SIGNAL symbol as in 802.11
MAC_HEADER_LEN = 20    # The maximum MAC header length, in consist with ARQ.py
CRC_LEN = 4
TOTAL_ARQ_NSEQ = 128
TOTAL_MAC_NSEQ = 256

global MAC_AVAILABLE_LEN
MAC_AVAILABLE_LEN = 0

# MAC Header Format (13 bytes used, 7 bytes available)
#   [18,19]: pktlen, useful for the tunnel mode
#   [11]: If SACK is enable, then [11] indicates the length of SACK sequences
# Beacon: TYPE | BEACON# | BURST# |      0    |      0    |           |           |        |        |        |        | PKT_LEN |     0    | ... |
# XOR   : TYPE |    0    |    0   | A_BEACON# | A_BURST#  | B_BEACON# |  B_BURST# | A_SEQ# | A_ACK# | B_SEQ# | B_ACK# | PKT_LEN |     0    | ... |
# User A: TYPE | BEACON# | BURST# | A_BEACON# | A_BURST#  |           |           | A_SEQ# | A_ACK# |        |        | PKT_LEN | SACK_LEN | ... |
# User B: TYPE | BEACON# | BURST# |           |           | B_BEACON# |  B_BURST# |        |        | B_SEQ# | B_ACK# | PKT_LEN | SACK_LEN | ... |
#
# Note: 
#   TYPE: "B", "X", "E", "E"
#   TYPE/*#/*_ACK (1 byte)
#   PKT_LEN (2 bytes)
#   SACK_LEN (1 byte)

# denote packet type
class packet_type:
  NULL, BEACON, ENDNODE, XOR = range(4)

class FRAME_FORMAT(object):
  def __init__(self,node=0):
    self.TYPE_POS = 0
    self.BEACON_POS = 1
    self.BURSTS_POS = 2
    self.PKTLEN_POS = 11    # 2 bytes
    self.SACK_ST_POS = 13   # 1 byte
    global MAC_AVAILABLE_LEN
    MAC_AVAILABLE_LEN = MAC_HEADER_LEN-(self.SACK_ST_POS+1)

class ENDNODE_FRAME_FORMAT(FRAME_FORMAT):
  def __init__(self,node=None):
    FRAME_FORMAT.__init__(self)
    assert(node == 'A' or node == 'B')

    # For XOR packet only
    if node == 'A':
      self.SELF_BEACON_POS = 3
      self.SELF_BURSTS_POS = 4
      self.SELF_SEQ_POS = 7
      self.SELF_ACK_POS = 8
      self.RECV_SEQ_POS = 9
      self.RECV_ACK_POS = 10

    elif node == 'B':
      self.SELF_BEACON_POS = 5
      self.SELF_BURSTS_POS = 6
      self.RECV_SEQ_POS = 7
      self.RECV_ACK_POS = 8
      self.SELF_SEQ_POS = 9
      self.SELF_ACK_POS = 10



class RELAY_RX_FRAME_FORMAT(FRAME_FORMAT):
  def __init__(self):
    FRAME_FORMAT.__init__(self)
    self.A_SEQ_POS = 7
    self.A_ACK_POS = 8
    self.B_SEQ_POS = 9
    self.B_ACK_POS = 10

# /////////////////////////////////////////////////////////////////////////////
#                                Shared Config
# /////////////////////////////////////////////////////////////////////////////
def make_binary_data(count, rseed):
  import random, cmath, math, numpy, struct
  # generate random data
  random.seed(rseed)
  data = ''
  for i in range(count):
    data += struct.pack('!B',random.randint(0, 255))
  return data

def make_modulated_samples(data_tones, num_symbols, filename=None):
  import random, cmath, math, numpy
  # load data from options.txdata
  # otherwise generate random data
  # return a new source block
  count = num_symbols*data_tones
  if filename is not None:
    data = numpy.fromfile(filename, dtype=numpy.complex64, count=count, sep='')
    data = [complex(d) for d in data]
  else:
    random.seed(78532)
    #data = [cmath.exp(1j * 2*math.pi * random.random()) for i in range(count)]
    NN = 4
    sf = 1 #math.sqrt(0.5)
    qpsk = lambda : sf * cmath.exp(1j * (math.pi/NN * (2*random.randint(0, NN-1)+1)))
    off = NN/2-0.5
    qam = lambda : sf/off * ((random.randint(0, NN-1) - off) + 1j*(random.randint(0, NN-1) - off))

    data = [qpsk() for i in range(count)]
  src = gr.vector_source_c(data, repeat=True, vlen=data_tones)
  return src

# for short string print
def string_to_hex_list_short(s):
  return ' '.join(map(lambda x: hex(ord(x)), s))

def string_to_ord_list(s):
  return map(lambda x: ord(x), s)

def string_to_hex_list(s):
  return map(lambda x: hex(ord(x)), s)
    
def my_time(in_time=None):
  if in_time is None:
    r = int(round(time.time()*1000000)) % 10**8
  else:
    r = int(round(in_time*1000000)) % 10**8
  return int(r)

def string_xor(string_a, string_b):
  return ''.join( map(lambda a, b: chr(ord(a)^ord(b)),  string_a,  string_b) )
