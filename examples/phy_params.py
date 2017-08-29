#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
# Copyright 2014 Lizhao You
#

import math, random, cmath

# /////////////////////////////////////////////////////////////////////////////
#                 Shared configuration
# /////////////////////////////////////////////////////////////////////////////

PNC_TYPE = 0
global ieee_short_preamble ,ieee_long_preamble
ieee_short_preamble = [

      0.3680 + 0.3680j,
     -1.0596 + 0.0187j,
     -0.1078 - 0.6282j,
      1.1421 - 0.1012j,
      0.7360         ,
      1.1421 - 0.1012j,
     -0.1078 - 0.6282j,
     -1.0596 + 0.0187j,
      0.3680 + 0.3680j,
      0.0187 - 1.0596j,
     -0.6282 - 0.1078j,
     -0.1012 + 1.1421j,
           0 + 0.7360j,
     -0.1012 + 1.1421j,
     -0.6282 - 0.1078j,
      0.0187 - 1.0596j,
      0.3680 + 0.3680j,
     -1.0596 + 0.0187j,
     -0.1078 - 0.6282j,
      1.1421 - 0.1012j,
      0.7360         ,
      1.1421 - 0.1012j,
     -0.1078 - 0.6282j,
     -1.0596 + 0.0187j,
      0.3680 + 0.3680j,
      0.0187 - 1.0596j,
     -0.6282 - 0.1078j,
     -0.1012 + 1.1421j,
           0 + 0.7360j,
     -0.1012 + 1.1421j,
     -0.6282 - 0.1078j,
      0.0187 - 1.0596j,
      0.3680 + 0.3680j,
     -1.0596 + 0.0187j,
     -0.1078 - 0.6282j,
      1.1421 - 0.1012j,
      0.7360         ,
      1.1421 - 0.1012j,
     -0.1078 - 0.6282j,
     -1.0596 + 0.0187j,
      0.3680 + 0.3680j,
      0.0187 - 1.0596j,
     -0.6282 - 0.1078j,
     -0.1012 + 1.1421j,
           0 + 0.7360j,
     -0.1012 + 1.1421j,
     -0.6282 - 0.1078j,
      0.0187 - 1.0596j,
      0.3680 + 0.3680j,
     -1.0596 + 0.0187j,
     -0.1078 - 0.6282j,
      1.1421 - 0.1012j,
      0.7360          ,
      1.1421 - 0.1012j,
     -0.1078 - 0.6282j,
     -1.0596 + 0.0187j,
      0.3680 + 0.3680j,
      0.0187 - 1.0596j,
     -0.6282 - 0.1078j,
     -0.1012 + 1.1421j,
           0 + 0.7360j,
     -0.1012 + 1.1421j,
     -0.6282 - 0.1078j,
      0.0187 - 1.0596j]

ieee_long_preamble =[

     1.2500         ,
     -0.0410 - 0.9626j,
      0.3180 - 0.8893j,
      0.7747 + 0.6624j,
      0.1689 + 0.2231j,
      0.4786 - 0.7017j,
     -0.9210 - 0.4414j,
     -0.3065 - 0.8494j,
      0.7803 - 0.2071j,
      0.4267 + 0.0326j,
      0.0079 - 0.9200j,
     -1.0944 - 0.3790j,
      0.1958 - 0.4683j,
      0.4694 - 0.1195j,
     -0.1799 + 1.2853j,
      0.9539 - 0.0328j,
      0.5000 - 0.5000j,
      0.2953 + 0.7868j,
     -0.4577 + 0.3144j,
     -1.0501 + 0.5218j,
      0.6577 + 0.7389j,
      0.5565 + 0.1130j,
     -0.4825 + 0.6503j,
     -0.4516 - 0.1744j,
     -0.2803 - 1.2071j,
     -0.9751 - 0.1325j,
     -1.0186 - 0.1640j,
      0.6006 - 0.5923j,
     -0.0224 + 0.4302j,
     -0.7351 + 0.9210j,
      0.7337 + 0.8470j,
      0.0983 + 0.7808j,
     -1.2500         ,
      0.0983 - 0.7808j,
      0.7337 - 0.8470j,
     -0.7351 - 0.9210j,
     -0.0224 - 0.4302j,
      0.6006 + 0.5923j,
     -1.0186 + 0.1640j,
     -0.9751 + 0.1325j,
     -0.2803 + 1.2071j,
     -0.4516 + 0.1744j,
     -0.4825 - 0.6503j,
      0.5565 - 0.1130j,
      0.6577 - 0.7389j,
     -1.0501 - 0.5218j,
     -0.4577 - 0.3144j,
      0.2953 - 0.7868j,
      0.5000 + 0.5000j,
      0.9539 + 0.0328j,
     -0.1799 - 1.2853j,
      0.4694 + 0.1195j,
      0.1958 + 0.4683j,
     -1.0944 + 0.3790j,
      0.0079 + 0.9200j,
      0.4267 - 0.0326j,
      0.7803 + 0.2071j,
     -0.3065 + 0.8494j,
     -0.9210 + 0.4414j,
      0.4786 + 0.7017j,
      0.1689 - 0.2231j,
      0.7747 - 0.6624j,
      0.3180 + 0.8893j,
     -0.0410 + 0.9626j]

class ofdm_params:
  # General OFDM parameters:
  #   fft_length        size of FFT
  #   occupied_tones    how many tones in the FFT are occupied (excluding DC)
  #                     (the margin is used for shaping and coarse freq. tracking)
  #   cp_length         cyclic prefix
  #
  # FFT diagram:
  #   |< pad >|< occupied/2 >|DC|< occupied/2 >|< pad-1 >|
  #                          |^fft_length/2 == DC (even)

  def pad_symbol(self, symbol):
    padded = self.fft_length*[0,]
    padded[self.pad : self.pad + self.occupied_tones + 1] = symbol
    return padded

  def random_symbol(self):
    # NOTE: pilot bins have power 1 (see mapper/demapper)
    # we make our preamble have (complex) power 1 per bin as well x scaling factor
    sf = 1 # FIXME: allow non-unity scaling factors
    psk = lambda : cmath.exp(1j * 2 * math.pi * random.random())
    symbol = [ sf * psk() for i in range(self.occupied_tones)]
    # fill in the null DC
    half = self.occupied_tones/2
    return symbol[:half] + [0] + symbol[half:]

  def make_even(self, symbol):
    # NOTE: we're using only even frequencies, so give them more power
    # so that the overall symbol power is the same.
    # raw_ofdm_frame_acquisition::correlate() uses diff phase only so it's ok
    sf = math.sqrt(2)
    for i in range(len(symbol)):
      if((self.pad + i) & 1): # zero-out every odd frequency
        symbol[i] = 0
      else: # boost power in every even frequency
        symbol[i] *= sf

  def construct_pnc_preamble(self, options):
    global ieee_short_preamble ,ieee_long_preamble
    self.num_preambles = 2
    # make the preamble
    random.seed(9817)
    #random.seed()
    preambles = [ self.random_symbol() for i in range(self.num_preambles+1) ]
    #self.rawofdm_preambles = preambles  # lzyou: important backup

    self.half_sync = True #options.half_sync
    if self.half_sync:
      # the first one even for sync
      self.make_even(preambles[0])


    ##############################################################################################
    # I. Preambles in Frequency Domain (len=53)

    pre0 = [ 8.101298283888491 - 7.897401919497907j, 0,  5.113779739053749 -10.092039572053300j, 0, 
            -2.733628894732520 -10.978490909946768j, 0, 11.255291127186776 + 1.148229296203257j, 0, 
             9.840414085709714 + 5.582674007660262j, 0,  9.794007539749146 - 5.663693755865097j, 0,
            -0.255984192343444 -11.310811955302897j, 0,  8.368352668916184 + 7.613848574352970j, 0,
           -10.736286397831510 + 3.568212074604673j, 0, -3.649976019629170 -10.708766451845795j, 0,
            -7.423081084994919 - 8.538024592723445j, 0,  6.628051208587084 + 9.168911990128583j, 0,
           -11.277316472572981 + 0.906712076809686j, 0, -0.000000238418579 + 0.000000268220901j, 0,
             9.129050416945123 - 6.682846686388399j, 0, -2.266448671819444 +11.084368206326072j, 0,
            11.249599363921211 + 1.202709368347543j, 0,-11.066923335702919 - 2.350148273653639j, 0,
             1.405076543089520 -11.226120490622634j, 0,-11.226808909293240 + 1.399562566408559j, 0,
             6.855439713083078 - 9.000162496346590j, 0,  2.743000626564026 -10.976152628660202j, 0,
            11.265937974033719 + 1.038575314084831j, 0,  9.441888054907398 + 6.233037168076924j, 0,
            10.982598564236877 - 2.717082554382025j, 0, 10.250101570852925 + 4.789098671446473j, 0,
            11.309704683162504 - 0.300953154870231j]
    #{ YQ: normalize the power of preamble. 
    sf0 = 1/11.31371*1.4142*2     # increase the power of STS to benefit peak detection (ratio 2)
    pre0 = [p*sf0 for p in pre0]
    # A's Training Symbol: Freq, only keep data_tones
    # NOTE: random number: random.seed(9817) | second preabmle
    pre1 = [  0, 0, 0, 0, 0, 0,
             -6.2952 + 4.9366j,  2.4255 + 7.6235j,  5.7135 - 5.5996j, -7.5109 + 2.7543j,
             -7.6708 - 2.2714j,  3.0858 - 7.3809j,  7.4767 - 2.8459j, -7.8102 + 1.7322j, 
              5.6839 - 5.6297j,  1.7752 - 7.8006j, -7.4013 - 3.0366j,  4.3934 + 6.6856j, 
              7.5909 - 2.5254j, -7.6760 - 2.2537j, -0.0898 - 7.9995j, -7.6591 + 2.3105j, 
             -7.9456 + 0.9312j, -7.9993 - 0.1043j,  7.9965 - 0.2350j, -4.0937 - 6.8732j, 
             -7.3674 - 3.1179j, -7.8306 + 1.6375j, -4.7677 - 6.4241j,  7.1099 + 3.6673j, 
             -4.7978 - 6.4016j, -2.4849 + 7.6043j,  0.0000 + 0.0000j,  4.8656 - 6.3503j,
              7.0433 - 3.7937j,  7.9698 - 0.6945j, -5.3350 - 5.9614j, -2.3271 - 7.6540j,
              7.3225 - 3.2219j,  4.8572 + 6.3567j, -1.2351 + 7.9041j,  6.3683 + 4.8420j,
              7.0268 - 3.8241j,  3.8621 + 7.0060j,  4.6707 + 6.4950j,  6.2755 - 4.9617j,
             -5.5160 - 5.7943j,  3.7751 + 7.0533j, -6.6011 + 4.5195j,  0.3198 - 7.9936j,
             -7.9144 + 1.1671j, -7.9225 + 1.1105j,  7.7123 + 2.1263j, -0.9759 + 7.9402j,
             -3.9915 - 6.9331j, -5.5539 + 5.7580j, -0.5757 - 7.9793j, -6.1940 - 5.0630j,
              7.4395 - 2.9418j,  0, 0, 0, 0, 0]
    #{ YQ: normalize the power of preamble. 
    sf1 = 1/8.0*2                 # increase the power of STS to benefit peak detection (ratio 2)
    pre1 = [p*sf1 for p in pre1]
    #}
    # B's Training Symbol: Freq, only keep data_tones
    # NOTE: random number: random.seed(5618) | second preabmle
    pre2 = [  0, 0, 0, 0, 0, 0,
             -0.6316 + 7.9750j, -7.5356 + 2.6859j, -2.0688 - 7.7279j, -7.1000 - 3.6864j, 
              7.9505 + 0.8882j, -2.2716 - 7.6707j,  7.5805 - 2.5566j, -1.9501 - 7.7587j, 
              6.2525 + 4.9906j,  0.4932 - 7.9848j,  6.9758 + 3.9164j, -1.3729 + 7.8813j, 
             -6.3068 - 4.9218j,  7.6832 + 2.2290j, -7.4885 + 2.8146j, -7.6399 + 2.3731j, 
             -7.5195 - 2.7309j,  5.5274 + 5.7835j, -0.0081 - 8.0000j,  7.9864 + 0.4655j, 
             -1.7794 - 7.7996j,  1.9845 - 7.7500j,  7.3615 + 3.1317j,  7.4007 - 3.0379j, 
             -7.9723 + 0.6657j, -7.9997 - 0.0731j,  0.0000 - 0.0000j,  6.4739 + 4.6998j, 
              6.4446 + 4.7400j,  7.4081 - 3.0199j, -3.4665 + 7.2100j,  7.3704 - 3.1108j, 
              7.1683 + 3.5518j, -1.9296 - 7.7638j, -3.0715 - 7.3869j,  7.1743 - 3.5396j, 
              2.7845 - 7.4998j,  7.9896 + 0.4080j,  1.6896 + 7.8195j, -5.0648 - 6.1926j, 
              5.0249 - 6.2250j,  2.8257 + 7.4844j, -6.8207 - 4.1807j, -4.9442 - 6.2893j, 
              1.6790 + 7.8218j, -3.8523 - 7.0114j,  0.1515 + 7.9986j, -6.4149 - 4.7800j, 
              7.8433 + 1.5754j, -7.8026 - 1.7664j,  4.1055 - 6.8662j,  0.1742 + 7.9981j, 
              7.9469 - 0.9199j,   0, 0, 0, 0, 0]

    #{ YQ: normalize the power of preamble. 
    sf2 = 1/8.0*2                 # increase the power of STS to benefit peak detection (ratio 2)
    pre2 = [p*sf2 for p in pre2]

    # random.seed(1633)  first preamble/second preamble see gen_preamble.py
    lts_p1_fd = [ 0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,
                  0.000000+0.000000j,  0.000000+0.000000j, -0.964041-0.265753j, -0.311081-0.950383j,
                  0.694243-0.719741j,  0.748465-0.663174j, -0.667212-0.744868j, -0.315039+0.949079j,
                  0.763710+0.645560j, -0.999497-0.031711j, -0.626186-0.779674j,  0.523803-0.851839j,
                  0.997888 -0.064965j, 0.594347+0.804208j,  0.954984+0.296659j,  0.017153-0.999853j,
                 -0.740678+0.671861j, -0.447389+0.894339j,  0.516509+0.856282j,  0.143106+0.989707j,
                 -0.640159-0.768243j, -0.506641-0.862157j, -0.356178+0.934418j,  0.887594-0.460627j,
                  0.911697-0.410863j,  0.996237+0.086671j, -0.142711-0.989764j, -0.751052-0.660243j,
                  0.000000+0.000000j, -0.936169-0.351551j,  0.480509-0.876990j, -0.914081-0.405531j,
                 -0.923946+0.382522j,  0.992471+0.122480j, -0.879144-0.476556j,  0.935499+0.353331j,
                  0.576196-0.817312j,  0.460521+0.887649j, -0.908802-0.417228j,  0.122546-0.992463j,
                 -0.675412+0.737441j,  0.422751-0.906246j, -0.225820-0.974169j,  0.963864-0.266394j,
                 -0.709303+0.704904j,  0.975498+0.220007j, -0.247117+0.968986j, -0.507419-0.861700j,
                  0.803426+0.595405j,  0.954001-0.299803j, -0.999955+0.009460j,  0.999976-0.006933j,
                  0.157435-0.987529j, -0.493881+0.869530j, -0.126728-0.991937j,  0.000000+0.000000j,
                  0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,]
    lts_p1_fd = [p*8.0*1.414 for p in lts_p1_fd]

    lts_p2_fd = [ 0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,
                  0.000000+0.000000j,  0.000000+0.000000j, -0.086711-0.996234j,  0.573712-0.819057j,
                  0.427122+0.904194j, -0.050723-0.998713j,  0.932047+0.362337j, -0.535011-0.844845j,
                 -0.116044+0.993244j,  0.683903-0.729573j, -0.468873-0.883265j, -0.892572-0.450905j,
                  0.995458-0.095202j, -0.809641-0.586925j,  0.892427+0.451192j, -0.963464+0.267836j,
                 -0.998280-0.058630j, -0.296789+0.954943j,  0.431583+0.902073j, -0.646898+0.762577j,
                  0.801085-0.598551j,  0.525247-0.850950j, -0.278098+0.960553j,  0.967465-0.253004j,
                  0.878453-0.477829j,  0.992566-0.121709j,  0.964667+0.263473j,  0.498167-0.867081j,
                  0.000000+0.000000j,  0.993891+0.110363j,  0.092825+0.995682j,  0.957423+0.288690j,
                 -0.975912-0.218165j, -0.921485-0.388413j, -0.549797+0.835299j,  0.719382-0.694615j,
                 -0.965276-0.261232j,  0.655964+0.754792j,  0.744273-0.667875j,  0.312725+0.949844j,
                 -0.493058+0.869997j, -0.836143+0.548512j, -0.611285-0.791410j,  0.660599-0.750739j,
                  0.989159-0.146848j,  0.966920-0.255079j, -0.439609+0.898189j, -0.443485-0.896282j,
                  0.921337-0.388765j,  0.711330-0.702858j,  0.845603-0.533813j,  0.050340+0.998732j,
                 -0.099673-0.995020j, -0.861151+0.508349j,  0.970908-0.239453j,  0.000000+0.000000j,
                  0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,  0.000000+0.000000j,]
    lts_p2_fd = [p*8.0*1.414 for p in lts_p2_fd]

    # NOTE: OFDM Preambles for PNC and RawOFDM to estimate channel information
    if options.mode == 'PNC':
      # for PNC
      zeros = [0]*(self.occupied_tones+1);
      # add preamble for Relay
      preambles_tmp = [pre1, pre2]

      if options.node == "A":
        preambles_tmp = [pre1, zeros]
      elif options.node == "B":
        preambles_tmp = [zeros, pre2]
    else:
      # for RawOFDM
      preambles_tmp = [pre1, pre2]

    self.preambles = preambles_tmp

    ##############################################################################################
    # II. Preambles in Time Domain (len=64)

    rawofdm_sts_a = [
       1.2984 - 0.8548j,  -1.0196 + 0.3308j,  -0.1185 + 0.3173j,  0.5303 + 0.8118j,
      -0.1019 + 0.7091j,   0.2147 - 0.4824j,  -0.6892 + 0.3757j,  0.4383 + 0.0783j,
       0.4786 - 1.1558j,  -0.1643 - 0.4001j,   0.3664 + 0.6463j, -0.3522 + 0.4780j,
       0.2358 + 0.0465j,   0.4488 + 0.5154j,   0.0414 + 1.0010j,  0.1826 + 1.5399j,
      -0.1926 + 0.9062j,  -0.1387 - 0.8931j,  -0.1599 - 0.6938j,  0.1183 - 0.2761j,
       0.0605 - 0.3776j,   0.3285 + 0.2880j,   0.5988 - 0.1381j, -0.5900 - 1.3076j,
      -0.7603 - 1.0833j,  -0.7019 + 0.0782j,  -0.4639 - 0.0143j, -0.3260 - 0.3310j,
       0.5487 - 0.2704j,   1.6965 - 0.3181j,  -1.1422 + 0.5860j, -0.6653 - 0.1119j,
       1.2984 - 0.8548j,  -1.0196 + 0.3308j,  -0.1185 + 0.3173j,  0.5303 + 0.8118j,
      -0.1019 + 0.7091j,   0.2147 - 0.4824j,  -0.6892 + 0.3757j,  0.4383 + 0.0783j,
       0.4786 - 1.1558j,  -0.1643 - 0.4001j,   0.3664 + 0.6463j, -0.3522 + 0.4780j,
       0.2358 + 0.0465j,   0.4488 + 0.5154j,   0.0414 + 1.0010j,  0.1826 + 1.5399j,
      -0.1926 + 0.9062j,  -0.1387 - 0.8931j,  -0.1599 - 0.6938j,  0.1183 - 0.2761j,
       0.0605 - 0.3776j,   0.3285 + 0.2880j,   0.5988 - 0.1381j, -0.5900 - 1.3076j,
      -0.7603 - 1.0833j,  -0.7019 + 0.0782j,  -0.4639 - 0.0143j, -0.3260 - 0.3310j,
       0.5487 - 0.2704j,   1.6965 - 0.3181j,  -1.1422 + 0.5860j, -0.6653 - 0.1119j,]
    rawofdm_sts_a = [p*1.2 for p in rawofdm_sts_a]

    ieee80211a_sts = [ 
       0.0460 + 0.0460j,  -0.1324 + 0.0023j,  -0.0135 - 0.0785j,  0.1428 - 0.0127j,
       0.0920 + 0.0000j,   0.1428 - 0.0127j,  -0.0135 - 0.0785j, -0.1324 + 0.0023j,
       0.0460 + 0.0460j,   0.0023 - 0.1324j,  -0.0785 - 0.0135j, -0.0127 + 0.1428j,
       0.0000 + 0.0920j,  -0.0127 + 0.1428j,  -0.0785 - 0.0135j,  0.0023 - 0.1324j,
       0.0460 + 0.0460j,  -0.1324 + 0.0023j,  -0.0135 - 0.0785j,  0.1428 - 0.0127j,
       0.0920 + 0.0000j,   0.1428 - 0.0127j,  -0.0135 - 0.0785j, -0.1324 + 0.0023j,
       0.0460 + 0.0460j,   0.0023 - 0.1324j,  -0.0785 - 0.0135j, -0.0127 + 0.1428j,
       0.0000 + 0.0920j,  -0.0127 + 0.1428j,  -0.0785 - 0.0135j,  0.0023 - 0.1324j,
       0.0460 + 0.0460j,  -0.1324 + 0.0023j,  -0.0135 - 0.0785j,  0.1428 - 0.0127j,
       0.0920 + 0.0000j,   0.1428 - 0.0127j,  -0.0135 - 0.0785j, -0.1324 + 0.0023j,
       0.0460 + 0.0460j,   0.0023 - 0.1324j,  -0.0785 - 0.0135j, -0.0127 + 0.1428j,
       0.0000 + 0.0920j,  -0.0127 + 0.1428j,  -0.0785 - 0.0135j,  0.0023 - 0.1324j,
       0.0460 + 0.0460j,  -0.1324 + 0.0023j,  -0.0135 - 0.0785j,  0.1428 - 0.0127j,
       0.0920 + 0.0000j,   0.1428 - 0.0127j,  -0.0135 - 0.0785j, -0.1324 + 0.0023j,
       0.0460 + 0.0460j,   0.0023 - 0.1324j,  -0.0785 - 0.0135j, -0.0127 + 0.1428j,
       0.0000 + 0.0920j,  -0.0127 + 0.1428j,  -0.0785 - 0.0135j,  0.0023 - 0.1324j,]
    ieee80211a_sts =  [p*14.0 for p in ieee80211a_sts]

    rawofdm_lts_a = [
     -0.324729651212692 - 0.648506164550781j,  -0.301016688346863 + 0.359643161296844j,
     -0.147326365113258 + 1.058500766754150j,   0.308397918939590 + 0.353920727968216j,
      0.113463878631592 - 0.555182576179504j,   0.783052086830139 - 0.210373610258102j,
      1.038782119750977 + 0.523121595382690j,   0.038778871297836 + 1.057318687438965j,
      0.303817242383957 + 0.611358046531677j,   0.351564615964890 + 0.193092763423920j,
      0.042094588279724 + 0.011735804378986j,  -0.725173592567444 - 0.853282272815704j,
     -1.025235891342163 + 0.565869688987732j,  -0.180808752775192 + 1.309297442436218j,
     -0.232895791530609 - 0.031507849693298j,   0.803657352924347 + 0.023751318454742j,
      0.809926629066467 + 0.388462454080582j,  -0.696269154548645 + 1.213965773582458j,
      0.536057114601135 - 0.360280871391296j,   0.534684121608734 - 1.910227775573730j,
     -0.368418514728546 + 0.425022453069687j,   0.956639289855957 + 0.734396398067474j,
      0.715041041374207 + 0.537298798561096j,  -0.114415407180786 + 0.320772647857666j,
      0.258616387844086 - 0.332280337810516j,   0.405675143003464 + 0.680152058601379j,
      0.968100428581238 - 0.588708758354187j,   0.505425691604614 - 0.558997452259064j,
     -0.708929419517517 + 0.109176024794579j,  -0.107907086610794 - 1.016447663307190j,
      0.014984667301178 + 0.613836824893951j,  -0.272099196910858 + 0.367176473140717j,
      0.267610937356949 - 1.178259372711182j,  -0.502658367156982 - 0.648753941059113j,
     -0.524189591407776 - 0.665371358394623j,   0.179502278566360 + 0.150814265012741j,
     -0.793793082237244 + 0.505022168159485j,  -1.009974360466003 - 0.379487127065659j,
      0.107874453067780 - 0.148828387260437j,   0.290660738945007 + 0.710338413715363j,
     -0.821817219257355 + 0.384588837623596j,   0.023147732019424 - 0.065792113542557j,
      1.620932698249817 + 0.389522135257721j,  -0.228508919477463 - 0.353674590587616j,
     -0.821103811264038 - 0.008949890732765j,   0.688975334167480 + 0.764788985252380j,
      0.194231986999512 + 0.098834812641144j,  -0.246814742684364 + 0.934092581272125j,
     -0.280057817697525 + 0.248521834611893j,  -1.127288818359375 - 1.080757260322571j,
     -1.408289909362793 - 0.583601474761963j,  -0.066827774047852 - 0.672649919986725j,
     -0.003877311944962 - 0.231080800294876j,  -0.951217770576477 - 0.248973965644836j,
      0.549793362617493 - 0.196395725011826j,   0.285471946001053 + 0.495385587215424j,
     -0.476266354322433 - 0.027247756719589j,   0.536776483058929 - 0.045307524502277j,
     -0.651929199695587 + 0.185041368007660j,  -0.215558499097824 + 0.494138538837433j,
      0.370194315910339 + 0.114210337400436j,  -0.757714629173279 - 1.359393239021301j,
      0.677338123321533 - 1.213922619819641j,   0.811844050884247 - 0.758927166461945j]
    rawofdm_lts_a = [p*sf1*8.0 for p in rawofdm_lts_a]

    rawofdm_lts_b = [
      0.600603103637695 - 0.642856299877167j,   0.555812895298004 + 0.004549294710159j,
      0.584578216075897 + 0.615208506584167j,  -0.158044815063477 + 0.712308406829834j,
     -0.960626363754272 - 0.045826829969883j,  -0.660843372344971 + 0.553179442882538j,
     -0.514051437377930 + 0.805295705795288j,  -0.470963001251221 + 0.542817950248718j,
     -0.157838299870491 + 0.367176592350006j,   0.361174076795578 - 0.239544808864594j,
      0.132994443178177 + 0.043339848518372j,  -0.630867958068848 + 0.092171967029572j,
     -0.112968862056732 + 0.529931843280792j,  -0.312588065862656 + 0.876693308353424j,
     -0.275063216686249 + 0.800422430038452j,   0.361452966928482 + 0.974227130413055j,
     -0.522131204605103 - 0.099471867084503j,  -0.126634806394577 + 0.235434323549271j,
      0.062711864709854 + 0.397178828716278j,  -0.441921979188919 - 0.809287309646606j,
      0.653882503509521 - 0.261922299861908j,   0.915526270866394 - 0.259624749422073j,
      0.373903781175613 - 0.123748466372490j,  -0.182362228631973 - 0.272522628307343j,
     -0.247716516256332 - 1.031872987747192j,   0.130689084529877 + 0.174283951520920j,
     -0.088178709149361 - 0.294190526008606j,  -0.395977854728699 - 0.813577950000763j,
     -0.007834650576115 + 0.957274079322815j,   1.360511064529419 + 0.837775111198425j,
      0.145312249660492 - 0.354911625385284j,  -0.910548269748688 - 0.213123261928558j,
      0.456434339284897 + 0.589231312274933j,  -1.513518571853638 + 0.267002433538437j,
     -1.148481369018555 + 0.013660699129105j,   1.170813083648682 + 1.133775949478149j,
     -0.464080572128296 + 0.128193050622940j,   0.086121529340744 - 1.316248655319214j,
      0.523923635482788 + 0.232003122568130j,  -0.518945693969727 + 0.118401855230331j,
      0.494644552469254 - 1.870595455169678j,   0.539611577987671 - 0.933428049087524j,
      0.117953240871429 + 0.682337582111359j,   0.012290954589844 + 1.330901145935059j,
      0.113877646625042 + 0.735423684120178j,   0.237517833709717 - 1.392126679420471j,
      0.063391976058483 - 0.318477690219879j,   0.254892766475677 + 1.169041395187378j,
     -0.226137489080429 - 0.086121782660484j,   0.204377382993698 - 0.703465938568115j,
      0.658793449401855 - 1.584909915924072j,   0.095155924558640 - 0.721519708633423j,
      0.243911772966385 + 0.322156012058258j,  -0.281243562698364 - 0.556648969650269j,
     -0.430657893419266 + 0.189154192805290j,  -0.392031759023666 - 0.578258812427521j,
     -0.508833467960358 - 1.006151914596558j,   0.278049200773239 - 0.324500679969788j,
     -0.308683604001999 - 0.994299888610840j,  -0.341718554496765 + 1.144831657409668j,
      0.683588445186615 + 1.349358081817627j,   0.249712139368057 - 0.672454535961151j,
      0.062778450548649 - 0.041987776756287j,   0.524501562118530 - 0.361062824726105j]
    rawofdm_lts_b = [p*sf2*8.0 for p in rawofdm_lts_b]


    lts_p1_td = [ 0.024669-0.107433j,  0.016415-0.006623j,  0.013062-0.091110j, -0.017934-0.078501j,
                 -0.057756-0.011191j, -0.090012-0.165005j, -0.050551-0.074583j, -0.007830+0.019275j,
                  0.014054-0.064815j, -0.009202-0.044611j, -0.033039+0.011295j, -0.085177+0.074243j,
                 -0.093750+0.190504j,  0.107134+0.128529j,  0.126067-0.061901j,  0.001272+0.015053j,
                  0.007389+0.068532j, -0.113595+0.000020j, -0.089363+0.051601j,  0.085821+0.021979j,
                 -0.100575+0.014446j, -0.161196+0.038697j,  0.081164+0.058296j, -0.003052-0.016868j,
                  0.022747-0.194814j,  0.225686-0.051660j,  0.042143-0.001574j,  0.053749-0.040375j,
                  0.117816+0.094243j, -0.058117-0.058761j,  0.031918-0.033812j, -0.004409+0.138398j,
                 -0.117693-0.001383j, -0.025905-0.105277j,  0.071191-0.101128j,  0.154377+0.035383j,
                  0.023278+0.090436j,  0.016076+0.002616j,  0.096829+0.019331j, -0.027731+0.002849j,
                 -0.040670-0.023435j, -0.017765-0.003927j,  0.078902+0.126922j, -0.000606+0.209312j,
                 -0.100750+0.060697j,  0.127107-0.040051j, -0.009417-0.068272j, -0.033784-0.017376j,
                  0.161579+0.038349j, -0.043420+0.005139j, -0.073991+0.011840j,  0.006844+0.033588j,
                  0.061504+0.035781j, -0.003931-0.116527j, -0.164770-0.114230j,  0.084452+0.036558j,
                  0.062463-0.046611j, -0.181373+0.064362j, -0.072318+0.131308j,  0.008097+0.007076j,
                  0.087841+0.016679j,  0.019632-0.073720j, -0.119973-0.023967j, -0.021625-0.013796j,]
    lts_p1_td = [p*8.0*1.414 for p in lts_p1_td]

    lts_p2_td = [ 0.122134-0.046596j,  0.026542+0.085367j,  0.069549-0.032312j,  0.082203-0.211373j,
                 -0.012385-0.084468j, -0.064555+0.008442j, -0.010534+0.026724j,  0.051431+0.027548j,
                 -0.001780-0.012348j,  0.003324-0.015258j, -0.087874+0.020383j, -0.149043-0.002257j,
                 -0.045284+0.041097j, -0.006582+0.099742j,  0.020326-0.038062j, -0.074495-0.069172j,
                 -0.041050-0.023970j,  0.031707-0.112077j,- 0.094443-0.142268j,  0.057385-0.040169j,
                  0.096215+0.042457j, -0.058943+0.003971j, -0.065741+0.166023j, -0.055274+0.231561j,
                  0.083348-0.038136j, -0.032620-0.029839j, -0.078523-0.017633j,  0.158911-0.073057j,
                 -0.012936+0.055785j, -0.051628+0.067993j, -0.018533+0.087861j, -0.173331+0.109746j,
                  0.028001+0.080470j, -0.004709-0.057229j, -0.158928-0.102182j,  0.046456+0.148217j,
                 -0.002009+0.048579j, -0.045280-0.126869j,  0.099961-0.040213j, -0.034017-0.011446j,
                 -0.030505+0.027257j,  0.098228-0.068039j, -0.056479-0.015047j, -0.065307+0.082469j,
                  0.016710-0.008091j,  0.023867+0.066089j,  0.115679-0.016876j,  0.034644-0.090155j,
                 -0.023812-0.039454j, -0.043602-0.042300j, -0.167581+0.015616j,  0.005927-0.136206j,
                  0.160199-0.044921j, -0.031413+0.134010j, -0.147546-0.072072j,  0.024404-0.099948j,
                  0.132148-0.096305j,  0.022184-0.032067j,  0.046634+0.148753j,  0.038522+0.098441j,
                  0.027159+0.038133j,  0.146390+0.016489j,  0.037877+0.071817j,  0.038673+0.037374j,]
    lts_p2_td = [p*8.0*1.414 for p in lts_p2_td]

    # NOTE: Preambles in Time-Domain:
    zeros = [0]*(self.fft_length)
    if options.mode == 'PNC':
      preambles_tmp=[ieee80211a_sts, ieee80211a_sts, rawofdm_lts_a, rawofdm_lts_b]
      #{ YQ: add preamble of A and B
      if options.node == "A":
        preambles_tmp = [ieee80211a_sts, ieee80211a_sts, zeros, zeros, rawofdm_lts_a, zeros]
      elif options.node == "B":
        #preambles_tmp = [zeros, zeros, ieee80211a_sts, ieee80211a_sts, zeros, rawofdm_lts_b]
        #FIXME: the zeros before STS will generate plateau when auto-correlation ?
        preambles_tmp = [ieee80211a_sts, ieee80211a_sts, zeros, rawofdm_lts_b]
    else:
      # for RawOFDM
      preambles_tmp=[ieee_short_preamble, ieee_short_preamble, ieee_long_preamble, ieee_long_preamble]

    #  1) insert preambles @ Tx chain, 
    #  2) do cross correlation @ Rx chain
    self.preambles_td = preambles_tmp
    self.padding_td = [zeros]

    self.cross_corr_preambles = [ieee_long_preamble, ieee_long_preamble]



  def __init__(self, options):
    # for simplicity make fft_length and occupied_tones even
    self.fft_length     = int(math.ceil(options.fft_length / 2.0) * 2)
    self.occupied_tones = int(math.ceil(options.occupied_tones / 2.0) * 2)
    self.cp_length      = options.cp_length

    pad = (self.fft_length - self.occupied_tones)/2
    self.pad = pad
    
    # generat PNC preambles
    self.construct_pnc_preamble(options)

    #print preambles
    self.padded_preambles = [ self.pad_symbol(p) for p in self.preambles ]

    # build carrier map
    half = self.occupied_tones/2
    carriers = half*[1,]

    # set as pilots every 14 subcarriers
    num_pilots = 0
    # NOTE: bug fix, rawofdm use 13, but 11 standard use 14
    for i in range(5, half, 14):
      carriers[i] = 2
      num_pilots += 1

    # mirror around the DC in the middle
    carriers.extend([0] + list(reversed(carriers)))
    self.carriers = carriers

    print "====================== len(self.carriers) = %d" %(len(self.carriers) )

    padded_carriers = self.fft_length*[0,]
    padded_carriers[pad : pad + self.occupied_tones + 1] = carriers
    self.padded_carriers = padded_carriers

    self.pilot_tones = num_pilots * 2
    self.data_tones = self.occupied_tones - self.pilot_tones

    # for all I/O sizing purposes, include DC
    self.occupied_tones+= 1

    if options.verbose:
        self._print_verbage()

  def add_options(normal, expert):
    """
    Adds OFDM-specific options to the Options Parser
    """
    expert.add_option("", "--fft-length", type="intx", default=64,
                      help="set the number of FFT bins [default=%default]")
    expert.add_option("", "--occupied-tones", type="intx", default=52,
                      help="set the number of occupied FFT bins [default=%default]")
    expert.add_option("", "--cp-length", type="intx", default=16,
                      help="set the number of bits in the cyclic prefix [default=%default]")

  add_options = staticmethod(add_options)

  def _print_verbage(self):
    """
    Prints information about the OFDM modulator
    """
    from sys import stderr
    print >> stderr, "\nRaw OFDM parameters:"
    print >> stderr, "FFT length:      %3d"   % (self.fft_length)
    print >> stderr, "Occupied Tones:  %3d (with DC)"   % (self.occupied_tones)
    print >> stderr, "Pilot Tones:     %3d"   % (self.pilot_tones)
    print >> stderr, "Data Tones:      %3d"   % (self.data_tones)
    print >> stderr, "CP length:       %3d"   % (self.cp_length)
    print >> stderr, "Extra Preambles: %3d"   % (self.num_preambles)
