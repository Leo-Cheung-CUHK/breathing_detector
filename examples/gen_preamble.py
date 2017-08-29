#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math, random, cmath


occupied_tones = 53

def random_symbol():

	# NOTE: pilot bins have power 1 (see mapper/demapper)
	# we make our preamble have (complex) power 1 per bin as well x scaling factor
	sf = 1 # FIXME: allow non-unity scaling factors
	psk = lambda : cmath.exp(1j * 2 * math.pi * random.random())
	symbol = [ sf * psk() for i in range(occupied_tones)]
	# fill in the null DC
	half = occupied_tones/2
	sym = symbol[:half] + [0] + symbol[half+1:]
	print len(sym), half
	return sym


num_preambles = 2
# make the preamble
random.seed(1633)
#random.seed()
preambles = [ random_symbol() for i in range(num_preambles) ]

for i in range(len(preambles)):
	pre = preambles[i]
	for j in range(len(pre)):
		print pre[j]
	print "==================================="

