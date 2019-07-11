#!/usr/bin/python

import sys
TEMPLATE="""
#ifndef _ROM_H_
#define _ROM_H_
#include <avr/pgmspace.h>
#define ROM_ADDR  ((unsigned short)0xC000)
const unsigned char ROM[16384] PROGMEM = {
%s
};
#endif
"""

def format_data(data):
	lines = []
	while len(data) > 0:
		chunk = data[0:16]
		data = data[16:]
		lines.append(','.join(['0x%02x' % ord(c) for c in chunk]) + (',' if len(data) > 0 else ''))
	return lines
	
def write_rom(path):
	print path
	with open(path,'r') as f:
		data = f.read()
	
	with open('rom.h','w') as f:
		f.write(TEMPLATE % '\n'.join(format_data(data)))

if len(sys.argv) == 0:
	exit(1)
write_rom(sys.argv[1])
