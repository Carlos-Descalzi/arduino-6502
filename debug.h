#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <avr/pgmspace.h>
#include "serial.h"
//#define DEBUG(str)	serial_write_str(str)

#ifdef DEBUGMSG
#include <stdio.h>
#define DEBUG(args...)	fprintf(&serial_out,args)
#define DEBUGP(args...)	fprintf_P(&serial_out,args)
#else
//#define DEBUG(args...)
#define DEBUGP(args...)
#endif

#endif
