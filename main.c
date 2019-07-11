#include "rom.h"
#include <avr/io.h>
#include "serial.h"
#include "debug.h"
#include "up6502.h"
#include <util/delay.h>
#define RAM_TOP 3072
static volatile unsigned char loram[RAM_TOP];

#define UART_DATA       ((unsigned short)0x8800)
#define UART_STATUS     ((unsigned short)0x8801)
#define UART_CMD        ((unsigned short)0x8802)
#define UART_CTRL       ((unsigned short)0x8803)


uint8_t mem_read_callback(uint16_t addr, uint8_t*val){
	if (addr < RAM_TOP){
		*val = loram[addr];
		return 0;
	} else if (addr == UART_DATA){
		*val = serial_read();
		//DEBUG("READ:%c %02x\n",*val,*val);
		return 0;
	} else if (addr == UART_STATUS){
		*val = 0x10 | (serial_is_data() ? 0x08 : 0x00);
		return 0;
	} else if (addr >= ROM_ADDR){
		*val = pgm_read_byte_near(ROM + (addr - ROM_ADDR));
		return 0;
	}
	
	return 1;
}

uint8_t mem_write_callback(uint16_t addr, uint8_t val){
	if (addr < RAM_TOP){
		loram[addr] = val;
		return 0;
	} else if (addr == UART_DATA){
		serial_write(val);
		return 0;
	}
	return 1;
}

int main(){
	serial_init(9600);
	serial_write_str("START!!!\n");
	serial_write_str("1\n");
	for (int i=0;i<RAM_TOP;loram[i++]=0);
	up6502_setup(mem_read_callback,mem_write_callback);
	while(1){
		up6502_loop();
	}
	return 0;
}




