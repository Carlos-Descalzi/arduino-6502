#include "definitions.h"
#include "rom.h"

#include <Adafruit_GFX.h>    // Core graphics library
#include "SWTFT.h" // Hardware-specific library


#define DO_DEBUG


#ifdef DO_DEBUG
#define DEBUG_INIT()   Serial.begin(9600)
#define DEBUG_PRINT  Serial.print
#define DEBUG_PRINTLN  Serial.println
#else
#define DEBUG_INIT()
#define DEBUG_PRINT(...)  
#endif

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


SWTFT tft;

#define DEBUG(msg,...) 
#define ADDRL PORTC
#define ADDRL_DIR DDRC
#define ADDRH PORTA
#define ADDRH_DIR DDRA
#define DATA PORTB
#define DATA_DIR DDRB

#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_D 0x08
#define FLAG_B 0x10
#define FLAG_V 0x20
#define FLAG_N 0x80

#define PIN_RDY PIND2
#define FLAG_RW 0x20
#define PIN_IRQ PIND3
#define PIN_NMI PIND4

#define UART_DATA       ((unsigned short)0x8800)
#define UART_STATUS     ((unsigned short)0x8801)
#define UART_CMD        ((unsigned short)0x8802)
#define UART_CTRL       ((unsigned short)0x8803)

#define	EXTRAM_ADDR	((unsigned short)0x1000)
#define INT_ADDR	((unsigned short)0xFFF8)
#define INT_ABORTL	((unsigned short)0xFFF8)
#define INT_ABORTH	((unsigned short)0xFFF9)
#define INT_NMIL	((unsigned short)0xFFFA)
#define INT_NMIH	((unsigned short)0xFFFB)
#define INT_RESETL	((unsigned short)0xFFFC)
#define INT_RESETH	((unsigned short)0xFFFD)
#define INT_IRQL	((unsigned short)0xFFFE)
#define INT_IRQH	((unsigned short)0xFFFF)

//taddress address;
taddress fetch_address;

unsigned char a;
unsigned char x;
unsigned char y;
unsigned char p;
unsigned char s;
taddress pc;
char old_nmi;

int debug;

unsigned char loram[4096];

void setup();

void set_read(){
    //PORTB |= FLAG_RW;
    //DATA_DIR = 0x00;
}
void set_write(){
    //PORTB &= ~FLAG_RW;
    //DATA_DIR = 0xFF;
}
#ifndef _TEST_
void wait_addr_ready(){
    //while (!PIN_RDY);
}
#endif
char irq(){
    return 0;//return !PIN_IRQ && (p & FLAG_I);
}
char nmi(){
    /*if (old_nmi != PIN_NMI){
        old_nmi = PIN_NMI;

        if (!PIN_NMI){
            return 1;
        }
    }*/
    return 0;
}

void update_z_n(unsigned char v){
  if (v)         p &= ~FLAG_Z; else p |= FLAG_Z;
  if (v & 0x80)  p |= FLAG_N;  else p &= ~FLAG_N;
}
#define update_c(c)  if (c)         p |= FLAG_C;  else p &= ~FLAG_C;

unsigned char fetch_addr(taddress _address){
    set_read();
    if (_address >= ROM_ADDR){
      return pgm_read_byte_near(rom + (_address - ROM_ADDR));
    } else if (_address == UART_DATA){
      return Serial.read(); 
    } else if (_address == UART_STATUS){	
      return 0x10 | (Serial.available() > 0 ? 0x08 : 0x00);
    } else if (_address == UART_CMD){
      return 0;	
    } else if (_address == UART_CTRL){
      return 0;	
    } else if (_address >= EXTRAM_ADDR){
      DEBUG_PRINT("********** FETCH HIRAM ");
      DEBUG_PRINTLN(_address.w,HEX);
      return 0;//DATA;
    }
    return loram[_address.w];
}

void store_at(unsigned char value, taddress _address){
    set_write();
    if (_address == UART_DATA){
      //Serial.print((char)value);
      tft.print((char)value);
    } else if (_address == UART_CTRL){
    } else if (_address == UART_CMD){
    } else if (_address == UART_STATUS){
    } else if (_address < EXTRAM_ADDR){
      loram[_address] = value;
    } else if (_address < ROM_ADDR){
      DEBUG_PRINT("********** STORE HIRAM ");
      DEBUG_PRINTLN(_address.w,HEX);
    }
}

unsigned char fetch_a(){
    return a;
}
void store_a(unsigned char value){
    a = value;
}
void store_lastdir_a(unsigned char value){
    a = value;
}
unsigned char fetch_imm(){
    unsigned char v;

    v = fetch_addr(pc);
    pc++;

    return v;
}

/** ZP ADDRESSING **/

unsigned char fetch_z(){
    fetch_address.b.h = 0;
    fetch_address.b.l = fetch_imm();
    return fetch_addr(fetch_address);
}
void store_z(unsigned char value){
    fetch_address.b.h = 0;
    fetch_address.b.l = fetch_imm();
    store_at(value,fetch_address);
}
void store_lastdir_z(unsigned char value){
    store_at(value,fetch_address);
}
/** ZP + X ADDRESSING **/

unsigned char fetch_z_x(){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + x) & 0xFF;
    return fetch_addr(fetch_address);
}

void store_z_x(unsigned char value){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + x) & 0xFF;
    store_at(value,fetch_address);
}
void store_lastdir_z_x(unsigned char value){
  store_at(value,fetch_address);
}

/** ZP + Y ADDRESSING **/

unsigned char fetch_z_y(){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + y) & 0xFF;
    return fetch_addr(fetch_address);
}

void store_z_y(unsigned char value){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + y) & 0xFF;
    store_at(value,fetch_address);
}
void store_lastdir_z_y(unsigned char value){
  store_at(value,fetch_address);
}

/** ABS ADDRESSING **/

unsigned char fetch_abs(){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    return fetch_addr(fetch_address);
}

void store_abs(unsigned char value){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    store_at(value,fetch_address);
}

void store_lastdir_abs(unsigned char value){
  store_at(value,fetch_address);
}
/** ABS + X ADDRESSING **/
unsigned char fetch_abs_x(){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address += x;
    return fetch_addr(fetch_address);
}

void store_abs_x(unsigned char value){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address += x;
    store_at(value,fetch_address);
}
void store_lastdir_abs_x(unsigned char value){
  store_at(value,fetch_address);
}
/** ABS + Y ADDRESSING **/

unsigned char fetch_abs_y(){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address += y;
    return fetch_addr(fetch_address);
}
void store_abs_y(unsigned char value){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address += y;
    store_at(value,fetch_address);
}
void store_lastdir_abs_y(unsigned char value){
  store_at(value,fetch_address);
}

/** IND Y ADDRESSING **/

unsigned char fetch_ind_y(){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = fetch_imm();
    ind.b.l = fetch_addr(address);
    address++;
    ind.b.h = fetch_addr(address);
    ind+= y;
    fetch_address = ind;
    return fetch_addr(fetch_address);
}

void store_ind_y(unsigned char value){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = fetch_imm();
    ind.b.l = fetch_addr(address);
    address++;
    ind.b.h = fetch_addr(address);
    ind+=y;
    fetch_address = ind;
    store_at(value,fetch_address);
}


unsigned char fetch_x_ind(){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = (fetch_imm() + x) & 0xFF;
    ind.b.l = fetch_addr(address);
    address++;
    ind.b.h = fetch_addr(address);
    fetch_address = ind;
    return fetch_addr(fetch_address);
}

void store_x_ind(unsigned char value){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = (fetch_imm() + x) & 0xFF;
    ind.b.l = fetch_addr(address);
    address++;
    ind.b.h = fetch_addr(address);
    fetch_address = ind;
    store_at(value,fetch_address);
}

void store_lastdir_x_ind(unsigned char value){
    store_at(value,fetch_address);
}


void do_push(unsigned char value){
    taddress _s;
    _s = (unsigned short)(0x100|s);
    store_at(value,_s);
    s--;
}

unsigned char do_pop(){
    taddress _s;
    s++;
    _s = (unsigned short)(0x100|s);
    return fetch_addr(_s);
}

void handle_interrupt(unsigned short vector,const taddress _pc, unsigned char _p){
    taddress address;

    do_push(_pc.b.h);
    do_push(_pc.b.l);
    do_push(_p);
    address = vector;
    pc.b.l = fetch_addr(address);
    address++;
    pc.b.h = fetch_addr(address);
}

void op_rti(){
    taddress _pc;
    p = do_pop();
    _pc.b.l = do_pop();
    _pc.b.h = do_pop();
    pc = _pc;
}

void op_unknown(){}

void op_brk(){
    handle_interrupt(0xFFFE,pc,p|FLAG_B);
}

void op_jsr_abs(){
    taddress _pc,current_pc;
    _pc.b.l = fetch_imm();
    _pc.b.h = fetch_imm();
    current_pc = pc;
    current_pc-=1;
    do_push(current_pc.b.h);
    do_push(current_pc.b.l);
    pc = _pc;
}

void op_rts(){
    taddress _pc;
    _pc.b.l = do_pop();
    _pc.b.h = do_pop();
    _pc++;
    pc = _pc;
}


void op_jmp_abs(){
    taddress _pc;
    _pc.b.l = fetch_imm();
    _pc.b.h = fetch_imm();
    pc = _pc;
}

void op_jmp_ind(){
    taddress address;
    taddress _pc;

    address.b.l = fetch_imm();
    address.b.h = fetch_imm();
    _pc.b.l = fetch_addr(address);  
    address++;
    _pc.b.h = fetch_addr(address);

    pc = _pc;
}

void op_nop(){
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
}

#define do_op_ora(mode)\
void op_ora_##mode(){\
    a |= fetch_##mode();\
    update_z_n(a);\
}
do_op_ora(imm)
do_op_ora(ind_y)
do_op_ora(x_ind)
do_op_ora(z)
do_op_ora(abs)
do_op_ora(abs_x)
do_op_ora(abs_y)
do_op_ora(z_x)
#define do_op_and(mode)\
void op_and_##mode(){\
    a &= fetch_##mode();\
    update_z_n(a);\
}
do_op_and(x_ind)
do_op_and(ind_y)
do_op_and(imm)
do_op_and(abs)
do_op_and(z)
do_op_and(abs_y)
do_op_and(abs_x)
do_op_and(z_x)
#define do_op_eor(mode)\
void op_eor_##mode(){\
    a ^= fetch_##mode();\
    update_z_n(a);\
}
do_op_eor(x_ind)
do_op_eor(ind_y)
do_op_eor(imm)
do_op_eor(abs)
do_op_eor(z)
do_op_eor(abs_y)
do_op_eor(abs_x)
do_op_eor(z_x)

unsigned char bcd(unsigned char v){
  return (v & 0xF) + (v >> 4) * 10;
}

#define do_op_adc(mode)\
void op_adc_##mode(){\
    unsigned char r = fetch_##mode();\
    unsigned short v;\
    if (p & FLAG_D){\
      v = bcd(a) + bcd(r);\
      if (p & FLAG_C) v++;\
      update_c(v > 99);\
    } else {\
      v = a + r;\
      if (p & FLAG_C) v++;\
      update_c(v > 0xFF);\
    }\
    if ((r & 0x80) != (v & 0x80)) p |= FLAG_V; else p &= ~FLAG_V;\
    a = (unsigned char)(v & 0xFF);\
    update_z_n(a);\
}
do_op_adc(x_ind)
do_op_adc(z)
do_op_adc(imm)
do_op_adc(abs)
do_op_adc(ind_y)
do_op_adc(z_x)
do_op_adc(abs_y)
do_op_adc(abs_x)
#define do_op_cmp(mode)\
void op_cmp_##mode(){\
    unsigned char m = fetch_##mode();\
    unsigned char v = a - m;\
    update_z_n(v);\
    update_c(a >= m);\
}
do_op_cmp(imm)
do_op_cmp(abs)
do_op_cmp(abs_x)
do_op_cmp(abs_y)
do_op_cmp(ind_y)
do_op_cmp(z_x)
do_op_cmp(z)
do_op_cmp(x_ind)

#define do_op_cpr(r,mode)\
void op_cp##r##_##mode(){\
    unsigned char m = fetch_##mode();\
    unsigned char v = r - m;\
    update_z_n(v);\
    update_c(r >=m);\
}

do_op_cpr(y,imm)
do_op_cpr(y,z)
do_op_cpr(y,abs)
do_op_cpr(x,imm)
do_op_cpr(x,z)
do_op_cpr(x,abs)

#define do_op_lsr(mode)\
void op_lsr_##mode(){\
    unsigned char v = fetch_##mode();\
    unsigned char cv = (v & 1);\
    v >>=1;\
    store_lastdir_##mode(v);\
    if (v == 0) p |= FLAG_Z; else p &= ~FLAG_Z;\
    p &= ~FLAG_N;\
    update_c(cv);\
}

do_op_lsr(z)
do_op_lsr(a)
do_op_lsr(z_x)
do_op_lsr(abs_x)
do_op_lsr(abs)

#define do_op_rol(mode)\
void op_rol_##mode(){\
    unsigned char v = fetch_##mode();\
    unsigned char c = v & 0x80;\
    v <<=1;\
    if (p & FLAG_C) v|=1;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
    update_c(c);\
}
do_op_rol(z)
do_op_rol(a)
do_op_rol(abs_x)
do_op_rol(abs)
do_op_rol(z_x)

#define do_op_ror(mode)\
void op_ror_##mode(){\
    unsigned char v = fetch_##mode();\
    unsigned char c = (v & 1);\
    v >>=1;\
    if (p & FLAG_C) v|=0x80;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
    update_c(c);\
}

do_op_ror(z)
do_op_ror(a)
do_op_ror(abs)
do_op_ror(z_x)
do_op_ror(abs_x)

void op_plp(){
    p = do_pop();
}
void op_pha(){
    do_push(a);
}
void op_php(){
    do_push(p);
}
void op_pla(){
    a = do_pop();
    update_z_n(a);
}

#define do_op_st(r,mode)\
void op_st##r##_##mode(){\
    store_##mode(r);\
}

do_op_st(a,z)
do_op_st(a,abs)
do_op_st(a,x_ind)
do_op_st(a,ind_y)
do_op_st(a,abs_x)
do_op_st(a,abs_y)
do_op_st(a,z_x)

do_op_st(x,z)
do_op_st(x,abs)
do_op_st(x,z_y)

do_op_st(y,z)
do_op_st(y,abs)
do_op_st(y,z_x)

#define do_op_asl(mode)\
void op_asl_##mode(){\
    unsigned char r = fetch_##mode();\
    unsigned char c = r & 0x80;\
    unsigned char v = (r << 1) & 0xFE;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
    update_c(c);\
}
do_op_asl(a)
do_op_asl(z)
do_op_asl(z_x)
do_op_asl(abs)
do_op_asl(abs_x)

void op_cli(){   p &= ~FLAG_I;}
void op_clc(){   p &= ~FLAG_C;}
void op_cld(){   p &= ~FLAG_D;}
void op_clv(){   p &= ~FLAG_V;}
void op_sei(){   p |= FLAG_I;}
void op_sec(){   p |= FLAG_C;}
void op_sed(){   p |= FLAG_D;}

#define do_op_bit(mode)\
void op_bit_##mode(){\
    unsigned char c = a & fetch_##mode();\
    if (c & 0x80)  p |= FLAG_N; else p &= ~FLAG_N;\
    if (c & 0x40)  p |= FLAG_V; else p &= ~FLAG_V;\
    if (!c)        p |= FLAG_Z; else p &= ~FLAG_Z;\
}
do_op_bit(z)
do_op_bit(abs)

#define do_branch(name,cond)\
void op_##name##_rel(){\
    char offset = fetch_imm();\
    if (cond){\
        pc.w+=offset;\
    }\
}
do_branch(bpl,!(p & FLAG_N))
do_branch(bmi,(p & FLAG_N))
do_branch(bvc,!(p & FLAG_V))
do_branch(bvs,(p & FLAG_V))
do_branch(beq,(p & FLAG_Z))
do_branch(bne,!(p & FLAG_Z))
do_branch(bcs,(p & FLAG_C))
do_branch(bcc,!(p & FLAG_C))


#define do_op_t(s,d)\
void op_t##s##d(){\
    d = s;\
    update_z_n(d);\
}

void op_txs(){
  s = x;
}

do_op_t(s,x)
do_op_t(a,x)
do_op_t(x,a)
do_op_t(a,y)
do_op_t(y,a)


#define do_op_ld(r,mode)\
void op_ld##r##_##mode(){\
    r = fetch_##mode();\
    update_z_n(r);\
}
do_op_ld(a,imm)
do_op_ld(a,x_ind)
do_op_ld(a,z)
do_op_ld(a,abs)
do_op_ld(a,abs_x)
do_op_ld(a,abs_y)
do_op_ld(a,ind_y)
do_op_ld(a,z_x)

do_op_ld(y,imm)
do_op_ld(y,z)
do_op_ld(y,abs)
do_op_ld(y,z_x)
do_op_ld(y,abs_x)

do_op_ld(x,imm)
do_op_ld(x,z)
do_op_ld(x,abs)
do_op_ld(x,z_y)
do_op_ld(x,abs_y)

#define do_op_inc(mode)\
void op_inc_##mode(){\
    unsigned char v = fetch_##mode() +1;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
}

do_op_inc(z)
do_op_inc(abs)
do_op_inc(z_x)
do_op_inc(abs_x)

#define do_op_inc_r(r)\
void op_in##r(){\
    r++;\
    update_z_n(r);\
}

do_op_inc_r(y)
do_op_inc_r(x)

#define do_op_dec_r(r)\
void op_de##r(){\
    r--;\
    update_z_n(r);\
}
do_op_dec_r(x)
do_op_dec_r(y)

#define do_op_dec(mode)\
void op_dec_##mode(){\
    unsigned char v = fetch_##mode() -1;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
}

do_op_dec(z)
do_op_dec(abs)
do_op_dec(z_x)
do_op_dec(abs_x)

#define do_op_sbc(mode)\
void op_sbc_##mode(){\
    unsigned char c = fetch_##mode();\
    short v;\
    if (p & FLAG_D){\
      v = bcd(a) - bcd(c);\
      if (!(p & FLAG_C)) v--;\
      if (v > 99 || v < 0) p |= FLAG_V; else p &= ~FLAG_V;\
    } else {\
      v = a - c;\
      if (!(p & FLAG_C)) v--;\
      if (v > 127 || v < -128) p |= FLAG_V; else p &= ~FLAG_V;\
    }\
    update_z_n((unsigned char)(v & 0xFF));\
    update_c(v >= 0);\
    a = (unsigned char)(v & 0xFF);\
}
do_op_sbc(x_ind)
do_op_sbc(z)
do_op_sbc(abs)
do_op_sbc(ind_y)
do_op_sbc(z_x)
do_op_sbc(abs_x)
do_op_sbc(abs_y)
do_op_sbc(imm)

void op_nop();

#define OP(op) {op,#op}

typedef struct {
  void (*opr)(void);
  const char*name;
} Operator;

//typedef void (*Operator)();

const Operator OPERATORS[] = {
    // 0x00
    OP(op_brk),         OP(op_ora_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_ora_z),       OP(op_asl_z),       OP(op_unknown),
    OP(op_php),         OP(op_ora_imm),     OP(op_asl_a),   OP(op_unknown), OP(op_unknown),     OP(op_ora_abs),     OP(op_asl_abs),     OP(op_unknown),
    // 0x10
    OP(op_bpl_rel),     OP(op_ora_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_ora_z_x),     OP(op_asl_z_x),     OP(op_unknown),
    OP(op_clc),         OP(op_ora_abs_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_ora_abs_x),   OP(op_asl_abs_x),   OP(op_unknown),
     // 0x20
    OP(op_jsr_abs),     OP(op_and_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_bit_z),       OP(op_and_z),       OP(op_rol_z),       OP(op_unknown),
    OP(op_plp),         OP(op_and_imm),     OP(op_rol_a),   OP(op_unknown), OP(op_bit_abs),     OP(op_and_abs),     OP(op_rol_abs),     OP(op_unknown),
    // 0x30
    OP(op_bmi_rel),     OP(op_and_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_and_z_x),     OP(op_rol_z_x),     OP(op_unknown),
    OP(op_sec),         OP(op_and_abs_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_and_abs_x),   OP(op_rol_abs_x),   OP(op_unknown),
    // 0x40
    OP(op_rti),         OP(op_eor_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_eor_z),       OP(op_lsr_z),       OP(op_unknown),
    OP(op_pha),         OP(op_eor_imm),     OP(op_lsr_a),   OP(op_unknown), OP(op_jmp_abs),     OP(op_eor_abs),     OP(op_lsr_abs),     OP(op_unknown),
    // 0x50
    OP(op_bvc_rel),     OP(op_eor_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_eor_z_x),     OP(op_lsr_z_x),     OP(op_unknown),
    OP(op_cli),         OP(op_eor_abs_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_eor_abs_x),   OP(op_lsr_abs_x),   OP(op_unknown),
    // 0x60
    OP(op_rts),         OP(op_adc_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_adc_z),       OP(op_ror_z),       OP(op_unknown),
    OP(op_pla),         OP(op_adc_imm),     OP(op_ror_a),   OP(op_unknown), OP(op_jmp_ind),     OP(op_adc_abs),     OP(op_ror_abs),     OP(op_unknown),
    // 0x70
    OP(op_bvs_rel),     OP(op_adc_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_adc_z_x),     OP(op_ror_z_x),     OP(op_unknown),
    OP(op_sei),         OP(op_adc_abs_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_adc_abs_x),   OP(op_ror_abs_x),   OP(op_unknown),
    // 0x80
    OP(op_unknown),     OP(op_sta_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_sty_z),       OP(op_sta_z),       OP(op_stx_z),       OP(op_unknown),
    OP(op_dey),         OP(op_unknown),     OP(op_txa),     OP(op_unknown), OP(op_sty_abs), 	OP(op_sta_abs),     OP(op_stx_abs),     OP(op_unknown),
    // 0x90
    OP(op_bcc_rel),     OP(op_sta_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_sty_z_x),     OP(op_sta_z_x),     OP(op_stx_z_y),     OP(op_unknown),
    OP(op_tya),         OP(op_sta_abs_y),   OP(op_txs),     OP(op_unknown), OP(op_unknown),     OP(op_sta_abs_x),   OP(op_unknown),     OP(op_unknown),
    // 0xA0
    OP(op_ldy_imm),     OP(op_lda_x_ind),   OP(op_ldx_imm), OP(op_unknown), OP(op_ldy_z),       OP(op_lda_z),       OP(op_ldx_z),       OP(op_unknown),
    OP(op_tay),         OP(op_lda_imm),     OP(op_tax),     OP(op_unknown), OP(op_ldy_abs),     OP(op_lda_abs),     OP(op_ldx_abs),     OP(op_unknown),
    //0xB0),
    OP(op_bcs_rel),     OP(op_lda_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_ldy_z_x),     OP(op_lda_z_x),     OP(op_ldx_z_y),     OP(op_unknown),
    OP(op_clv),         OP(op_lda_abs_y),   OP(op_tsx),     OP(op_unknown), OP(op_ldy_abs_x),   OP(op_lda_abs_x),   OP(op_ldx_abs_y),   OP(op_unknown),
    //0xC0
    OP(op_cpy_imm),     OP(op_cmp_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_cpy_z),       OP(op_cmp_z),       OP(op_dec_z),       OP(op_unknown),
    OP(op_iny),         OP(op_cmp_imm),     OP(op_dex),     OP(op_unknown), OP(op_cpy_abs),     OP(op_cmp_abs),     OP(op_dec_abs),     OP(op_unknown),
    // 0xD0
    OP(op_bne_rel),     OP(op_cmp_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_cmp_z_x),     OP(op_dec_z_x),     OP(op_unknown),
    OP(op_cld),         OP(op_cmp_abs_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_cmp_abs_x),   OP(op_dec_abs_x),   OP(op_unknown),
    // 0xE0
    OP(op_cpx_imm),     OP(op_sbc_x_ind),   OP(op_unknown), OP(op_unknown), OP(op_cpx_z),       OP(op_sbc_z),       OP(op_inc_z),       OP(op_unknown),
    OP(op_inx),         OP(op_sbc_imm),     OP(op_nop),     OP(op_unknown), OP(op_cpx_abs),     OP(op_sbc_abs),     OP(op_inc_abs),     OP(op_unknown),
    // 0xF0
    OP(op_beq_rel),     OP(op_sbc_ind_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_sbc_z_x),     OP(op_inc_z_x),     OP(op_unknown),
    OP(op_sed),         OP(op_sbc_abs_y),   OP(op_unknown), OP(op_unknown), OP(op_unknown),     OP(op_sbc_abs_x),   OP(op_inc_abs_x),   OP(op_unknown)
};


void start(){
  taddress address;

  address = 0xFFFC;
  pc.b.l = fetch_addr(address);
  address++;
  pc.b.h = fetch_addr(address);        
}

void reset(){
 	a = x = y = p = s = 0;
	start();
}

void do_dump(){
  DEBUG_PRINT("A:");DEBUG_PRINT(a,HEX);
  DEBUG_PRINT(",X:");DEBUG_PRINT(x,HEX);
  DEBUG_PRINT(",Y:");DEBUG_PRINT(y,HEX);
  DEBUG_PRINT(",P:");
  
  DEBUG_PRINT(p & FLAG_N ? "N": "-");
  DEBUG_PRINT("-");
  DEBUG_PRINT(p & FLAG_V ? "V": "-");
  DEBUG_PRINT(p & FLAG_B ? "B": "-");
  DEBUG_PRINT(p & FLAG_D ? "D": "-");
  DEBUG_PRINT(p & FLAG_I ? "I": "-");
  DEBUG_PRINT(p & FLAG_Z ? "Z": "-");
  DEBUG_PRINT(p & FLAG_C ? "C": "-");
  DEBUG_PRINT(",S:");DEBUG_PRINTLN(s,HEX);
}
int dump(){
  return debug;//pc.w >= 0xC8ED && pc.w <= 0xC927;
}
//#define _DUMP_ 1
void loop(){

    unsigned char opcode;
	if (nmi()){
		unsigned short _p = p;
		p &= ~FLAG_I;
		handle_interrupt(0xFFFA,pc,_p);
	} else if (irq()){
		unsigned short _p = p;
		p &= ~FLAG_I;
		handle_interrupt(0xFFFE,pc,_p);
	}
  if(dump()){
        DEBUG_PRINT("Address:");
        DEBUG_PRINT(pc.w,HEX);
  }  

	opcode = fetch_imm();
  if(dump()){
        DEBUG_PRINT(",Opcode:");
        DEBUG_PRINT(opcode,HEX);
        DEBUG_PRINT(" ");
        DEBUG_PRINT(OPERATORS[opcode].name);
        DEBUG_PRINT("\t ");
  }
        if (OPERATORS[opcode].opr == op_unknown){
          DEBUG_PRINT("*** UNKNOWN OPCODE ***");
          DEBUG_PRINT(opcode,HEX);
          DEBUG_PRINT(" ");
          DEBUG_PRINTLN(pc.w-1,HEX);
          while (1) {}
        }
	OPERATORS[opcode].opr();

  if(dump()){
        DEBUG_PRINT("MEM:");
        DEBUG_PRINT(loram[0x10],HEX);
        DEBUG_PRINT(" ");
        do_dump();
        //delay(50);
  }
}

void setup(){
    //DATA_DIR = 0x00;
    //ADDRL_DIR = 0xFF;
    //ADDRH_DIR = 0xFF;
    //DDRD = 0x20;
    Serial.begin(9600);

  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(identifier);

  tft.setRotation(1);
  tft.fillScreen(BLACK);
  tft.fillScreen(BLACK);
  tft.fillScreen(BLACK);

  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);



    for (int i=0;i<4096;loram[i++]=0);
    reset();
    DEBUG_PRINTLN("Setup done, running....");
    debug = 0;
}





