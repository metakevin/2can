/*
 * _2canboot.c
 *
 * Created: 10/28/2012 8:38:11 PM
 *  Author: kday
 * NOTE: see copyright below.  I have made some modifications to it. 
 */ 

/*
stk500boot.c  20030810

Copyright (c) 2003, Jason P. Kyle
All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Licence can be viewed at http://www.fsf.org/licenses/gpl.txt


Target = Atmel AVR m128,m64,m32,m16,m8,m162,m163,m169,m8515,m8535
ATmega161 has a very small boot block so isn't supported.

Tested with m128,m8,m163 - feel free to let me know how/if it works for you.
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define F_CPU			16000000L
#define BAUD_RATE		115200L

#define DECRYPT 0
#define ENCRYPT 1
//#define DES_ENCRYPTION

#define HW_VER	0x02
#define SW_MAJOR	0x01
#define SW_MINOR	0x0e



#define SIG1	0x1E	// Yep, Atmel is the only manufacturer of AVR micros.  Single source :(
#if defined __AVR_ATmega128__
	#define SIG2	0x97
	#define SIG3	0x02
	#define PAGE_SIZE	0x80U	//128 words
	#define UART0
//	#define UART1
#elif defined __AVR_ATmega64__
	#define SIG2	0x96
	#define SIG3	0x02
	#define PAGE_SIZE	0x80U	//128 words
	#define UART0
//	#define UART1
#elif defined __AVR_ATmega32__
	#define SIG2	0x95
	#define SIG3	0x02
	#define PAGE_SIZE	0x40U	//64 words
#elif defined __AVR_ATmega16__
	#define SIG2	0x94
	#define SIG3	0x03
	#define PAGE_SIZE	0x40U	//64 words
#elif defined __AVR_ATmega8__
	#define SIG2	0x93
	#define SIG3	0x07
	#define PAGE_SIZE	0x20U	//32 words
#elif defined __AVR_ATmega162__
	#define SIG2	0x94
	#define SIG3	0x04
	#define PAGE_SIZE	0x40U	//64 words
	#define UART0
//	#define UART1
#elif defined __AVR_ATmega163__
	#define SIG2	0x94
	#define SIG3	0x02
	#define PAGE_SIZE	0x40U	//64 words
#elif defined __AVR_ATmega169__
	#define SIG2	0x94
	#define SIG3	0x05
	#define PAGE_SIZE	0x40U	//64 words
#elif defined __AVR_ATmega8515__
	#define SIG2	0x93
	#define SIG3	0x06
	#define PAGE_SIZE	0x20U	//32 words
#elif defined __AVR_ATmega8535__
	#define SIG2	0x93
	#define SIG3	0x08
	#define PAGE_SIZE	0x20U	//32 words
#elif defined __AVR_AT90CAN128__
	#define SIG2	0x97
	#define SIG3	0x81
	#define PAGE_SIZE	0x80U	//128 words
	#define UART0
#elif defined __AVR_AT90CAN32__
	#define SIG2	0x95
	#define SIG3	0x81
	#define PAGE_SIZE	0x80U	//128 words
	#define UART0
#endif


void serout(char);
void serstr(char *str);
uint16_t serhexwordin(void);
void serhexwordout(uint16_t v);
void serhexbyteout(uint8_t v);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);


union address_union {
	uint16_t word;
	uint8_t  byte[2];
} address;

union length_union {
	uint16_t word;
	uint8_t  byte[2];
} length;

#if 0
struct flags_struct {
	unsigned eeprom : 1;
	unsigned rampz  : 1;
    unsigned did_program : 1;
} flags;
#endif

uint8_t did_program;

uint8_t buff[256];
uint8_t address_high;

uint8_t pagesz=0x80;

void (*app_reset)(void) = 0x0000;

void app_start(void)
{
    uint8_t m = MCUCR;
    MCUCR = m | (1<<IVCE);  /* interrupt vector change enable */
    MCUCR = m | (1<<IVSEL); /* Interrupt vector select = application */
    app_reset();
}

// CBUS3 is PG3
#define BL PING
#define BL_PIN PING3
 
/* LEDs on port F: 0,1,2 red,white,blue */
#define LED_PORT PORTF
#define LED_DDR  DDRF
#define LED_PIN  PINF
#define LED      PINF1

void led(uint8_t on)
{
    /* Turn on/off active low LED */
    LED_DDR |= (1<<LED);
	if (on)
		LED_PORT &= ~(1<<LED);
	else
	    LED_PORT |= (1<<LED);		
}

int main(void)
{
    uint8_t reset_cause = MCUSR;
    MCUSR = 0;
    wdt_disable();

    led(1);

    uint8_t ch,ch2;
    uint16_t w;
		        
	uint8_t bl_pin_asserted = (BL&(1<<BL_PIN));
	uint8_t flash_empty = (__LPM_enhanced__(0x0000) == 0xFF);
	
				
	asm volatile("nop\n\t");
	
	if (!flash_empty && !bl_pin_asserted)
	{
        app_start();	    
	}		

    UBRR0H = 0;
//    UBRR0L = 8; /* 115200 */
    UBRR0L = 25; /* 38400 */
    UCSR0A = 0;
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);

    serout('B');
    serout('O');
    serout('O');
    serout('T');
    serout('\r');
    serout('\n');


	serout('\0');    
        
 did_program = 0;
 
 for (;;) {     
   ch = getch();
	if(ch=='0') {		// Hello is anyone home?
		nothing_response();
	}
	else if(ch=='P') {		// Enter programming mode
		nothing_response();
	}
	else if(ch=='Q') {		// Leave programming mode
		nothing_response();

        if (did_program)
        {
            //serstr("Reset...\r\n");
            /* Force a reset */
            wdt_enable(WDTO_15MS);
            while(1)
                ;
        }
	}
	else if(ch=='U') {		//Set address, little endian. EEPROM in bytes, FLASH in words
							//Perhaps extra address bytes may be added in future to support > 128kB FLASH.
							//This might explain why little endian was used here, big endian used everywhere else.
		address.byte[0] = getch();
		address.byte[1] = getch();
		nothing_response();
	}
	else if(ch=='d') {		// Write memory, length is big endian and is in bytes
		length.byte[1] = getch();
		length.byte[0] = getch();
#if 0
		flags.eeprom = 0;
		if (getch() == 'E') flags.eeprom = 1;
#else
        getch();
#endif
		for (w=0;w<length.word;w++) {
		  buff[w] = getch();	// Store data in buffer, can't keep up with serial data stream whilst programming pages
		}
		if (getch() == ' ') {
#if 0
			if (flags.eeprom) {		//Write to EEPROM one byte at a time
				for(w=0;w<length.word;w++) {
					eeprom_write_byte((uint8_t *)address.word,buff[w]);
					address.word++;
				}			
			}
			else 
#endif
            {					//Write to FLASH one page at a time
				if (address.byte[1]>127) address_high = 0x01;	//Only possible with m128, m256 will need 3rd address byte. FIXME
				else address_high = 0x00;
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
				RAMPZ = address_high;
#endif
				address.word = address.word << 1;	//address * 2 -> byte location
//				if ((length.byte[0] & 0x01) == 0x01) length.word++;	//Even up an odd number of bytes
				if ((length.byte[0] & 0x01)) length.word++;	//Even up an odd number of bytes
				cli();									//Disable interrupts, just to be sure
				while(bit_is_set(EECR,EEWE));			//Wait for previous EEPROM writes to complete
				asm volatile("clr	r17				\n\t"	//page_word_count
							 "lds	r30,address		\n\t"	//Address of FLASH location (in bytes)
							 "lds	r31,address+1	\n\t"
							 "ldi	r28,lo8(buff)		\n\t"	//Start of buffer array in RAM
							 "ldi	r29,hi8(buff)		\n\t"
							 "lds	r24,length		\n\t"	//Length of data to be written (in bytes)
							 "lds	r25,length+1	\n\t"
							 "length_loop:			\n\t"	//Main loop, repeat for number of words in block							 							 
							 "cpi	r17,0x00	\n\t"	//If page_word_count=0 then erase page
							 "brne	no_page_erase	\n\t"						 
							 "wait_spm1:			\n\t"
							 "lds	r16,0x57		\n\t"	//Wait for previous spm to complete
							 "andi	r16,1\n\t"
							 "cpi	r16,1\n\t"
							 "breq	wait_spm1\n\t"
							 "ldi	r16,0x03		\n\t"	//Erase page pointed to by Z
							 "sts	0x57,r16		\n\t"
							 "spm					\n\t"							 
#ifdef __AVR_ATmega163__
	 						 ".word 0xFFFF					\n\t"
							 "nop							\n\t"
#endif
							 "wait_spm2:			\n\t"
							 "lds	r16,0x57		\n\t"	//Wait for previous spm to complete
							 "andi	r16,1\n\t"
							 "cpi	r16,1\n\t"
							 "breq	wait_spm2\n\t"									 

							 "ldi	r16,0x11				\n\t"	//Re-enable RWW section
					 		 "sts	0x57,r16				\n\t"						 			 
					 		 "spm							\n\t"
#ifdef __AVR_ATmega163__
	 						 ".word 0xFFFF					\n\t"
							 "nop							\n\t"
#endif
							 "no_page_erase:		\n\t"							 
							 "ld	r0,Y+			\n\t"		//Write 2 bytes into page buffer
							 "ld	r1,Y+			\n\t"							 
							 
							 "wait_spm3:			\n\t"
							 "lds	r16,0x57		\n\t"	//Wait for previous spm to complete
							 "andi	r16,1\n\t"
							 "cpi	r16,1\n\t"
							 "breq	wait_spm3\n\t"
							 "ldi	r16,0x01		\n\t"	//Load r0,r1 into FLASH page buffer
							 "sts	0x57,r16		\n\t"
							 "spm					\n\t"
							 
							 "inc	r17				\n\t"	//page_word_count++
							 "cpi r17,%0	\n\t"
							 "brlo	same_page		\n\t"	//Still same page in FLASH
							 "write_page:			\n\t"
							 "clr	r17				\n\t"	//New page, write current one first
							 "wait_spm4:			\n\t"
							 "lds	r16,0x57		\n\t"	//Wait for previous spm to complete
							 "andi	r16,1\n\t"
							 "cpi	r16,1\n\t"
							 "breq	wait_spm4\n\t"
#ifdef __AVR_ATmega163__
							 "andi	r30,0x80		\n\t"	// m163 requires Z6:Z1 to be zero during page write
#endif							 							 
							 "ldi	r16,0x05		\n\t"	//Write page pointed to by Z
							 "sts	0x57,r16		\n\t"
							 "spm					\n\t"
#ifdef __AVR_ATmega163__
	 						 ".word 0xFFFF					\n\t"
							 "nop							\n\t"
							 "ori	r30,0x7E		\n\t"		// recover Z6:Z1 state after page write (had to be zero during write)
#endif
							 "wait_spm5:			\n\t"
							 "lds	r16,0x57		\n\t"	//Wait for previous spm to complete
							 "andi	r16,1\n\t"
							 "cpi	r16,1\n\t"
							 "breq	wait_spm5\n\t"									 
							 "ldi	r16,0x11				\n\t"	//Re-enable RWW section
					 		 "sts	0x57,r16				\n\t"						 			 
					 		 "spm							\n\t"					 		 
#ifdef __AVR_ATmega163__
	 						 ".word 0xFFFF					\n\t"
							 "nop							\n\t"
#endif
							 "same_page:			\n\t"							 
							 "adiw	r30,2			\n\t"	//Next word in FLASH
							 "sbiw	r24,2			\n\t"	//length-2
							 "breq	final_write		\n\t"	//Finished
							 "rjmp	length_loop		\n\t"
							 "final_write:			\n\t"
							 "cpi	r17,0			\n\t"
							 "breq	block_done		\n\t"
							 "adiw	r24,2			\n\t"	//length+2, fool above check on length after short page write
							 "rjmp	write_page		\n\t"
							 "block_done:			\n\t"
							 "clr	__zero_reg__	\n\t"	//restore zero register
							 : : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r30","r31");

/* Should really add a wait for RWW section to be enabled, don't actually need it since we never */
/* exit the bootloader without a power cycle anyhow */
			}
			serout(0x14);
			serout(0x10);
		}
        did_program = 1;
	}
	else if(ch=='t') {		//Read memory block mode, length is big endian.
		length.byte[1] = getch();
		length.byte[0] = getch();
#if 0
		if (getch() == 'E') {
            flags.eeprom = 1;
        }
		else {
			flags.eeprom = 0;
		}
#else
        getch();
#endif
		if (getch() == ' ') {		// Command terminator
			serout(0x14);
			for (w=0;w < length.word;w++) {		// Can handle odd and even lengths okay
//				if (flags.eeprom) {	// Byte access EEPROM read
//					serout(eeprom_read_byte((uint8_t *)address.word));
//					address.word++;
//				}
//				else {
                    /* kday: address.word is the word address in flash. 
                     * elpm is required for >64kB flash (lpm doesn't use RAMPSZ).
                     */
                    uint8_t flv;
                    uint16_t fla;
                    fla = address.word;
                    fla <<= 1;
                    fla += w;
#if 0 // for at90can128
                    if (address.word > 0x7FFF) {
                        RAMPZ = 1;
                    }
                    else {
                        RAMPZ = 0;
                    }
                    asm("elpm %0, Z" : "=r"(flv) : "z"(fla));
                    serout(flv);
#else // at90can32
                    asm("lpm %0, Z" : "=r"(flv) : "z"(fla));
                    serout(flv);
#endif

//				}
			}
			serout(0x10);
		}
        did_program = 1; /* want to reset after verify also */
        /* Note!  For some reason if did_program is set before the flash reading/programming, and
         * not after, it gets cleared.  Not sure why, but there is a bug lurking... */
	}
  }
}


void serout(char ch)
{
#ifdef UART0
	while (!((UCSR0A) & _BV(UDRE0)));
	UDR0 = ch;
#elif defined UART1
	while (!((UCSR1A) & _BV(UDRE1)));
	UDR1 = ch;
#else		// m8,16,32,169,8515,8535,163
	while (!((UCSRA) & _BV(UDRE)));
	UDR = ch;
#endif
}

char getch(void)
{
	volatile uint32_t c = 0;
#ifdef UART0
	while(!((UCSR0A) & _BV(RXC0)))
	{
		static const uint32_t m = 0x1FFFF, n = 0x0FFFF;
		++c;
		if ((c & m) == 0)
		{
			led(1);
		}
		else if ((c & m) == n)
		{
			led(0);
		}		
	}
	return ((UDR0));
#elif defined UART1
	while(!((UCSR1A) & _BV(RXC1)));
	return ((UDR1));
#else		// m8,16,32,169,8515,8535,163
	while(!((UCSRA) & _BV(RXC)));
	return ((UDR));
#endif
}

void getNch(uint8_t count)
{
uint8_t i;
	for(i=0;i<count;i++) {
#ifdef UART0
		while(!((UCSR0A) & _BV(RXC0)));
		(UDR0);
#elif defined UART1
		while(!((UCSR1A) & _BV(RXC1)));
		(UDR1);
#else		// m8,16,32,169,8515,8535,163
		while(!((UCSRA) & _BV(RXC)));
		(UDR);
#endif		
	}
}

void byte_response(uint8_t val)
{
	if (getch() == ' ') {
		serout(0x14);
		serout(val);
		serout(0x10);
	}
}

void nothing_response(void)
{
	if (getch() == ' ') {
		serout(0x14);
		serout(0x10);
	}
}

