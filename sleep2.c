/*Copyright (c) 2010, Mariano Alvira <mar@devl.org> and other contributors
 * to the MC1322x project (http://mc1322x.devl.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of libmc1322x: see http://mc1322x.devl.org
 * for details. 
 *
 *
 */

#include <mc1322x.h>
#include <board.h>

#include "tests.h"
#include "config.h"

volatile char rupt = 0;

void adc_isr(void)
{
	if((*CRM_WU_CNTL&0x8)==0x8)
	{
		*CRM_SLEEP_CNTL |= 0x1; //this is magic, don't know why it works, just needs to be here!!! 
	}
	
	if(((ADC->IRQ)&0x1000) == 0x1000)
	{
	    *CRM_WU_CNTL &= ~(0x8);
		uint16_t trig = ADC->TRIGGERS;
		rupt = ((trig&0x0008)==0x0008);
	}

	ADC->IRQ = 0xf000;
}

void main(void) {

	uart_init(UART1, 115200);

	*mem32(0x00401ffc) = 0x01234567;
	*mem32(0x00407ffc) = 0xdeadbeef;
	*mem32(0x0040fffc) = 0xface00ff;
	*mem32(0x00410000) = 0xabcd0123;



	putstr("sleep test\n\r");
	putstr("0x00401ffc: ");
	put_hex32(*mem32(0x00401ffc));
	putstr("\r\n");
	putstr("0x00407ffc: ");
	put_hex32(*mem32(0x00407ffc));
	putstr("\r\n");
	putstr("0x0040fffc: ");
	put_hex32(*mem32(0x0040fffc));
	putstr("\r\n");
	putstr("0x00410000: ");
	put_hex32(*mem32(0x00410000));
	putstr("\r\n");


GPIO->FUNC_SEL.ADC3 = 1;
GPIO->PAD_DIR.ADC3 = 0;
GPIO->PAD_KEEP.ADC3 = 0;
GPIO->PAD_PU_EN.ADC3 = 0;

adc_init();
ADC->COMP_3=	0x38ff;	//use channel 3, compare on 0ff
ADC->SEQ_1 =	0x8;	//only channel 3 is enabled
ADC->CONTROL &= ~0xe000;
	/* radio must be OFF before sleeping */
	/* otherwise MCU will not wake up properly */
	/* this is undocumented behavior */
//	radio_off();

#if USE_32KHZ
	/* turn on the 32kHz crystal */
	putstr("enabling 32kHz crystal\n\r");
	/* you have to hold it's hand with this on */
	/* once you start the 32xHz crystal it can only be stopped with a reset (hard or soft) */
	/* first, disable the ring osc */
	clear_bit(*CRM_RINGOSC_CNTL,0);
	/* enable the 32kHZ crystal */
	set_bit(*CRM_XTAL32_CNTL,0);

	/* set the XTAL32_EXISTS bit */
	/* the datasheet says to do this after you've check that RTC_COUNT is changing */
	/* the datasheet is not correct */
	set_bit(*CRM_SYS_CNTL,5);
	{
		static volatile uint32_t old;
		old = *CRM_RTC_COUNT;
		putstr("waiting for xtal\n\r");
		while(*CRM_RTC_COUNT == old) { 
			continue; 
		}
		/* RTC has started up */

		set_bit(*CRM_SYS_CNTL,5);
		putstr("32kHZ xtal started\n\r");

	}
#endif	
		

	/* go to sleep */
	*CRM_SLEEP_CNTL = 0x70;
//	*CRM_WU_CNTL = 0; /* don't wake up */
	*CRM_WU_CNTL = 0xa; /* enable wakeup from wakeup timer, rtc for sampling, and auto adc */
	*CRM_WU_CNTL &= ~0x1; //disable wakeup timer
//	*CRM_WU_TIMEOUT = 1875000; /* wake 10 sec later if doze */
#if USE_32KHZ
//	*CRM_WU_TIMEOUT = 327680*2;
#else
//	*CRM_WU_TIMEOUT =	200000; /* wake 10 sec later if hibernate ring osc */
#endif

	*CRM_RTC_TIMEOUT=	200; //sample 10 times a second
enable_irq(ADC);
	/* hobby board: 2kHz = 11uA; 32kHz = 11uA */
//	*CRM_SLEEP_CNTL = 1; /* hibernate, RAM page 0 only, don't retain state, don't power GPIO */ /* approx. 2kHz = 2.0uA */
	/* hobby board: 2kHz = 18uA; 32kHz = 19uA */
//	*CRM_SLEEP_CNTL = 0x41; /* hibernate, RAM page 0 only, retain state, don't power GPIO */ /* approx. 2kHz = 10.0uA */
	/* hobby board: 2kHz = 20uA; 32kHz = 21uA */
//	*CRM_SLEEP_CNTL = 0x51; /* hibernate, RAM page 0&1 only, retain state, don't power GPIO */ /* approx. 2kHz = 11.7uA */
	/* hobby board: 2kHz = 22uA; 32kHz = 22.5uA */
//	*CRM_SLEEP_CNTL = 0x61; /* hibernate, RAM page 0,1,2 only, retain state, don't power GPIO */ /* approx. 2kHz = 13.9uA */
	/* hobby board: 2kHz = 24uA; 32kHz = 25uA */
	*CRM_SLEEP_CNTL = 0x71; /* hibernate, all RAM pages, retain state, don't power GPIO */ /* approx. 2kHz = 16.1uA */
//	*CRM_SLEEP_CNTL = 0xf1; /* hibernate, all RAM pages, retain state,       power GPIO */ /* consumption depends on GPIO hookup */

//	*CRM_SLEEP_CNTL = 2; /* doze     , RAM page 0 only, don't retain state, don't power GPIO */ /* approx. 69.2 uA */
//	*CRM_SLEEP_CNTL = 0x42; /* doze     , RAM page 0 only, retain state, don't power GPIO */ /* approx. 77.3uA */
//	*CRM_SLEEP_CNTL = 0x52; /* doze     , RAM page 0&1 only, retain state, don't power GPIO */ /* approx. 78.9uA */
//	*CRM_SLEEP_CNTL = 0x62; /* doze     , RAM page 0,1,2 only, retain state, don't power GPIO */ /* approx. 81.2uA */
//	*CRM_SLEEP_CNTL = 0x72; /* doze     , all RAM pages, retain state, don't power GPIO */ /* approx. 83.4uA - possibly with periodic refresh*/
//	*CRM_SLEEP_CNTL = 0xf2; /* doze     , all RAM pages, retain state,       power GPIO */ /* consumption depends on GPIO hookup */


	/* wait for the sleep cycle to complete */
	while((*CRM_STATUS & 0x1) == 0) { continue; }
	/* write 1 to sleep_sync --- this clears the bit (it's a r1wc bit) and powers down */
	*CRM_STATUS = 1; 
	
	/* asleep */

	/* wait for the awake cycle to complete */
	while((*CRM_STATUS & 0x1) == 0) { continue; }
	/* write 1 to sleep_sync --- this clears the bit (it's a r1wc bit) and finishes wakeup */
	*CRM_STATUS = 1; 

	putstr("\n\r\n\r\n\r");
	putstr("0x00401ffc: ");
	put_hex32(*mem32(0x00401ffc));
	putstr("\r\n");
	putstr("0x00407ffc: ");
	put_hex32(*mem32(0x00407ffc));
	putstr("\r\n");
	putstr("0x0040fffc: ");
	put_hex32(*mem32(0x0040fffc));
	putstr("\r\n");
	putstr("0x00410000: ");
	put_hex32(*mem32(0x00410000));
	putstr("\r\n");

	*GPIO_PAD_DIR0 = LED_RED;
#define DELAY 400000
	GPIO->FUNC_SEL.TXON = 3;
	GPIO->PAD_DIR_SET.TXON=1;
//uint32_t read;
	volatile uint32_t i;
	while(1) {
		gpio_set(TXON);
		
		for(i=0; i<DELAY; i++) { continue; }

		gpio_reset(TXON);

		for(i=0; i<DELAY; i++) { continue; }
		if(rupt==1)
		{
			putstr("adc interrupt! \r\n");
			rupt=0;
		}
	};
}

