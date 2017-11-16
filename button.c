#include <avr/io.h>
#include <util/delay.h>

#define BLINK_DELAY_MS 1000

int main(void){
	/* set pin 5 of PORTB for output*/
	char val=0;
	DDRB |= _BV(DDB5);
	DDRB &= ~_BV(DDB4);
	while(1){
		/* set pin 5 high to turn led on */
		val= PINB & _BV(PORTB4);
		if(!val){
			PORTB |= _BV(PORTB5);
		}
		else{
			PORTB &= ~_BV(PORTB5);
		}
		//PORTB &= ~_BV(PORTB4);
		//_delay_ms(BLINK_DELAY_MS);
		/* set pin 5 low to turn led off */
		//PORTB &= ~_BV(PORTB5);
		//PORTB &= ~_BV(PORTB4);
		//_delay_ms(BLINK_DELAY_MS);
	}
}
