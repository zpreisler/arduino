#include <avr/io.h>
#include <util/delay.h>

inline void lcd_ddr_out(){
	DDRD |= 0b11100000; //rs,rw,enable output
	DDRB |= 0b00111100; //D4,D5,D6,D7 output
}
inline void lcd_ddrb_in(){
	DDRB &= 0b11000011; //D4,D5,D6,D7 input
}
inline void lcd_ddrb_out(){
	DDRB |= 0b00111100; //D4,D5,D6,D7 output
}
inline void lcd_cmd_high(){
	PORTD |= 0b11100000; //rs,rw,enable high
}
inline void lcd_cmd_low(){
	PORTD &= 0b00011111; //rs,rw,enable low
}
inline void lcd_data_high(){
	PORTB |= 0b00111100; //D4,D5,D6,D7 high
}
inline void lcd_data_low(){
	PORTB &= 0b11000011; //D4,D5,D6,d7 low
}
inline void lcd_rs_high(){
	PORTD |= 0b00100000; //rs high
}
inline void lcd_rs_low(){
	PORTD &= 0b11011111; //rs low
}
inline void lcd_rw_high(){
	PORTD |= 0b01000000; //rw high
}
#define LCD_RW_LOW PORTD &= 0b10111111
inline void lcd_rw_low(){
	PORTD &= 0b10111111; //rw low
}
#define LCD_ENABLE_HIGH PORTD |= 0b10000000
void lcd_enable_high(){
	PORTD |= 0b10000000; //enable high
}
#define LCD_ENABLE_LOW PORTD &= 0b01111111
inline void lcd_enable_low(){
	PORTD &= 0b01111111; //enable low
}
void lcd_init_pulse(){
	lcd_cmd_low();
	lcd_data_low();
	PORTB |= 0b00001100; // D4,D5 high
	lcd_enable_high();
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	lcd_enable_low();
}
void lcd_blk(){
	uint8_t blk;
	lcd_cmd_low();
	lcd_data_low();
	lcd_ddrb_in();
	lcd_rw_high();
	do{
		LCD_ENABLE_HIGH;
		//lcd_enable_high();
		blk = PINB & 0b00100000;
		LCD_ENABLE_LOW;
		//lcd_enable_low();
		//lcd_enable_high(); //second part of 4bit
		LCD_ENABLE_HIGH;
		LCD_ENABLE_LOW;
		//lcd_enable_low();
	}while(blk);
	lcd_ddrb_out();
}
void lcd_4bit(){
	lcd_blk();
	lcd_cmd_low();
	lcd_data_low();
	PORTB |= 0b00001000; // D5 high
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();
}
void lcd_function_set(){
	lcd_blk();
	lcd_cmd_low();
	lcd_data_low();

	PORTB |= 0b00001000; // D5 high
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();

	lcd_data_low();
	PORTB |= 0b00100000; // D7,D6 high
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();
}
void lcd_display_on(){
	lcd_blk();
	lcd_cmd_low();
	lcd_data_low();

	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();

	PORTB |= 0b00111100; // D7,D6,D5,D4 high
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();
}
void lcd_display_off(){
	lcd_blk();
	lcd_cmd_low();
	lcd_data_low();

	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();

	PORTB |= 0b00100000; // D7 high
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();
}
void lcd_clear(){
	lcd_blk();
	lcd_cmd_low();
	lcd_data_low();

	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();

	PORTB |= 0b00000100; // D4 high
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();
}
void lcd_entry_mode(){
	lcd_blk();
	lcd_cmd_low();
	lcd_data_low();

	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();

	PORTB |= 0b00011000;
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_high();
	//lcd_enable_low();
}
void lcd_16x2(){ //Call after initialization sequence
	lcd_function_set();
	lcd_display_on();
	lcd_clear();
	lcd_entry_mode();
}
void lcd_init(){
	_delay_ms(15);	//wait > 15ms
	lcd_init_pulse();
	_delay_ms(5); //wait > 4.1ms
	lcd_init_pulse();
	_delay_us(100); //wait > 100us
	lcd_init_pulse();
	lcd_4bit();
	lcd_16x2();
}
void lcd_print_char(char *c){
	uint8_t mask_high=0xF0;
	uint8_t mask_low=0x0F;
	lcd_blk();
	lcd_cmd_low();
	lcd_rs_high();

	lcd_data_low();
	PORTB |= ((uint8_t)*c & mask_high) >> 2;
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_low();

	lcd_data_low();
	PORTB |= ((uint8_t)*c & mask_low) << 2;
	LCD_ENABLE_HIGH;
	LCD_ENABLE_LOW;
	//lcd_enable_low();
}
int main(void){
	char c[]="Hello world!!!";
	lcd_ddr_out();
	lcd_init();
	lcd_print_char(c);
	lcd_print_char(c+1);
	lcd_print_char(c+2);
	lcd_print_char(c+3);
	lcd_print_char(c+4);
	lcd_print_char(c+5);
	lcd_print_char(c+6);
	lcd_print_char(c+7);
	lcd_print_char(c+8);
	lcd_print_char(c+9);
	lcd_print_char(c+10);
	lcd_print_char(c+11);
	lcd_print_char(c+12);
	lcd_print_char(c+13);
}
