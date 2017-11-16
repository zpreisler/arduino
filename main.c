#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define LCD_ENABLE_HIGH PORTD |= 0b10000000
#define LCD_ENABLE_LOW PORTD &= 0b01111111

#define LCD_CMD_HIGH PORTD |= 0b11100000
#define LCD_CMD_LOW PORTD &= 0b00011111
#define LCD_DATA_HIGH PORTB |= 0b00111100
#define LCD_DATA_LOW PORTB &= 0b11000011

#define LCD_RS_HIGH PORTD |= 0b00100000
#define LCD_RS_LOW PORTD &= 0b11011111
#define LCD_RW_HIGH PORTD |= 0b01000000
#define LCD_RW_LOW PORTD &= 0b10111111

#define LCD_DDRD_OUTPUT DDRD |= 0b11100000
#define LCD_DDRB_INPUT DDRB &= 0b11000011
#define LCD_DDRB_OUTPUT DDRB |= 0b00111100

#define DHT11_OUTPUT DDRD |= 0b00010000
#define DHT11_INPUT DDRD &= 0b11101111

#define DHT11_HIGH PORTD |= 0b00010000
#define DHT11_LOW PORTD &= 0b11101111

#define DS1307READ	0xD1
#define DS1307WRITE	0xD0

#define HC595_OUTPUT DDRB |= 0b00000011; DDRD |= 0b00001000;
#define HC595_INPUT DDRB &= 0b11111100; DDRD &= 0b11110111;

#define HC595_DS_HIGH PORTD |= 0b00001000
#define HC595_DS_LOW PORTD &= 0b11110111

#define HC595_SHIFT_HIGH PORTB |= 0b00000010
#define HC595_SHIFT_LOW PORTB &= 0b11111101

#define HC595_LATCH_HIGH PORTB |= 0b00000001
#define HC595_LATCH_LOW PORTB &= 0b11111110

inline void lcd_ddr_out(){
	LCD_DDRD_OUTPUT;
	LCD_DDRB_OUTPUT;
}
void lcd_pulse(){
	LCD_ENABLE_HIGH;
	_delay_us(1);
	LCD_ENABLE_LOW;
}
void lcd_init_pulse(){
	LCD_CMD_LOW;
	LCD_DATA_LOW;
	PORTB |= 0b00001100; // D4,D5 high
	lcd_pulse();
}
void lcd_blk(){
	uint8_t blk;
	LCD_CMD_LOW;
	LCD_DATA_LOW;
	LCD_DDRB_INPUT;
	LCD_RW_HIGH;
	do{
		LCD_ENABLE_HIGH;
		_delay_us(1);
		blk = PINB & 0b00100000;
		LCD_ENABLE_LOW;

		lcd_pulse();
	}while(blk);
	LCD_DDRB_OUTPUT;
}
void lcd_4bit(){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;
	PORTB |= 0b00001000; // D5 high
	lcd_pulse();
}
void lcd_function_set(){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;

	PORTB |= 0b00001000; // D5 high
	lcd_pulse();

	LCD_DATA_LOW;
	PORTB |= 0b00100000; // D7,D6 high
	lcd_pulse();
}
void lcd_display_on(){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;

	lcd_pulse();

	PORTB |= 0b00111100; // D7,D6,D5,D4 high
	lcd_pulse();
}
void lcd_display_off(){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;

	lcd_pulse();

	PORTB |= 0b00100000; // D7 high
	lcd_pulse();
}
void lcd_clear(){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;

	lcd_pulse();

	PORTB |= 0b00000100; // D4 high
	lcd_pulse();
}
void lcd_entry_mode(){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;

	lcd_pulse();

	PORTB |= 0b00011000;
	lcd_pulse();
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
void lcd_set_cursor(uint8_t address){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_DATA_LOW;
	PORTB |= 0b00100000 | ((address & 0xF0) >> 2);
	lcd_pulse();

	LCD_DATA_LOW;
	PORTB |=  (address & 0x0F) << 2;
	lcd_pulse();
}
void lcd_print_char(const char *c){
	lcd_blk();
	LCD_CMD_LOW;
	LCD_RS_HIGH;

	LCD_DATA_LOW;
	PORTB |= ((uint8_t)*c & 0xF0) >> 2;
	lcd_pulse();

	LCD_DATA_LOW;
	PORTB |= ((uint8_t)*c & 0x0F) << 2;
	lcd_pulse();
}
void lcd_print(const char *c){
	while(*c){
		lcd_print_char(c);
		c++;
	}
}
void dht11_start(){
	DHT11_OUTPUT;
	DHT11_LOW;
	_delay_ms(18);
	DHT11_HIGH;
	_delay_us(30);
	DHT11_INPUT;
}
void dht11_end(){
	DHT11_OUTPUT;
	DHT11_HIGH;
}
void dht11_pulse_length(uint8_t *length,uint8_t *s){
	uint8_t state=0,last_state=0;
	TCNT0=0;
	TCCR0B |= (1 << CS01);
	last_state = PIND & 0b00010000;
	do{
		state = PIND & 0b00010000;	
	}while(state==last_state);
	TCCR0B=0;
	*length=TCNT0;
	*s=last_state;
}
uint8_t dht11_read_bit(){
	uint8_t state=0,last_state=0;
	TCNT0=0;
	do{
		last_state = PIND & 0b00010000;
	}while(!last_state);
	TCCR0B |= (1 << CS01);
	do{
		state = PIND & 0b00010000;	
	}while(state==last_state);
	TCCR0B=0;
	return TCNT0;
}
void dht11_read(uint8_t *d){
	uint8_t i,idx=0,byte=0,mask=128;
	uint8_t out,out0;
	char c[16];
	dht11_start();
	out0=dht11_read_bit();
	for(i=0;i<40;i++){
		//if(dht11_read_bit()>70){//short or long pulse
		out=dht11_read_bit();//short or long pulse
		if(out>100){//short or long pulse
			byte |= mask;
		}
		mask >>= 1;
		if(!mask){
			mask=128;
			*(d+idx++)=byte;
			byte=0;
		}
	}
	dht11_end();
	//lcd_set_cursor(0x0d);
	//lcd_print(itoa(out0,c,10));
	//lcd_set_cursor(0x4d);
	//lcd_print(itoa(out,c,10));
}
void dht11_print(uint8_t *d){
	char c[16];
	uint16_t a,b;
	a= ((uint16_t)d[0] << 8 ) | (uint16_t)d[1];
	b= ((uint16_t)d[2] << 8 ) | (uint16_t)d[3];
	//b=(uint16_t)(*(d+2));
	lcd_set_cursor(0x00);
	//lcd_print(itoa(d[2],c,2));
	lcd_print(itoa(a,c,10));
	lcd_set_cursor(0x40);
	lcd_print(itoa(b,c,10));
	//lcd_print(itoa(d[3],c,2));

	//lcd_set_cursor(0x08);
	//lcd_print(itoa(d[0],c,2));
	//lcd_set_cursor(0x48);
	//lcd_print(itoa(d[1],c,2));

	//lcd_set_cursor(0x0d);
	//lcd_print(itoa(d[4],c,10));
	//lcd_set_cursor(0x4d);
	//lcd_print(itoa(d[0]+d[1]+d[2]+d[3],c,10));
}
void i2c_init(){
	TWBR=2;	//100khz
	TWSR |= ((1<<TWPS1)|(1<<TWPS0));
	TWCR |= (1<<TWEN);
}
void i2c_start(){
	TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));
	while (!(TWCR & (1<<TWINT)));
}
void i2c_stop(){
	TWCR = ((1<<TWINT) | (1<<TWEN) | (1<<TWSTO));
}
void i2c_write(uint8_t byte){
	TWDR = byte;
	TWCR = ((1<< TWINT) | (1<<TWEN));
	while (!(TWCR & (1 <<TWINT)));
}
uint8_t i2c_read(uint8_t ack){
	TWCR = ((1<< TWINT) | (1<<TWEN) | (ack<<TWEA));
	while ( !(TWCR & (1 <<TWINT)));
	return TWDR;
}
void ds1307_init(){
	i2c_init();
	i2c_start();
	i2c_write(DS1307WRITE);
	i2c_write(0x07);
	i2c_write(0x00);
	i2c_stop();
}
void ds1307_settime(uint8_t *t){
	i2c_start();
	i2c_write(DS1307WRITE);
	i2c_write(0x00);
	i2c_write(*t); //sec
	i2c_write(*(t+1)); //min
	i2c_write(*(t+2)); //hours
	i2c_write(*(t+3)); //day
	i2c_write(*(t+4)); //date
	i2c_write(*(t+5)); //month
	i2c_write(*(t+6)); //year
	i2c_stop();
}
void ds1307_gettime(uint8_t *t){
	i2c_start();
	i2c_write(DS1307WRITE);
	i2c_write(0x00); //set read address
	i2c_start();
	i2c_write(DS1307READ);
	*t=i2c_read(1); //sec
	*(t+1)=i2c_read(1); //min
	*(t+2)=i2c_read(1); //hours
	*(t+3)=i2c_read(1); //day
	*(t+4)=i2c_read(1); //date
	*(t+5)=i2c_read(1); //month
	*(t+6)=i2c_read(0); //year
	i2c_stop();
}
void ds1307_print_time(uint8_t *t){
	char c[2];
	lcd_print(itoa(t[2]>>4,c,10));
	lcd_print(itoa(t[2]&0x0F,c,10));
	lcd_print_char(":");
	lcd_print(itoa(t[1]>>4,c,10));
	lcd_print(itoa(t[1]&0x0F,c,10));
	lcd_print_char(":");
	lcd_print(itoa(t[0]>>4,c,10));
	lcd_print(itoa(t[0]&0x0F,c,10));
}
void ds1307_print_date(uint8_t *t){
	char c[2],d;
	d=(t[4]>>4)+'0';
	lcd_print_char(&d);
	d=(t[4]&0x0F)+'0';
	lcd_print_char(&d);
	lcd_print_char("/");
	lcd_print(itoa(t[5]>>4,c,10));
	lcd_print(itoa(t[5]&0x0F,c,10));
	lcd_print_char("/");
	lcd_print(itoa(t[6]>>4,c,10));
	lcd_print(itoa(t[6]&0x0F,c,10));
}
void shift_init(){
	HC595_OUTPUT;
	HC595_DS_LOW;
}
void shift_write(uint8_t data){
	uint8_t i;
	HC595_SHIFT_LOW;
	HC595_LATCH_LOW;
	for(i=0;i<8;i++){
		if(data & 0b10000000){
			HC595_DS_HIGH;
		}
		else{
			HC595_DS_LOW;
		}
		HC595_SHIFT_HIGH;
		data=data<<1;
		HC595_SHIFT_LOW;
	}
	HC595_LATCH_HIGH;
}
void ADC_init(){
 // Select Vref=AVcc
 ADMUX |= (1<<REFS0);
 //set prescaller to 128 and enable ADC
 ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}
uint16_t read_ADC(uint8_t channel_ADC){
 //select ADC channel with safety mask
 //ADMUX = (ADMUX & 0xF0) | (channel_ADC & 0x0F);
 ADMUX = (ADMUX & 0xF0) | (channel_ADC);
 //single conversion mode
 ADCSRA |= (1<<ADSC);
 // wait until ADC conversion is complete
 while( ADCSRA & (1<<ADSC) );
 return ADC;
}
int main(void){
	uint8_t data[5];
	uint8_t timedate[7]={0x0,0x05,0x18,0x00,0x10,0x06,0x16};
	_delay_ms(100);
	lcd_ddr_out();
	lcd_init();
	ADC_init();
	
	//dht11_read(data);
	//dht11_print(data);
	//lcd_set_cursor(0x4d);
	lcd_set_cursor(0x0);

	lcd_clear();
	lcd_print("Hello World!!");
	ds1307_init();
	_delay_ms(2000);

	lcd_set_cursor(0x0);
	//dht11_read(data);
	//dht11_print(data);
	_delay_ms(2000);

	//ds1307_gettime(timedate);
	lcd_set_cursor(0x0);
	//ds1307_print_time(timedate);
	//ds1307_settime(timedate);
	//shift_init();
	//shift_write(0b11111111);
	
	lcd_clear();
	uint16_t a0=0;
	char c[16];
	for(;;){

		//ds1307_gettime(timedate);
		//lcd_set_cursor(0x03);
		//ds1307_print_time(timedate);

		//lcd_set_cursor(0x43);
		//ds1307_print_date(timedate);

		_delay_ms(2000);

		lcd_clear();
		dht11_read(data);
		dht11_print(data);

		a0=read_ADC(0);
		lcd_set_cursor(0x08);
		lcd_print(itoa(a0,c,10));
		_delay_ms(2000);

	}
}
