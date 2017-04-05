/*
 * test2.c
 *
 * Created: 3/20/2016 6:09:06 PM
 * Author : john
 */ 

#define F_CPU 16000000
#define SCL_frequentie 100000
#define BAUDRATE		9600
#define UBBR 103 //207 for asynchronous communication
//#define UBRR_BAUD	(((long)F_CPU/((long)16 * BAUDRATE))-1)

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define DEVICE_ADRES   8
#define RP6 DEVICE_ADRES
#define KOMPAS 0x60

void verzenden(uint8_t ad,uint8_t b);
void ontvangen(uint8_t ad,uint8_t[],uint8_t);
void init_master();

void initUSART();
void writeChar(char c);
void writeInteger(int16_t number, uint8_t base);
void writeString(char *string);

void Input(char in);
int ping();
void turnTo(int hoek);
int leesKompas();
char *windrichting(int hoek);
void gaNaarHoekR(int hoek);
void gaNaarHoekL(int hoek);

int main(void)
{
	PORTD = 0x03; //pullup SDA en SCL
	initUSART();
    init_master();
	sei();

	//gaNaarHoekR(90);
	while (1)
	{
		verzenden(RP6, 30);
		_delay_ms(100);
	}

}

void init_master() {
	TWSR = 0;
	// Set bit rate
	TWBR = ( ( F_CPU / SCL_frequentie ) - 16) / 2;
	TWCR = (1<<TWEN);
}

void ontvangen(uint8_t ad,uint8_t b[],uint8_t max) {
	uint8_t op[15];
	
	TWCR |= (1<<TWSTA);
	while(!(TWCR & (1<<TWINT)));
	op[0] = TWSR;

	TWDR=(ad<<1)+1;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	op[1] = TWSR;
	b[0]=TWDR;
	
	uint8_t tel=0;
	do{
		if(tel == max-1)
		TWCR=(1<<TWINT)|(1<<TWEN);
		else
		TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
		while(!(TWCR & (1<<TWINT)));
		op[tel] = TWSR;
		b[tel]=TWDR;
	}while(op[tel++] == 0x50);

	TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);

	//   for(uint8_t i=0;i<tel;++i) {
	//	 writeString("\n\r");writeInteger(op[i],16);
	//	 writeString(" data ");writeInteger(b[i],10);
	//   }

}

void verzenden(uint8_t ad,uint8_t b) {
	//  uint8_t op[5];

	TWCR |= (1<<TWSTA);
	while(!(TWCR & (1<<TWINT)));
	//   op[0] = TWSR;
	TWDR=(ad<<1);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	//    op[1] = TWSR;

	TWDR=257;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	//  op[2] = TWSR;


	TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	//	while(!(TWCR & (1<<TWINT)));
	//  for(uint8_t i=0;i<3;++i) {
	// writeString("\n\r");writeInteger(op[0],16);
	// writeString(" ");writeInteger(op[1],16);
	// writeString(" ");writeInteger(op[2],16);
}


void initUSART() {
	UBRR0H = (unsigned char)(UBBR>>8);
	UBRR0L = (unsigned char)UBBR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);	/* Enable receiver and transmitter */
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01); /* Set frame format: 8data, 2stop bit */
}

void writeChar(char ch)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = (uint8_t)ch;
}

void writeString(char *string)
{
	while(*string)
	writeChar(*string++);
}

void writeInteger(int16_t number, uint8_t base)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	writeString(&buffer[0]);
}

ISR(USART0_RX_vect){
	Input(UDR0);
}

void Input(char in){
	switch(in){
		case 'w':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 1);
		break;
		case 's':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 2);
		break;
		case 'a':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 3);
		break;
		case 'd':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 4);
		break;
		case '+':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 5);
		break;
		case '-':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 6);
		break;
		case 'q':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 7);
		break;
		case 'e':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 8);
		break;
		case '0':
		writeChar('\t');writeChar(in);writeChar('\r');
		verzenden(DEVICE_ADRES, 0);
		break;
		case '4':
		writeChar('\t');writeChar(in);writeChar('\r');
		gaNaarHoekL(180);
		break;
		case '6':
		writeChar('\t');writeChar(in);writeChar('\r');
		gaNaarHoekR(180);
		break;
	}
}

int ping(){
	//mega pin 48
	int x = 0;
	DDRL = 0xFF;     // all output
	_delay_us(2);
	PORTL = (1 << PINL1);  //pin PL1 (48 on arduino) high
	_delay_us(5);    //delay of 5 us
	PORTL = ~(1 << PINL1);  //4th pin low
	DDRL = 0x00;     // all input
	//wacht op pin input van ultra
	while (!(PINL & (1 << PINL1))){
		_delay_us(0.5);
	}
	//ga tellen
	while (PINL & (1 << PINL1)){
		x++;
		_delay_us(0.5);
	}
	//us naar cm een cm is 29us een gaat hen en weer
	x = x / 58;
	return x;
}

int leesKompas(){
	uint16_t hoek;
	uint8_t b[10];
	verzenden(0x60, 2);
	ontvangen(0x60, b, 2);
	hoek = ((b[0]<<8)+b[1])/10;
	return hoek;
}

char *windrichting(int hoek){
	char *r;
	if (hoek < 45)
	{
		r = "Noord";
	}
	else if (hoek >= 45 && hoek <135)
	{
		r = "Oost";
	}
	else if (hoek>=135 && hoek <225){
		r ="Zuid";
	}
	else if (hoek>= 225 && hoek<315){
		r = "West";
	}
	else if(hoek >= 315){
		r = "Noord";
	}
	else{
		r = "ERR";
	}

	return r;
}

void gaNaarHoekR(int hoek){
	int huidig = leesKompas();
	int bestem = huidig + hoek;
	if (bestem < 359)
	{
		while(leesKompas() < bestem){
			verzenden(RP6, 14);
			_delay_ms(10);
		}
		verzenden(RP6, 0);
	}
	else{
		while(leesKompas() > (bestem -360)){
			verzenden(RP6, 14);
			_delay_ms(10);
		}
		while(leesKompas() < (bestem-360)){
			verzenden(RP6, 14);
			_delay_ms(10);
		}
		verzenden(RP6, 0);
	}
}

void gaNaarHoekL(int hoek){
	int huidig = leesKompas();
	int bestem = huidig - hoek;
	if (bestem > 0)
	{
		while(leesKompas() > bestem){
			verzenden(RP6, 13);
			_delay_ms(10);
		}
		verzenden(RP6, 0);
	}
	else{
		while(leesKompas() < (bestem+360)){
			verzenden(RP6, 13);
			_delay_ms(10);
		}
		while(leesKompas() > (bestem +360)){
			verzenden(RP6, 13);
			_delay_ms(10);
		}
		verzenden(RP6, 0);
	}
}