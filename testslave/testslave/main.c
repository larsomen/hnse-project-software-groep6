/*
 * i2cw.c
 *
 * Created: 3/19/2016 2:14:44 PM
 * Author : john
 */ 

 #define F_CPU 8000000
 #define SCL_frequentie 100000
 #define BAUDRATE		38400
 #define UBRR_BAUD	(((long)F_CPU/((long)16 * BAUDRATE))-1)
 #define resetData()  for(uint8_t i=0;i<20;++i) data[i]=0
 #define rset() for(uint8_t i=0;i<20;++i) data_ont[i]=0

 #define UBAT 			(1 << PINA7) // ADC7 (Input)
 #define MCURRENT_L 		(1 << PINA6) // ADC6 (Input)
 #define MCURRENT_R 		(1 << PINA5) // ADC5 (Input)
 #define E_INT1 			(1 << PINA4) // INT1 (input per default... can be output)
 #define LS_L 			(1 << PINA3) // ADC3 (Input)
 #define LS_R 			(1 << PINA2) // ADC2 (Input)
 #define ADC1 			(1 << PINA1) // ADC1 (Input)
 #define ADC0 			(1 << PINA0) // ADC0 (Input)

 #define SL4 		(1 << PINB7)	// Output
 #define ACS_L 		(1 << PINB6)	// Output
 #define START 		(1 << PINB5)	// Input
 #define PWRON		(1 << PINB4)	// Output
 #define ACS_PWRH	(1 << PINB3)	// Output
 #define ACS 		(1 << PINB2)	// INT2 (Input)
 #define SL5 		(1 << PINB1)	// Output
 #define SL6 		(1 << PINB0)	// Output


 #define ACS_R 		(1 << PINC7)	// Output
 #define SL3 		(1 << PINC6)	// Output
 #define SL2 		(1 << PINC5)	// Output
 #define SL1 		(1 << PINC4)	// Output
 #define DIR_R 		(1 << PINC3)	// Output
 #define DIR_L 		(1 << PINC2)	// Output
 #define SDA 		(1 << PINC1)	// I2C Data (I/O)
 #define SCL 		(1 << PINC0)	// I2C Clock (Output (Master), Input (Slave))


 #define ACS_PWR		(1 << PIND6)	// Output
 #define MOTOR_R		(1 << PIND5)	// PWM Output (OC1A)
 #define MOTOR_L		(1 << PIND4)	// PWM Output (OC1B)
 #define ENC_R 		(1 << PIND3)	// INT1 (Input)
 #define ENC_L 		(1 << PIND2)	// INT0 (Input)
 #define TX 		(1 << PIND1)	// USART TX (Output)
 #define RX 		(1 << PIND0)	// USART RX (Input)

#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>

#define TRUE 0xFF;
#define FALSE 0;

#define MaxSpeed 150
#define MinSpeed 0

void init_i2c_slave(uint8_t ad);
void init_i2c_ontvang( void (*ontvanger) (uint8_t [],uint8_t));
void init_i2c_verzend( uint8_t (*verzender) ());
void slaaftwi();
void (*ontfunc) (uint8_t[],uint8_t);
uint8_t (*verfunc) ();

void initUSART();
void writeInteger(int16_t number, uint8_t base);
void writeString(char *string);
void writeChar(char ch);

void motorIint(void);
void control(int inByte);
void act(uint8_t speedL, uint8_t speedR, uint8_t links, uint8_t rechts);


ISR(TWI_vect) {

	slaaftwi();

}

uint8_t data_ont[20]; //max 20
volatile uint8_t data_flag = FALSE;
volatile uint8_t databyte=0x33;

void ontvangData(uint8_t [],uint8_t);
uint8_t verzendByte();

uint16_t encoder_L_count;
uint16_t encoder_R_count;
void initEncoders(void);
int encoder_Count_to_Centimeter(int encoder_count);

uint16_t encoder_L_driven;
uint16_t encoder_R_driven;
uint16_t encoder_L_total;
uint16_t encoder_R_total;


int main(void)
{

	//DDRC=0xFF;
	initUSART();
	init_i2c_slave(8);
	
	/*ontvangData is de functie die uitgevoerd wordt 
	wanneer een byte via de i2c bus ontvangen wordt
	*/
	init_i2c_ontvang(ontvangData); 
	
	/*verzendByte is de functie die aangeroepen wordt
	wanneer de slave een byte naar de master verzend*/
	init_i2c_verzend(verzendByte);

	motorIint();
	initEncoders();
	sei(); //De slave van i2c werkt met interrupt
	
    /* Replace with your application code */
	int drivenL;
    while (1) 
    {  
		control(data_ont[0]);
		data_ont[0] = 255;
		_delay_ms(10);
    }
	
}
 /*slave heeft data ontvangen van de master
 data[] een array waarin de ontvangen data staat
 tel het aantal bytes dat ontvangen is*/
 
void ontvangData(uint8_t data[],uint8_t tel){
	for(int i=0;i<tel;++i)
	    data_ont[i]=data[i];
	data_flag = TRUE;
	//writeString("o\n\r");
}

/* het byte dat de slave verzend naar de master
in dit voorbeeld een eenvoudige teller
*/

uint8_t verzendByte() {
		return encoder_Count_to_Centimeter(encoder_L_driven);
}

void init_i2c_slave(uint8_t ad) {
	
	TWSR = 0;
	TWBR = ((F_CPU / SCL_frequentie) - 16) / 2;
	TWCR = (1 << TWIE) | (1 << TWEN) | (1 << TWEA);
	TWAR = ad<<1;
}

void slaaftwi() {
	static uint8_t data[40];
	static uint8_t teller=0;
	switch(TWSR) {
		case 0x10:
		case 0x08:
		break;
		
		case 0x60:

		teller=0;

		break;
		case 0x68:

		break;
		case 0x80:
		data[teller++] = TWDR;
		break;

		case 0xA0:
		ontfunc(data,teller);
		resetData();
		break;

		case 0xA8:
		teller=0;
		TWDR=verfunc();
		break;

		case 0xB8:
		TWDR=verfunc();
		break;

		case 0xC0:   //NACK
		break;
		case 0xC8:
		break;
	}
	TWCR |= (1<<TWINT);    // Clear TWINT Flag
}

void init_i2c_ontvang( void (*ontvanger) (uint8_t [],uint8_t)) {
	ontfunc=ontvanger;
}

void init_i2c_verzend( uint8_t (*verzender) ()) {
	verfunc=verzender;
}

void initUSART() {

	UBRRH = UBRR_BAUD >> 8;
	UBRRL = (uint8_t) UBRR_BAUD;
	UCSRA = 0x00;
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	UCSRB = (1 << TXEN) | (1 << RXEN);
	writeString("usart werkt nog\n\r");
}

void writeChar(char ch)
{
	while (!(UCSRA & (1<<UDRE)));
	UDR = (uint8_t)ch;
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

void motorIint(void){
	DDRC |= (1<<2)|(1<<3);

	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);
	ICR1 = 210; //is max in plaats van 255 (volgens datashets)
	OCR1A = 0;
	OCR1B = 0;
	//start timers
	TCCR1B = (1<<WGM12)|(1<<CS10);
}

void control(int inByte){
	static uint8_t speedL = 0, speedR = 0;
	static uint8_t directionL = 0, directionR = 0;
	if(inByte == 1){
		directionL = 0;
		directionR = 0;
		speedL = speedR;
	}
	if (inByte == 2){
		directionL = 1;
		directionR = 1;
		speedL = speedR;
	}

	if(inByte == 3){
		directionL = 1;
		directionR = 0;
	}

	if(inByte == 4){
		directionR = 1;
		directionL = 0;
	}

	if (inByte == 5){
		if(speedL < MaxSpeed){
			speedL += 10;
		}
		if(speedR < MaxSpeed){
			speedR += 10;
		}
	}

	if (inByte == 6){
		if(speedL > MinSpeed){
			speedL -= 10;
		}
		if(speedR > MinSpeed){
			speedR -= 10;
		}
	}

	if(inByte == 7){
		if(speedR < MaxSpeed){
			speedR += 10;
		}
		else if(speedL > MinSpeed){
			speedL -= 10;
		}
	}

	if(inByte == 8){
		if(speedL < MaxSpeed){
			speedL += 10;
		}
		else if(speedR > MinSpeed){
			speedR -= 10;
		}
	}

	if(inByte == 13){
		directionL = 1;
		directionR = 0;
		speedL = 75;
		speedR = 75;
	}

	if(inByte == 14){
		directionL = 0;
		directionR = 1;
		speedL = 75;
		speedR = 75;
	}

	if(inByte == 20){
		speedL = 0;
		speedR = 0;
		encoder_L_driven = 0;
		encoder_R_driven = 0;
	}

	if(inByte == 0){
		directionL = 0;
		directionR = 0;
		speedL = 0;
		speedR = 0;
	}
	act(speedL, speedR, directionL, directionR);
}

void act(uint8_t speedL, uint8_t speedR, uint8_t links, uint8_t rechts){
	PORTC = (links<<2)|(rechts<<3);
	OCR1A = speedR;
	OCR1B = speedL;
}

void initEncoders(void){
	// Enable/Disable Encoders, IR Receiver
	DDRB |= (1 << 4);
	PORTB |= (1 << PINB4);
	//zet input van sensor
	DDRD |= (0<<2)|(0<<3);
	// Initialize External interrupts:
	MCUCR |= (1<<ISC00) | (1<<ISC10);
	GICR |= (1 << INT1) | (1 << INT0);
	encoder_L_count = 0;
	encoder_R_count = 0;
}

//enc_L
ISR(INT0_vect)
{
	encoder_L_count++;
	encoder_L_driven++;
}
//enc_R
ISR(INT1_vect)
{
	encoder_R_count++;
}

int encoder_Count_to_Centimeter(int encoder_count){
	//0.25 voor milimeters 0.025 voor centimeters
	return (encoder_count * 0.025);
}