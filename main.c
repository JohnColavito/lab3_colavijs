/*
 * Lab3Theremin.c
 *
 * Created: 2/17/2023 10:44:09 AM
 * Author : johnc
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

void _init() {
	UBRR0H = (unsigned char)(BAUD_PRESCALER >> 8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	UCSR0B |= (1<<RXEN0);
	UCSR0B |= (1<<TXEN0);
	UCSR0C |= (1<<UCSZ01);
	UCSR0C |= (1<<UCSZ00);
	UCSR0C |= (1<<USBS0);
}

void printChar(unsigned char c) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void print(char* str) {
	while (*str != 0x00) {
		printChar(*str);
		str++;
	}
}

// 8 ###############################################################
volatile long count = 0;
volatile int recievedB = 1;
char String[50];
volatile int mode = 0;
volatile int adcV = -1;
int dC = -1;

ISR(TIMER1_CAPT_vect)
{
	if (!recievedB) {
		if (TCCR1B & (1 << ICES1)) {
			TCNT1 = 0;
			TCCR1B &= ~(1<<ICES1);
		} else {
			count = TCNT1;
			TCCR1B |= (1<<ICES1);
			recievedB = 1;
		}
	}
	if (PINB & (1<<PINB2)) {
		if (mode == 0) {
			mode = 1;
		} else {
			mode = 0;
		}
	}
}

int main(void)
{
	Initialize();

	while (1) {
		if (recievedB) {
			PORTB |= (1<<PORTB1);
			_delay_us(157);
			PORTB &= ~(1<<PORTB1);
			recievedB = 0;
		}
		int temp = (64 * (count) * 8) / 1024 / 58;
		int distance = (int) (.4*temp+28);
		if (mode) {
			if (distance < 32) {
				OCR0A = 30;
			} else if (distance < 34) {
				OCR0A = 32;
			} else if (distance < 38) {
				OCR0A = 35;
			} else if (distance < 42) {
				OCR0A = 40;
			} else if (distance < 46) {
				OCR0A = 45;
			} else if (distance < 50) {
				OCR0A = 47;
			} else if (distance < 55) {
				OCR0A = 53;
			} else {
				OCR0A = 60;
			}
		} else {
			OCR0A = distance;
		}
		
		dC = (adcV-240)/75;
		if (dC < 0) {
			dC = 0;
		} else if (dC > 9) {
			dC = 9;
		}
		dC = dC*5.0 + 5;
		
		OCR0B = (OCR0A*((dC/100.0)));
		
		sprintf(String, "ADC: %d, DC: %d, mode: %d \n", adcV, dC, OCR0B);
		print(String);
		
		_delay_ms(1000);
	}
}

ISR(ADC_vect) {
	adcV = ADC;
}

void Initialize()
{
	_init();
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// trig
	DDRB |= (1<<DDB1);
	PORTB &= ~(1<<PORTB1);
	// echo
	DDRB &= ~(1<<DDB0);
	
	DDRB &= ~(1<<DDB2);
	PORTB |= (1<<PORTB2);
	
	// Timer1 for tick count
	// Enable clock (/8; 1t = 2MHz)
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);
	
	//Sets the clock to normal
	TCCR1B &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	
	//look for a rising edge
	TCCR1B |= (1<<ICES1);
	
	// Clear input capture flag
	TIFR1 |= (1<<ICF1);
	
	// Enable input capture interrupt
	TIMSK1 |= (1<<ICIE1);
	
	TCCR1A |= (1<<COM1A0);
	TCCR1A &= ~(1<<COM1A1);	

	// Enable clock0 (/64; 1t = 250000Hz)
	TCCR0B |= (1<<CS00);
	TCCR0B |= (1<<CS01);
	TCCR0B &= ~(1<<CS02);
	
	// Sets the clock to PWM phase correct
	TCCR0A &= ~(1<<WGM01);
	TCCR0A |= (1<<WGM00);
	TCCR0B |= (1<<WGM02);
	
	// compare match mode
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	TCCR0A |= (1<<COM0B0);
	TCCR0A |= (1<<COM0B1);
	
	OCR0A = 30;
	OCR0B = OCR0A * (1.0 - (45/100.0));
	
	// Enable ADC and set the prescaler to 128 for 125 kHz 
	PRR &= ~(1<<PRADC);
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	// Set the reference voltage and enable ADC
	ADMUX &= 0xF0;
	ADMUX |= (1<<REFS0);
	
	//select channel 0
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	
	ADCSRA |= (1<<ADATE);
	
	ADCSRB &= ~(1<<ADTS0); 
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	
	//disable dib
	DIDR0 |= (1<<ADC0D);
		
	ADCSRA |= (1<<ADEN);
	  
	ADCSRA |= (1<<ADIE);  
	
	ADCSRA |= (1<<ADSC);
	
	sei();
}
// 8 ###############################################################


// 7 ###############################################################
/*volatile long count = 0;
volatile int recievedB = 1;
char String[1000];
int x = 0;

ISR(TIMER1_CAPT_vect)
{
	if (!recievedB) {
		if (TCCR1B & (1 << ICES1)) {
			TCNT1 = 0;
			TCCR1B &= ~(1<<ICES1);
		} else {
			count = TCNT1;
			TCCR1B |= (1<<ICES1);
			recievedB = 1;
		}
	}
}

int main(void)
{
	Initialize();

	while (1) {
		if (recievedB) {
			PORTB |= (1<<PORTB1);
			_delay_us(157);
			PORTB &= ~(1<<PORTB1);
			recievedB = 0;
		}
		int temp = (64 * (count) * 8) / 1024 / 58;
		int distance = (int) (.4*temp+28);
		if (distance < 32) {
			OCR0A = 30;
		} else if (distance < 34) {
			OCR0A = 32;
		} else if (distance < 38) {
			OCR0A = 35;
		} else if (distance < 42) {
			OCR0A = 40;
		} else if (distance < 46) {
			OCR0A = 45;
		} else if (distance < 50) {
			OCR0A = 47;
		} else if (distance < 55) {
			OCR0A = 53;
		} else {
			OCR0A = 60;
		}
		
		_delay_ms(1000);
	}
}

void Initialize()
{
	_init();
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// trig
	DDRB |= (1<<DDB1);
	PORTB &= ~(1<<PORTB1);
	// echo
	DDRB &= ~(1<<DDB0);
	
	// Timer1 for tick count
	// Enable clock (/8; 1t = 2MHz)
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);
	
	//Sets the clock to normal
	TCCR1B &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	
	//look for a rising edge
	TCCR1B |= (1<<ICES1);
	
	// Clear input capture flag
	TIFR1 |= (1<<ICF1);
	
	// Enable input capture interrupt
	TIMSK1 |= (1<<ICIE1);
	
	TCCR1A |= (1<<COM1A0);
	TCCR1A &= ~(1<<COM1A1);	

	// Enable clock0 (/64; 1t = 250000Hz)
	TCCR0B |= (1<<CS00);
	TCCR0B |= (1<<CS01);
	TCCR0B &= ~(1<<CS02);
	
	// Sets the clock to PWM phase correct
	TCCR0A &= ~(1<<WGM01);
	TCCR0A |= (1<<WGM00);
	TCCR0B |= (1<<WGM02);
	
	// compare match mode
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	
	OCR0A = 30;
	
	sei();
}*/
// 7 ###############################################################

// 6 ###############################################################
/*volatile long count = 0;
volatile int recievedB = 1;
char String[50];

ISR(TIMER1_CAPT_vect)
{
	if (!recievedB) {
		if (TCCR1B & (1 << ICES1)) {
			TCNT1 = 0;
			TCCR1B &= ~(1<<ICES1);
			} else {
			count = TCNT1;
			TCCR1B |= (1<<ICES1);
			recievedB = 1;
		}
	}
}

int main(void)
{
	Initialize();

	while (1) {
		if (recievedB) {
			PORTB |= (1<<PORTB1);
			_delay_us(157);
			PORTB &= ~(1<<PORTB1);
			recievedB = 0;
		}
		int temp = (64 * (count) * 8) / 1024 / 58;
		int distance = (int) (.4*temp+28);
		OCR0A = distance;
		_delay_ms(1000);
	}
}

void Initialize()
{
	_init();
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// trig
	DDRB |= (1<<DDB1);
	PORTB &= ~(1<<PORTB1);
	// echo
	DDRB &= ~(1<<DDB0);
	
	// Timer1 for tick count
	// Enable clock (/8; 1t = 2MHz)
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);
	
	//Sets the clock to normal
	TCCR1B &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	
	//look for a rising edge
	TCCR1B |= (1<<ICES1);
	
	// Clear input capture flag
	TIFR1 |= (1<<ICF1);
	
	// Enable input capture interrupt
	TIMSK1 |= (1<<ICIE1);
	
	TCCR1A |= (1<<COM1A0);
	TCCR1A &= ~(1<<COM1A1);

	// Enable clock0 (/64; 1t = 250000Hz)
	TCCR0B |= (1<<CS00);
	TCCR0B |= (1<<CS01);
	TCCR0B &= ~(1<<CS02);
	
	// Sets the clock to PWM phase correct
	TCCR0A &= ~(1<<WGM01);
	TCCR0A |= (1<<WGM00);
	TCCR0B |= (1<<WGM02);
	
	// compare match mode
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	
	OCR0A = 30;
	
	sei();
}*/
// 6 ###############################################################

// 5 ###############################################################
/*volatile long count = 0;
volatile int recievedB = 1; 
char String[50];

void Initialize()
{	
	_init();
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// trig
	DDRB |= (1<<DDB1);
	PORTB &= ~(1<<PORTB1);
	// echo
	DDRB &= ~(1<<DDB0);
	
	// Timer1 for tick count
	// Enable clock (/8; 1t = 2MHz)
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);
	
	//Sets the clock to normal
	TCCR1B &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	
	//look for a rising edge
	TCCR1B |= (1<<ICES1);
	
	// Clear input capture flag
	TIFR1 |= (1<<ICF1);
	
	// Enable input capture interrupt
	TIMSK1 |= (1<<ICIE1);
	
	sei();
}

ISR(TIMER1_CAPT_vect)
{
	if (!recievedB) {
		if (TCCR1B & (1 << ICES1)) {
			TCNT1 = 0;
			TCCR1B &= ~(1<<ICES1);
		} else {
			count = TCNT1;
			TCCR1B |= (1<<ICES1);
			recievedB = 1;
		}
	}
}

int main(void)
{
	Initialize();

	while (1) {
		if (recievedB) {
			int distance = (64 * (count) * 8) / 1024 / 58;
			sprintf(String, "count: %ld, distance: %d \n", count, distance);
			print(String);
			PORTB |= (1<<PORTB1);
			_delay_us(157);
			PORTB &= ~(1<<PORTB1);
			recievedB = 0;
		}
		_delay_ms(1);
	}
}*/
// 5 ###############################################################

// 4 ###############################################################
/*void Initialize()
{
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// Enable clock (/256; 1t = 62500Hz)
	TCCR0B &= ~(1<<CS00);
	TCCR0B &= ~(1<<CS01);
	TCCR0B |= (1<<CS02);
	
	// enable output Compare Match A
	TIMSK0 |= (1<<OCIE0A);
	
	// Sets the clock to PWM
	TCCR0A &= ~(1<<WGM01);
	TCCR0A |= (1<<WGM00);
	TCCR0B |= (1<<WGM02);
	
	// compare match mode
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	
	OCR0A = 35;
	
	sei();
}

int main(void)
{
	Initialize();

	while (1) {	}
}*/
// 4 ###############################################################

// 3 ###############################################################
/*void Initialize()
{
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// Enable clock (/256; 1t = 62500Hz)
	TCCR0B &= ~(1<<CS00);
	TCCR0B &= ~(1<<CS01);
	TCCR0B |= (1<<CS02);
	
	// enable output Compare Match A
	TIMSK0 |= (1<<OCIE0A);
	
	// enable overflow interrupt
	TIMSK0 |= (1<<TOIE0);
	
	// Clear overflow interrupt flag
	TIFR0 |= (1<<TOV0);
	
	// Sets the clock to CTC
	TCCR0A |= (1<<WGM01);
	TCCR0A &= ~(1<<WGM00);
	TCCR0A &= ~(1<<WGM02);
	
	// compare match mode
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	
	OCR0A = 70;
	
	sei();
}

int main(void)
{
	Initialize();

	while (1) {	}
}*/
// 3 ###############################################################

// 2 ###############################################################
/*void Initialize()
{
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// Enable clock (/256; 1t = 62500Hz)
	TCCR0B &= ~(1<<CS00);
	TCCR0B &= ~(1<<CS01);
	TCCR0B |= (1<<CS02);
	
	// enable output Compare Match A
	TIMSK0 |= (1<<OCIE0A);
	
	// enable overflow interrupt
	TIMSK0 |= (1<<TOIE0);
	
	// Clear overflow interrupt flag
	TIFR0 |= (1<<TOV0);
	
	// Sets the clock to Normal
	TCCR0A &= ~(1<<WGM01);
	TCCR0A &= ~(1<<WGM00);
	TCCR0A &= ~(1<<WGM02);
	
	// compare match mode
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A |= (1<<COM0A0);
	
	OCR0A = 69;
	
	sei();
}

ISR(TIMER0_COMPA_vect)
{
	PORTD ^= (1<<PORTD6);
	TCNT0 = 0;
}

ISR(TIMER0_OVF_vect)
{
	//PORTD ^= (1<<PORTD6);
}

int main(void)
{
	Initialize();

	while (1){}
}*/
// 2 ###############################################################

// 1 ###############################################################
/*void Initialize()
{
	cli();
	
	//sets pin 6 as output
	DDRD |= (1<<DDD6);
	
	// Enable clock (/256; 1t = 62500Hz) 
	TCCR0B &= ~(1<<CS00);
	TCCR0B &= ~(1<<CS01);
	TCCR0B |= (1<<CS02);
	
	// enable output Compare Match A 
	TIMSK0 |= (1<<OCIE0A);
	
	// enable overflow interrupt
	TIMSK0 |= (1<<TOIE0);
	
	// Clear overflow interrupt flag
	TIFR0 |= (1<<TOV0);
	
	//Sets the clock to Normal
	TCCR0A &= ~(1<<WGM01);
	TCCR0A &= ~(1<<WGM00);
	TCCR0B &= ~(1<<WGM02);
	
	//Compare match output A mode
//	TCCR0A |= (1<<COM0A1);
//	TCCR0B |= (1<<WGM12);
//	TCCR0B |= (1<<WGM13);
	
	sei();
}

ISR(TIMER0_OVF_vect)
{
	PORTD ^= (1<<PORTD6);
}

int main(void)
{
	Initialize();

    while (1) 
    {
		
    }
}*/
// 1 ###############################################################


