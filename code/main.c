#include <stdlib.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/atomic.h>

#include "usart.h"

/* uncomment to use FIFO mode for turning off switches in same order */
//#define USE_FIFO

#define OCLEFT OCR1A
#define OCRIGHT OCR1B
#define OCSERVO OCR2

/* encoder inputs, reverse PD3/PD4 if necessary */
#define PHASE_A	(PIND & 1<<PD4)
#define PHASE_B (PIND & 1<<PD3)


/* 
 * controller parameters section
 * These parameters depend on the printer, supply voltage, 
 * encoder resolution and sampling rate.
 * There is no correction for non-static supply voltage.
 */
#define PID_KP 	18 // proportional, 
#define PID_KI	0.0 // works without integral in most cases
#define PID_KD 	50 // 
/*
 * NOTE: to adjust KI and KD for sampling rate: 
 * KI ~ 1/rate
 * KD ~ rate
 */

/* servo positions, safe range is about 60-140 */
#define SERVO_IDLE	servo_set=64 
#define SERVO_TOG	servo_set=134
#define	SERVO_HOLD	servo_set=121 //position while moving if any switch is active

/* safety borders, put servo in idle mode */
#define POSLIMITLOW     900
#define POSLIMITHIGH    7000

/* position of switches */
#define POS0	1085
#define POS1 	1855
#define POS2	2617	
#define POS3	3372
#define POS4	4162
#define POS5	4915
#define POS6	5698
#define POS7	6436

/* tolerance +/- of position for activating servo */
#define POSTOL	24 

/* delay times for switches, see main */
#define WAIT0	6
#define WAIT1	10


int usart_putchar_printf(char var, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

volatile int16_t position;
volatile int16_t position_set;
volatile uint8_t lid_pos;
static int8_t last;
static uint8_t led_output;
static uint8_t keyvar;
static uint8_t calib_active;
static uint8_t servo_set;

/* encoder readout, thx to P. Dannegger */
void encode_init( void )
{
	int8_t new;
	new = 0;
	if( PHASE_A )
	    new = 3;
	if( PHASE_B )
	    new ^= 1;
	last = new;
	position = 0;
	TCCR0 |=  (1 << CS00);
	TIMSK |= 1<<TOIE0;
}

void init( void )
{
	/* LID PWM, OC1A, OC1B, SRV PWM */
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);

	/* SL_LED, DS, SHCK, STCK */
	DDRD |= (1 << PD2) | (1 << PD5) | (1 << PD6) | (1 << PD7);
	
	/* pullups for switches */
	PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
	PORTB |= (1 << PB4) | (1 << PB5);

	/* 8 bit PWM */	
        TCCR1A |= (1<<WGM10);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);
        TCCR1B |= (1 << CS10);
        /* 8 bit 500Hz */
	TCCR2 |= (0 << CS22) | (0 << CS21) | (1 << CS20);
	TIMSK |= (1 << TOIE2);

	encode_init();
	
	led_output = 0;
	sei();
}

/* flip these functions to reverse moving direction */
/* PB5 + OCRIGHT */
void move_right(uint8_t speed)
{
	OCRIGHT = 0;
	if(keyvar || calib_active) 
		OCLEFT = speed;
	else
		OCLEFT = 0;
}

/*PB4 + OCLEFT */
void move_left(uint8_t speed)
{
	OCLEFT = 0;
	if(keyvar || calib_active)
		OCRIGHT = speed;
	else
		OCLEFT = 0;
}

void m_stop( void )
{
	OCRIGHT = 255; 
	OCLEFT = 255; 
}

/* basic PID controller */
void set_pid( void )
{
	static int32_t int_sum = 0;
	static int32_t deltalast = 0;
	static uint16_t idle_count = 0;
	int64_t m_output = 0;

	int32_t delta = position_set - position ;

        if (!keyvar && idle_count != 1000){
                idle_count++;
        } else if (keyvar){
                idle_count = 0;
        }

        /* safety range for servo */
        if(position <= POSLIMITLOW || position >= POSLIMITHIGH){
                SERVO_IDLE;
        }


	int_sum += delta;
	if (int_sum > 20000) 
		int_sum = 20000;
	if (int_sum < -20000)
		int_sum = -20000;

	m_output = PID_KP*delta + (int32_t)(PID_KI*(float)int_sum) + (PID_KD*(delta-deltalast));
	deltalast = delta;

	m_output /= 2;

	if (m_output > 0) {
		if (m_output > 255) 
			move_right(255);
		else 
			move_right(m_output);
	} else if (m_output < 0){
		if (m_output < -255)
			move_left(255);
		else 
			move_left(-1*m_output);
	} else {
		m_stop();
	}

}

/*
 *  ISR must be fast enough to catch encoder at full speed
 */

ISR( TIMER2_OVF_vect )
{
	static uint16_t srv_prescale = 0;
	if (srv_prescale < 1000) {
		srv_prescale++;
		if (srv_prescale > servo_set) 
			PORTB &= ~(1 << PB3);
	} else {
		PORTB |= (1 << PB3);
		srv_prescale = 0;
	}
}

ISR( TIMER0_OVF_vect )
{
	static uint16_t prescale = 0;
	static uint8_t led_set = 1;
	/* this variable can be used to fix LED output ordering */
	uint8_t led_aligned = (led_output & 0x3f) | (led_output & 0x80) >> 1 | (led_output & 0x40) << 1;
//	cli();
	TCNT0 = 145; 
	int8_t new, diff;
	new = 0;
	if( PHASE_A )
	    new = 3;
	if( PHASE_B )
	    new ^= 1;
	diff = last - new;
	if ( diff & 1 ){
		last = new;
		position += (diff & 2) -1;
	}
	if (prescale < 20000){
		prescale++;
		if (prescale > 2000)
			PORTD &= ~(1 << PD2);
	} else {
		prescale = 0;
		if (keyvar)
			PORTD |= (1 << PD2);
	}

	if(led_aligned & led_set)
		PORTD |= (1 << PD5);
	else
		PORTD &= ~(1 << PD5);

	PORTD |= (1 << PD7);
	PORTD &= ~(1 << PD7);
	if(led_set <  128){
		led_set = led_set << 1;
	} else {
		led_set = 1;
		PORTD |= (1 << PD6);
		PORTD &= ~(1 << PD6);
	}
//	sei();
}


unsigned char byterev(unsigned char x) 
{ 
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa); 
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc); 
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0); 
    return x;    
}

/* busy delay loop with PID set */
void busydelay(uint8_t cycle)
{
	for(;cycle > 0; cycle--){
		set_pid();
		_delay_us(100);
	}
}

int main(void)
{
	int16_t last_pos;
	calib_active = 0;
	init();

	stdout = &mystdout;


        usart_init( MYUBRR );
	printf("\n\nuseless machine public v1.0 started\n");
	
	SERVO_IDLE;


	/* move slider to max. left to set zero position */
	calib_active = 1;
	move_left(200); // kick the slider 
	_delay_ms(100);
	move_left(180); // use a value just high to keep slider moving 
	printf("\ncalibrating zero position\n");
	do {
		last_pos = position;
		for (uint8_t i = 1; i < 128; i = i << 1) {
			led_output = i;
			_delay_ms(20);
		}
		led_output = 128;
		_delay_ms(20);
		for (uint8_t i = 128; i > 0; i = i >> 1) {
			led_output = i;
			_delay_ms(20);
		}
	} while (position != last_pos);
	calib_active = 0;
	printf("done\n");
	m_stop();

	position = 0;

	position_set = 2000;

	while ( 1 ){
		keyvar = 0;
		/* this section can be used to configure the switch order */
		if(PINC & (1 << PC0))
			keyvar += 0x10;
		if(PINC & (1 << PC1))
			keyvar += 0x20;
		if(PINC & (1 << PC2))
			keyvar += 0x04;
		if(PINC & (1 << PC3))
			keyvar += 0x08;
		if(PINC & (1 << PC4))
			keyvar += 0x01;
		if(PINC & (1 << PC5))
			keyvar += 0x02;
		if(PINB & (1 << PB4))
			keyvar += 0x40;
		if(PINB & (1 << PB5))
			keyvar += 0x80;
#if 1 
		printf("\rpos: %5d key: ", position);
		
		for (uint8_t i = 1; i < 128; i = i << 1) {
			if(keyvar & i)
				printf("1");
			else
				printf("0");
		}
		if(keyvar & 128)
			printf("1");
		else
			printf("0");
#endif		
		//led_output = byterev(keyvar);
		led_output = keyvar;

#ifdef USE_FIFO
#error this minimal version includes fixed prio mode only
#else 
		/* lsb is SW0 */
		if (keyvar & 1) {
			busydelay(WAIT0);
			position_set = POS0;
			if( position >= (POS0 - POSTOL) && position <= (POS0 + POSTOL)){
				SERVO_TOG;
			} else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 2){
			busydelay(WAIT0);
			position_set = POS1;
			if( position >= (POS1 - POSTOL) && position <= (POS1 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 4){
			busydelay(WAIT0);
			position_set = POS2;
			if( position >= (POS2 - POSTOL) && position <= (POS2 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 8){
			busydelay(WAIT0);
			position_set = POS3;
			if( position >= (POS3 - POSTOL) && position <= (POS3 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 16){
			busydelay(WAIT0);
			position_set = POS4;
			if( position >= (POS4 - POSTOL) && position <= (POS4 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 32){
			busydelay(WAIT0);
			position_set = POS5;
			if( position >= (POS5 - POSTOL) && position <= (POS5 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 64){
			busydelay(WAIT0);
			position_set = POS6;
			if( position >= (POS6 - POSTOL) && position <= (POS6 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else if (keyvar & 128){
			busydelay(WAIT0);
			position_set = POS7;
			if( position >= (POS7 - POSTOL) && position <= (POS7 + POSTOL))
				SERVO_TOG;
			else if (keyvar){
				SERVO_HOLD;
				busydelay(WAIT1);
			}
		} else {
			SERVO_IDLE;
			busydelay(25);
		}
#endif
	}
}

