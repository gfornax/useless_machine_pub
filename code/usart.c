#include <stdlib.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>

void usart_init( uint16_t ubrr) 
{
    UBRRH = (uint8_t)(ubrr>>8);
    UBRRL = (uint8_t)ubrr;


//    UBRRL = 12;
    UCSRB = (1<<TXEN);

  //  UCSRC = (3<<UCSZ0);
}

void usart_putchar(char data)
{
    while ( !(UCSRA & (_BV(UDRE))) );
    UDR = data;
}

char usart_getchar(void)
{
    while ( !(UCSRA & (_BV(RXC))) );
    return UDR;
}

unsigned char usart_kbhit(void)
{
    unsigned char b;
    b=0;
    if(UCSRA & (1<<RXC)) b=1;
    return b;
}
void usart_pstr(char *s)
{
    while (*s) {
        usart_putchar(*s);
        s++;
    }
}

// this function is called by printf as a stream handler

int usart_putchar_printf(char var, FILE *stream)
{

    // translate \n to \r for br@y++ terminal

    if (var == '\n') usart_putchar('\r');
    usart_putchar(var);
    return 0;
}

