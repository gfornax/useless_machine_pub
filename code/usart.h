#ifndef _USART_H_
#define _USART_H_

#include <inttypes.h>


#define BAUD 38400
#define MYUBRR F_CPU/16/BAUD-1

void usart_init(uint16_t ubrr);
char usart_getchar( void );
void usart_putchar( char data );
void usart_pstr (char *s);
unsigned char usart_kbhit(void);

#endif
