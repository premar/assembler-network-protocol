/******************************************************************************
 * File:            network_atmega8515.c
 *
 * Description:     Network physical layer implementation for the AVR
 *                  ATmega8515 microcontroller.
 *
 * Author:          Marius Preuss, Viktor Puselja
 *
 * Creation date:   2014-08-31
 *
******************************************************************************/

#include "network_hardware.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// network timer interrupt modes
#define NET_ATMG8515_TIMER_MODE_NONE   0
#define NET_ATMG8515_TIMER_MODE_READ   1
#define NET_ATMG8515_TIMER_MODE_WRITE  2

// network external interrupt constants
#define NET_ATMG8515_INT_VECT        INT0_vect
#define NET_ATMG8515_INT             INT0
#define NET_ATMG8515_INTF            INTF0
#define NET_ATMG8515_ISC1            ISC01
#define NET_ATMG8515_ISC0            ISC00

// network timer interrupt constants
#define NET_ATMG8515_TIMER_OVF_VECT  TIMER0_OVF_vect
#define NET_ATMG8515_TIMER_COMP_VECT TIMER0_COMP_vect
#define NET_ATMG8515_TOIE            TOIE0
#define NET_ATMG8515_OCIE            OCIE0
#define NET_ATMG8515_TOV             TOV0
#define NET_ATMG8515_OCF             OCF0
#define NET_ATMG8515_TCCR            TCCR0
#define NET_ATMG8515_CS0             CS00
#define NET_ATMG8515_CS1             CS01
#define NET_ATMG8515_CS2             CS02
#define NET_ATMG8515_TCNT            TCNT0
#define NET_ATMG8515_OCR             OCR0

// network port constants
#define NET_ATMG8515_PORT            PORTD
#define NET_ATMG8515_DDR             DDRD
#define NET_ATMG8515_PIN             PIND
#define NET_ATMG8515_PORT_PIN        PD2

// current timer mode
volatile uint8_t net_atmg8515_timer_mode = NET_ATMG8515_TIMER_MODE_NONE;

/*
 * Function: net_atmg8515_start_reading()
 * Description: Starts reading bytes from the physical layer.
 */
void net_atmg8515_start_reading(void);

/*
 * Function: net_atmg8515_start_reading_delayed()
 * Description: Starts the actual reading after the initial delay.
 */
void net_atmg8515_start_reading_delayed(void);

/*
 * Function: net_atmg8515_process_next_tick()
 * Description: Processes the next read or write tick.
 */
void net_atmg8515_process_next_tick(void);

/*
 * Function: net_atmg8515_set_port_mode()
 * Description: Sets the port to either read or write mode.
 */
void net_atmg8515_set_port_mode(bool read);

/*
 * Function: net_atmg8515_read_bit()
 * Description: Reads a single bit.
 */
uint8_t net_atmg8515_read_bit(void);

/*
 * Function: net_atmg8515_write_bit()
 * Description: Writes a single bit.
 */
void net_atmg8515_write_bit(uint8_t bit);

// external interrupt functions
void net_atmg8515_initialize_external_int(void);
void net_atmg8515_disable_external_int(void);
void net_atmg8515_enable_external_int(void);

// timer interrupt functions
void net_atmg8515_initialize_timer_int(void);
void net_atmg8515_disable_timer_ovf_int(void);
void net_atmg8515_disable_timer_cpm_int(void);
void net_atmg8515_enable_timer_ovf_int(void);
void net_atmg8515_enable_timer_cpm_int(void);

void network_initialize_hardware(void)
{
    net_atmg8515_initialize_external_int();
    net_atmg8515_initialize_timer_int();
    net_atmg8515_set_port_mode(true);
    net_atmg8515_enable_external_int();
}

void network_delay_ms(void)
{
    _delay_ms(1);
}

void network_start_writing(void)
{
    while (net_atmg8515_timer_mode != NET_ATMG8515_TIMER_MODE_NONE);

    net_atmg8515_disable_external_int();
    
    net_atmg8515_timer_mode = NET_ATMG8515_TIMER_MODE_WRITE;
    
    net_atmg8515_set_port_mode(false);
    net_atmg8515_enable_timer_ovf_int();
}

void network_wait_writing_complete(void)
{
    while (net_atmg8515_timer_mode == NET_ATMG8515_TIMER_MODE_WRITE);
}

void net_atmg8515_start_reading(void)
{
    net_atmg8515_disable_external_int();

    net_atmg8515_timer_mode = NET_ATMG8515_TIMER_MODE_READ;

    // use compare match with 127 to start reading in the middle of a bit
    net_atmg8515_enable_timer_cpm_int();
}

void net_atmg8515_start_reading_delayed(void)
{
    net_atmg8515_disable_timer_cpm_int();

    net_atmg8515_enable_timer_ovf_int();
    
    net_atmg8515_process_next_tick();
}

void net_atmg8515_process_next_tick(void)
{
    static uint8_t index = 0;
    static uint8_t net_atmg8515_timer_byte = 0;
    uint8_t bit;
    bool reset = false;

    if (net_atmg8515_timer_mode == NET_ATMG8515_TIMER_MODE_READ) {
        bit = net_atmg8515_read_bit();
        
        if (index == 0) {
            net_atmg8515_timer_byte = 0;
            if (bit != 0) {
                reset = true;
                network_process_byte(NETWORK_END_OF_DATA, 0);
            }
        }
        else {
            net_atmg8515_timer_byte <<= 1;
            net_atmg8515_timer_byte |= bit;
            
            if (index == 8)
                network_process_byte(NETWORK_NO_ERROR,
                    net_atmg8515_timer_byte);
        }
    }
    else if (net_atmg8515_timer_mode == NET_ATMG8515_TIMER_MODE_WRITE) {
        if (index == 0) {
            bit = 0;
            reset = !network_get_next_byte(&net_atmg8515_timer_byte);
        }
        else {
            bit = net_atmg8515_timer_byte & 0b10000000;
            net_atmg8515_timer_byte <<= 1;
        }
        
        if (!reset)
            net_atmg8515_write_bit(bit);
    }
    
    if (index == 8 || reset) {
		if (reset) {
        	net_atmg8515_disable_timer_ovf_int();
        	net_atmg8515_set_port_mode(true);
        	net_atmg8515_enable_external_int();
    
        	net_atmg8515_timer_mode = NET_ATMG8515_TIMER_MODE_NONE;
    	}
        index = 0;
	} else {
        index++;
	}
}

void net_atmg8515_set_port_mode(bool read)
{
    if (read) {
        // set input pull-up
        NET_ATMG8515_PORT |= (1 << NET_ATMG8515_PORT_PIN);
        NET_ATMG8515_DDR &= ~(1 << NET_ATMG8515_PORT_PIN);
    }
    else {
        // set output high
        NET_ATMG8515_PORT |= (1 << NET_ATMG8515_PORT_PIN);
        NET_ATMG8515_DDR |= (1 << NET_ATMG8515_PORT_PIN);
    }
}

uint8_t net_atmg8515_read_bit(void)
{
    return (NET_ATMG8515_PIN & (1 << NET_ATMG8515_PORT_PIN))
        >> NET_ATMG8515_PORT_PIN;
}

void net_atmg8515_write_bit(uint8_t bit)
{
    if (bit) // set port pin
        NET_ATMG8515_PORT |= (1 << NET_ATMG8515_PORT_PIN); 
    else // clear port pin
        NET_ATMG8515_PORT &= ~(1 << NET_ATMG8515_PORT_PIN);
}

void net_atmg8515_initialize_external_int(void)
{
    net_atmg8515_disable_external_int();

    // set interrupt on falling edge
    MCUCR &= ~(1 << NET_ATMG8515_ISC0);
    MCUCR |= (1 << NET_ATMG8515_ISC1);
}

void net_atmg8515_disable_external_int(void)
{
    // clear external interrupt flag
    GICR &= ~(1 << NET_ATMG8515_INT);
}

void net_atmg8515_enable_external_int(void)
{
    // clear eventual external register flag
    GIFR = (1 << NET_ATMG8515_INTF);

    // set external interrupt flag
    GICR |= (1 << NET_ATMG8515_INT);
}

void net_atmg8515_initialize_timer_int(void)
{
    net_atmg8515_disable_timer_ovf_int();
    net_atmg8515_disable_timer_cpm_int();

    // set normal mode, 1024 pre-scaler
    NET_ATMG8515_TCCR = (1 << NET_ATMG8515_CS1 | 1 << NET_ATMG8515_CS0);

    // compare match, used by read
    NET_ATMG8515_OCR = 127;
}

void net_atmg8515_disable_timer_ovf_int(void)
{
    // clear timer overflow interrupt enable flag
    TIMSK &= ~(1 << NET_ATMG8515_TOIE);
}

void net_atmg8515_disable_timer_cpm_int(void)
{
    // clear timer compare match interrupt enable flag
    TIMSK &= ~(1 << NET_ATMG8515_OCIE);
}

void net_atmg8515_enable_timer_ovf_int(void)
{
    // reset counter
    NET_ATMG8515_TCNT = 0;

    // clear eventual overflow register flag
    TIFR = (1 << NET_ATMG8515_TOV);

    // set timer overflow interrupt enable flag
    TIMSK |= (1 << NET_ATMG8515_TOIE);
}

void net_atmg8515_enable_timer_cpm_int(void)
{
    // reset counter
    NET_ATMG8515_TCNT = 0;

    // clear eventual compare register flag
    TIFR = (1 << NET_ATMG8515_OCF);

    // set timer compare match interrupt enable flag
    TIMSK |= (1 << NET_ATMG8515_OCIE);
}

ISR(NET_ATMG8515_INT_VECT)
{
    net_atmg8515_start_reading();
}

ISR(NET_ATMG8515_TIMER_COMP_VECT)
{
    net_atmg8515_start_reading_delayed();
}

ISR(NET_ATMG8515_TIMER_OVF_VECT)
{
    net_atmg8515_process_next_tick();
}
