/******************************************************************************
* File:         network.c
*
* Description:  Code file of the network api for communication between
*               multiple microcontrollers.
*
* Author:       Marius Preuss, Viktor Puselja
*
* Date:         2014-01-21
*
******************************************************************************/

#include "network.h"

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

struct network_connection network_conn;

volatile uint8_t network_timer_int_mode = NETWORK_TIMER_INTERRUPT_MODE_NONE;

volatile uint8_t network_timer_int_byte = 0;

#define NETWORK_CHECK_IS_INITIALIZED if (!network_conn.is_initialized) \
{ return NETWORK_NOT_INITIALIZED; }

uint8_t network_initialize(void)
{
    uint8_t error = NETWORK_NO_ERROR;
    bool is_address_used = true;
    uint8_t address;

    if (!network_conn.is_initialized)
    {
        network_initialize_external_int();
        network_initialize_timer_int();
        network_set_port_mode(true);

        network_conn.address = NETWORK_ADDRESS_NONE;
        network_conn.last_id = 0;
        network_conn.is_packet_avaiable = false;
        network_conn.is_initialized = true;

        address = NETWORK_ADDRESS_MIN;
        do
        {
            is_address_used = network_check(address++);
        } while (address < NETWORK_ADDRESS_MAX && is_address_used);

        if (!is_address_used)
        {
            network_conn.address = --address;
        }
        else
        {
            network_conn.is_initialized = false;
            error = NETWORK_NONE_ADDRESS_AVAILABLE;
        }
    }
    return error;
}

bool network_check(uint8_t address)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header packet_header;

    error = network_send(address, NETWORK_STATUS_CHECK, NETWORK_COMMAND_NONE,
        NULL, 0);
    if (error == NETWORK_NO_ERROR)
    {
        error = network_wait_for_packet(address, NETWORK_STATUS_ACKNOWLEDGE,
            NETWORK_COMMAND_NONE, NETWORK_ID_ANY, &packet_header, NULL,
            NETWORK_TIMEOUT_CHECK);
    }

    return error != NETWORK_TIMEOUT_EXCEEDED;
}

uint8_t network_get_address()
{
    NETWORK_CHECK_IS_INITIALIZED
    return network_conn.address;
}

uint8_t network_get_response(uint8_t address, uint8_t command,
    const uint8_t *request_data, uint8_t request_length,
    uint8_t **response_data, uint8_t *response_length, int16_t timeout)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header response_packet;

    NETWORK_CHECK_IS_INITIALIZED

    error = network_send(address, NETWORK_STATUS_REQUEST, command,
        request_data, request_length);
    if (error == NETWORK_NO_ERROR && address != NETWORK_ADDRESS_BROADCAST)
    {
        error = network_wait_for_packet(address, NETWORK_STATUS_RESPONSE,
            command, NETWORK_ID_ANY, &response_packet, response_data, timeout);
        if (error == NETWORK_NO_ERROR && response_data != NULL)
        {
            *response_length = response_packet.length;
        }
    }

    return error;
}

uint8_t network_get_request(struct network_request_data *request,
    int16_t timeout)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header request_packet;

    NETWORK_CHECK_IS_INITIALIZED

    error = network_wait_for_packet(NETWORK_ADDRESS_NONE,
        NETWORK_STATUS_REQUEST, NETWORK_COMMAND_NONE, NETWORK_ID_ANY,
        &request_packet, &request->data, timeout);
    if (error == NETWORK_NO_ERROR)
    {
        request->source = request_packet.source;
        request->destination = request_packet.destination;
        request->command = request_packet.command;
        request->length = request_packet.length;
    }

    return error;
}

uint8_t network_send_response(const struct network_request_data *request,
    const uint8_t *data, uint8_t length)
{
    NETWORK_CHECK_IS_INITIALIZED

    return network_send(request->source, NETWORK_STATUS_RESPONSE,
        request->command, data, length);
}

uint8_t network_send(uint8_t destination, uint8_t status, uint8_t command,
    const uint8_t *data, uint8_t length)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header packet_to_send;
    uint8_t attempts = 0;

    NETWORK_CHECK_IS_INITIALIZED

    packet_to_send.destination = destination;
    packet_to_send.source = network_conn.address;
    packet_to_send.status = status;
    packet_to_send.command = command;
    packet_to_send.id = ++network_conn.last_id;
    packet_to_send.length = length;

    do
    {
        error = network_write_packet(&packet_to_send, data);
        if (error == NETWORK_NO_ERROR)
        {
            error = network_wait_for_acknowledge(&packet_to_send);
        }
        attempts++;
    } while (error == NETWORK_TIMEOUT_EXCEEDED
        && attempts < NETWORK_SEND_ATTEMPTS);
    return error;
}

uint8_t network_wait_for_acknowledge(
    const struct network_packet_header *packet)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header acknowledge_packet;

    NETWORK_CHECK_IS_INITIALIZED

    if (packet->status != NETWORK_STATUS_CHECK
        && packet->status != NETWORK_STATUS_ACKNOWLEDGE) {
        error = network_wait_for_packet(packet->destination,
            NETWORK_STATUS_ACKNOWLEDGE, packet->command, packet->id,
            &acknowledge_packet, NULL, NETWORK_TIMEOUT_ACKNOWLEDGE);
    }
    return error;
}

uint8_t network_wait_for_packet(uint8_t source, uint8_t status,
    uint8_t command, uint8_t id, struct network_packet_header *packet_header,
    uint8_t **packet_data, int16_t timeout)
{
    uint8_t error = NETWORK_NO_ERROR;

    NETWORK_CHECK_IS_INITIALIZED

    do
    {
        error = network_get_last_packet(packet_header, packet_data);
        if (error == NETWORK_NO_ERROR)
        {
            if (packet_header != NULL
                && ((source != NETWORK_ADDRESS_NONE
                && source != packet_header->source)
                || (status != NETWORK_STATUS_ANY
                && status != packet_header->status)
                || (command != NETWORK_COMMAND_NONE
                && command != packet_header->command)
                || (id != NETWORK_ID_ANY && id != packet_header->id)))
            {
                error = NETWORK_NO_DATA;
            }
        }
        if (error == NETWORK_NO_DATA)
        {
            _delay_ms(1);
        }
    } while (error == NETWORK_NO_DATA
        && (timeout == NETWORK_TIMEOUT_INFINITE || timeout-- > 0));

    if (error == NETWORK_NO_ERROR)
    {
        network_acknowledge_packet(packet_header);
    }
    else if (timeout < 0)
    {
        error = NETWORK_TIMEOUT_EXCEEDED;
    }
    return error;
}

uint8_t network_get_last_packet(struct network_packet_header *packet_header,
    uint8_t **packet_data)
{
    uint8_t error = NETWORK_NO_ERROR;

    NETWORK_CHECK_IS_INITIALIZED

    if (network_conn.is_packet_avaiable)
    {
        if (packet_header != NULL)
        {
            memcpy(packet_header, &network_conn.last_packet_header,
            sizeof(struct network_packet_header));
            if (packet_data != NULL)
                *packet_data = network_conn.last_packet_data;
        }
        else
        {
            network_free_data(network_conn.last_packet_data);
            network_conn.last_packet_data = NULL;
            network_conn.is_packet_avaiable = false;
        }
    }
    else
    {
        error = NETWORK_NO_DATA;
    }
    return error;
}

uint8_t network_acknowledge_packet(const struct network_packet_header *packet)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header acknowledge_packet;

    NETWORK_CHECK_IS_INITIALIZED

    if (packet != NULL && packet->source != NETWORK_ADDRESS_BROADCAST
        && packet->status != NETWORK_STATUS_ACKNOWLEDGE)
    {
        acknowledge_packet.destination = packet->source;
        acknowledge_packet.source = network_conn.address;
        acknowledge_packet.status = NETWORK_STATUS_ACKNOWLEDGE;
        acknowledge_packet.command = packet->command;
        acknowledge_packet.id = packet->id;
        acknowledge_packet.length = 0;
        error = network_write_packet(&acknowledge_packet, NULL);
    }
    return error;
}

uint8_t network_write_packet(struct network_packet_header *packet_header,
    const uint8_t *packet_data)
{
    uint8_t error = NETWORK_NO_ERROR;
    uint8_t attempts = 0;

    NETWORK_CHECK_IS_INITIALIZED

    packet_header->checksum = network_calculate_checksum(packet_header);

    do
    {
        error = network_write_bytes((const uint8_t*)packet_header,
        sizeof(struct network_packet_header));
        if (error == NETWORK_NO_ERROR)
        {
            error = network_write_bytes(packet_data, packet_header->length);
        }

        if (error == NETWORK_COLLISION_DETECTED)
        {
            network_wait_after_collision();
        }
        attempts++;
    } while (error == NETWORK_WRITE_ERROR
        && attempts < NETWORK_WRITE_PACKET_ATTEMPTS);
    return error;
}

void network_wait_after_collision(void)
{
	// TODO: implement
	// use address and key as seed for random wait time
}

uint8_t network_process_byte(uint8_t error, uint8_t data)
{
    static uint8_t packet_index = 0;
    static struct network_packet_header packet_header;
    static uint8_t *packet_data = NULL;
    uint8_t bytes_to_skip = 0;
    bool reset = false;

    if (error == NETWORK_NO_ERROR)
    {
        if (packet_index < sizeof(struct network_packet_header))
        {
            ((uint8_t*)&packet_header)[packet_index++] = data;

            if (packet_index == 2)
            {
                if (packet_header.destination != network_conn.address
                    && packet_header.destination != NETWORK_ADDRESS_BROADCAST)
                {
                    error = NETWORK_INVALID_PACKET;
                }
            }
            else if (packet_index == sizeof(struct network_packet_header))
            {
                if (network_calculate_checksum(&packet_header)
                    != packet_header.checksum)
                {
                    error = NETWORK_INVALID_PACKET;
                }
            }
        }
        else if ((packet_index - sizeof(struct network_packet_header))
            < packet_header.length)
        {
            if (packet_data == NULL)
            {
                packet_data = (uint8_t*)malloc(packet_header.length);
            }
            packet_data[packet_index++ - sizeof(struct network_packet_header)]
                = data;
        }
    }

    if (error == NETWORK_NO_ERROR)
    {
        if (packet_index == sizeof(struct network_packet_header)
            + packet_header.length)
        {
            if (packet_header.status == NETWORK_STATUS_CHECK)
            {
                network_send(packet_header.source, NETWORK_STATUS_ACKNOWLEDGE,
                    NETWORK_COMMAND_NONE, NULL, 0);
            }
            else if (!network_conn.is_packet_avaiable)
            {
                memcpy(&network_conn.last_packet_header, &packet_header,
                    sizeof(struct network_packet_header));
                network_conn.last_packet_data = packet_data;
                packet_data = NULL;
                network_conn.is_packet_avaiable = true;
            }
            reset = true;
        }
    }
    else
    {
        bytes_to_skip = sizeof(struct network_packet_header)
            + packet_header.length - packet_index;
        reset = true;
    }

    if (reset)
    {
        packet_index = 0;
    }
    return bytes_to_skip;
}

uint8_t network_calculate_checksum(struct network_packet_header *packet)
{
    uint8_t checksum = 0;
    uint8_t packet_checksum = packet->checksum;
    uint8_t *current;

    packet->checksum = 0;
    for (current = (uint8_t*)packet; current
        < (uint8_t*)packet + sizeof(struct network_packet_header); current++)
    {
        checksum ^= *current;
    }
    packet->checksum = packet_checksum;
    return checksum;
}

void network_free_data(uint8_t *data)
{
    free(data);
}

void network_initialize_external_int(void)
{
    network_disable_external_int();

    // set interrupt on fallig edge
    MCUCR &= ~(1 << NETWORK_ISC0);
    MCUCR |= (1 << NETWORK_ISC1);
}

void network_disable_external_int(void)
{
    // clear external interrupt flag
    GICR &= ~(1 << NETWORK_INT);
}

void network_enable_external_int(void)
{
    // clear eventual external register flag
    GIFR = (1 << NETWORK_INTF);

    // set external interrupt flag
    GICR |= (1 << NETWORK_INT);
}

void network_initialize_timer_int(void)
{
    network_disable_timer_ovf_int();
    network_disable_timer_cpm_int();

    // set normal mode, 1024 prescaler
    NETWORK_TCCR = (1 << NETWORK_CS2 | 1 << NETWORK_CS0);

    // compare match, used by read
    NETWORK_OCR = 127;
}

void network_disable_timer_ovf_int(void)
{
    // clear timer overflow interrupt enable flag
    TIMSK &= ~(1 << NETWORK_TOIE);
}

void network_disable_timer_cpm_int(void)
{
    // clear timer compare match interrupt enable flag
    TIMSK &= ~(1 << NETWORK_OCIE);
}

void network_enable_timer_ovf_int(void)
{
    // reset counter
    NETWORK_TCNT = 0;

    // clear eventual overflow register flag
    TIFR = (1 << NETWORK_TOV);

    // set timer overflow interrupt enable flag
    TIMSK |= (1 << NETWORK_TOIE);
}

void network_enable_timer_cpm_int(void)
{
    // reset counter
    NETWORK_TCNT = 0;

    // clear eventual compare register flag
    TIFR = (1 << NETWORK_OCF);

    // set timer compare match interrupt enable flag
    TIMSK |= (1 << NETWORK_OCIE);
}

uint8_t network_write_bytes(const uint8_t *data, uint8_t length)
{
    uint8_t error = NETWORK_NO_ERROR;
    const uint8_t *current;

    for (current = data; error == NETWORK_NO_ERROR && current < data + length;
        current++)
    {
        error = network_write_byte(*current);
    }

    return error;
}

uint8_t network_write_byte(uint8_t byte)
{
    while (network_timer_int_mode != NETWORK_TIMER_INTERRUPT_MODE_NONE);

    network_disable_external_int();

    if (network_timer_int_mode != NETWORK_TIMER_INTERRUPT_MODE_NONE)
        return NETWORK_WRITE_ERROR;

    network_timer_int_mode = NETWORK_TIMER_INTERRUPT_MODE_WRITE;
    network_timer_int_byte = byte;

    network_set_port_mode(false);
    network_enable_timer_ovf_int();

	return NETWORK_NO_ERROR;
}

ISR(NETWORK_INT_VECT)
{
    network_disable_external_int();

    // if somehow a second external interrupt occurs while still processing
    // the last, terminate
    if (network_timer_int_mode != NETWORK_TIMER_INTERRUPT_MODE_NONE)
        return;

    network_timer_int_mode = NETWORK_TIMER_INTERRUPT_MODE_READ;
    network_timer_int_byte = 0;

    // use compare match with 127 to start reading in the middle of a bit
    network_enable_timer_cpm_int();
}

ISR(NETWORK_TIMER_COMP_VECT)
{
    network_disable_timer_cpm_int();

    network_enable_timer_ovf_int();
}

ISR(NETWORK_TIMER_OVF_VECT)
{
    static uint8_t index = 0;
    uint8_t bit = 0;

    if (network_timer_int_mode == NETWORK_TIMER_INTERRUPT_MODE_READ)
    {
        bit = NETWORK_PIN & (1 << NETWORK_PORT_PIN);
        bit >>= NETWORK_PORT_PIN;
        network_timer_int_byte <<= 1;
        network_timer_int_byte |= bit;

        if (index == 7)
        {
            network_process_byte(NETWORK_NO_ERROR, network_timer_int_byte);
        }
    }
    else if (network_timer_int_mode == NETWORK_TIMER_INTERRUPT_MODE_WRITE)
    {
        bit = network_timer_int_byte & 0b10000000;
        network_timer_int_byte <<= 1;
        network_write_bit(bit);
    }

    if (++index == 8)
    {
        network_timer_int_mode = NETWORK_TIMER_INTERRUPT_MODE_NONE;
        network_timer_int_byte = 0;
        index = 0;

        network_disable_timer_ovf_int();
        network_set_port_mode(true);
        network_enable_external_int();
    }
}

void network_set_port_mode(bool read)
{
    if (read)
    {
        // if current state is output low, output high must be used as a
        // intermediate step
        if ((NETWORK_DDR & (1 << NETWORK_PORT_PIN))
            && !(NETWORK_PORT & (1 << NETWORK_PORT_PIN)))
        {
            // set output high
            NETWORK_PORT |= (1 << NETWORK_PORT_PIN);
        }

        // set input pull-up
        NETWORK_DDR &= ~(1 << NETWORK_PORT_PIN);
        NETWORK_PORT |= (1 << NETWORK_PORT_PIN);
    }
    else
    {
        // if current status is tri-state, input pull-up must be used as a
        // intermediate step
        if (!(NETWORK_DDR & (1<<NETWORK_PORT_PIN))
            && !(NETWORK_PORT & (1<<NETWORK_PORT_PIN)))
        {
            // set input pull-up
            NETWORK_PORT |= (1<<NETWORK_PORT_PIN);
        }

        // set output high
        NETWORK_DDR |= (1 << NETWORK_PORT_PIN);
        NETWORK_PORT |= (1 << NETWORK_PORT_PIN);
    }
}

uint8_t network_write_bit(uint8_t bit)
{
    uint8_t error = NETWORK_NO_ERROR;

    if (bit)
    {
        NETWORK_PORT |= (1 << NETWORK_PORT_PIN);
    }
    else
    {
        NETWORK_PORT &= ~(1 << NETWORK_PORT_PIN);
    }

    if (bit != (NETWORK_VALIDATE_PORT & (1<< NETWORK_VALIDATE_DB)))
        error = NETWORK_COLLISION_DETECTED;

    return error;
}
