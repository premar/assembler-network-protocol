/******************************************************************************
* File:         network.c
*
* Description:  Implementation of the Network protocol for communication
*               between microcontrollers on a bus.
*
* Author:       Marius Preuss, Viktor Puselja
*
* Date:         2014-01-21
*
******************************************************************************/

#include "network.h"
#include "network_hardware.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// network status
#define NETWORK_STATUS_CHECK        0x00
#define NETWORK_STATUS_REQUEST      0x01
#define NETWORK_STATUS_RESPONSE     0x02
#define NETWORK_STATUS_ACKNOWLEDGE  0x03
#define NETWORK_STATUS_ANY          0xFF

// network timeouts
#define NETWORK_TIMEOUT_INFINITE    ((int16_t)UINT16_MAX)
#define NETWORK_TIMEOUT_CHECK       2000
#define NETWORK_TIMEOUT_ACKNOWLEDGE 2000

// network attempts
#define NETWORK_SEND_ATTEMPTS        3

// network process modes
#define NETWORK_PROCESS_MODE_PROCESS 0
#define NETWORK_PROCESS_MODE_RESET   1
#define NETWORK_PROCESS_MODE_SKIP    2

// represents the header of a network packet
struct network_packet_header {
  uint8_t destination;
  uint8_t length;
  uint8_t source;
  uint8_t status;
  uint8_t id;
  uint8_t checksum;
  uint8_t command;
};

// represents a network connection
struct network_connection
{
    uint8_t address;
    uint8_t last_id;
    bool is_packet_avaiable;
    struct network_packet_header last_packet_header;
    uint8_t *last_packet_data;
    bool is_sending_packet;
    struct network_packet_header packet_to_send_header;
    const uint8_t *packet_to_send_data;
};

volatile struct network_connection network_conn;

/*
 * Function: network_receive()
 * Description: Waits for a packet from the specified source and with the
 *              specified status and command. Other packets are discarded.
 */
uint8_t network_receive(uint8_t source_filter, uint8_t status_filter,
    uint8_t command_filter, uint8_t id_filter, uint8_t *source,
    uint8_t *destination, uint8_t *status, uint8_t* command, uint8_t *id,
    uint8_t **packet_data, uint8_t *length, int16_t timeout);

/*
 * Function: network_send()
 * Description: Sends a packet to the specified destination and with the
 *              specified status, command and data.
 */
uint8_t network_send(uint8_t destination, uint8_t status, uint8_t command,
    const uint8_t *data, uint8_t length, bool blocking);

/*
 * Function: network_read_packet()
 * Description: Reads a packet from the input buffer.
 */
uint8_t network_read_packet(uint8_t source, uint8_t status, uint8_t command,
    uint8_t id, struct network_packet_header *packet_header,
    uint8_t **packet_data, int16_t timeout);

/*
 * Function: network_write_packet()
 * Description: Writes a packet to the output buffer.
 */
void network_write_packet(struct network_packet_header *packet_header,
    const uint8_t *packet_data, bool blocking);

/*
 * Function: network_calculate_checksum()
 * Description: Returns the checksum of the packet.
 */
uint8_t network_calculate_checksum(struct network_packet_header *packet);

uint8_t network_initialize(void)
{
    uint8_t error = NETWORK_NO_ERROR;
    bool is_address_used = true;
    uint8_t address = NETWORK_ADDRESS_MIN;

    network_conn.address = NETWORK_ADDRESS_NONE;
    network_conn.last_id = 0;
    network_conn.is_packet_avaiable = false;
    
    network_initialize_hardware();

    while (address < NETWORK_ADDRESS_MAX && is_address_used)
        is_address_used = network_check(address++);

    if (!is_address_used)
        network_conn.address = --address;
    else
        error = NETWORK_NONE_ADDRESS_AVAILABLE;

    return error;
}

bool network_check(uint8_t address)
{
    uint8_t error = NETWORK_NO_ERROR;
    
    error = network_send(address, NETWORK_STATUS_CHECK, NETWORK_COMMAND_NONE,
        NULL, 0, false);

    return error != NETWORK_TIMEOUT_EXCEEDED;
}

uint8_t network_get_response(uint8_t address, uint8_t command,
    const uint8_t *request_data, uint8_t request_length, bool blocking,
    uint8_t **response_data, uint8_t *response_length, int16_t timeout)
{
    uint8_t error = NETWORK_NO_ERROR;

    error = network_send(address, NETWORK_STATUS_REQUEST, command,
        request_data, request_length, blocking);
    if (error == NETWORK_NO_ERROR && address != NETWORK_ADDRESS_BROADCAST)
        error = network_receive(address, NETWORK_STATUS_RESPONSE, command,
            NETWORK_ID_ANY, NULL, NULL, NULL, NULL, NULL, response_data,
           response_length, timeout);

    return error;
}

uint8_t network_get_request(struct network_request_data *request,
    int16_t timeout)
{
    return network_receive(NETWORK_ADDRESS_NONE, NETWORK_STATUS_REQUEST,
        NETWORK_COMMAND_NONE, NETWORK_ID_ANY, &request->source,
        &request->destination, NULL, &request->command, NULL, &request->data,
        &request->length, timeout);
}

uint8_t network_send_response(const struct network_request_data *request,
    const uint8_t *data, uint8_t length, bool blocking)
{
    return network_send(request->source, NETWORK_STATUS_RESPONSE,
        request->command, data, length, blocking);
}

void network_free_data(uint8_t *data)
{
    free(data);
}

uint8_t network_receive(uint8_t source_filter, uint8_t status_filter,
    uint8_t command_filter, uint8_t id_filter, uint8_t *source,
    uint8_t *destination, uint8_t *status, uint8_t* command, uint8_t *id,
    uint8_t **packet_data, uint8_t *length, int16_t timeout)
{
    struct network_packet_header packet_header;
    uint8_t error = NETWORK_NO_ERROR;

    error = network_read_packet(source_filter, status_filter, command_filter,
        id_filter, &packet_header, packet_data, timeout);
    if (error == NETWORK_NO_ERROR) {
        if (source != NULL)
            *source = packet_header.source;
        if (destination != NULL)
            *destination = packet_header.destination;
        if (status != NULL)
            *status = packet_header.status;
        if (command != NULL)
            *command = packet_header.command;
        if (id != NULL)
            *id = packet_header.id;
        if (length != NULL)
            *length = packet_header.length;
            
        if (packet_header.source != NETWORK_ADDRESS_BROADCAST &&
            packet_header.status != NETWORK_STATUS_ACKNOWLEDGE) {
            packet_header.destination = packet_header.source;
            packet_header.source = network_conn.address;
            packet_header.status = NETWORK_STATUS_ACKNOWLEDGE;
            packet_header.length = 0;
            network_write_packet(&packet_header, NULL, false);
        }
    }
    
    return error;
}

uint8_t network_send(uint8_t destination, uint8_t status, uint8_t command,
    const uint8_t *data, uint8_t length, bool blocking)
{
    uint8_t error = NETWORK_NO_ERROR;
    struct network_packet_header packet_to_send;
    uint8_t id;
    uint8_t attempts = 0;

    if (status == NETWORK_STATUS_CHECK || status == NETWORK_STATUS_REQUEST
        || status == NETWORK_STATUS_RESPONSE) {
        packet_to_send.destination = destination;
        packet_to_send.source = network_conn.address;
        packet_to_send.status = status;
        packet_to_send.command = command;
        id = ++network_conn.last_id;
        packet_to_send.id = id;
        packet_to_send.length = length;

        do {
            network_write_packet(&packet_to_send, data, blocking);
            if (destination != NETWORK_ADDRESS_BROADCAST)
                error = network_read_packet(destination,
                NETWORK_STATUS_ACKNOWLEDGE, command, id, NULL, NULL,
                NETWORK_TIMEOUT_ACKNOWLEDGE);
				PORTA = ~error;
        } while (error == NETWORK_TIMEOUT_EXCEEDED
            && ++attempts < NETWORK_SEND_ATTEMPTS);
    } else {
        error = NETWORK_INVALID_STATUS;
    }
    
    return error;
}

uint8_t network_read_packet(uint8_t source, uint8_t status, uint8_t command,
    uint8_t id, struct network_packet_header *packet_header,
    uint8_t **packet_data, int16_t timeout)
{
    bool discard;
    bool is_avaiable;
    
    do {
        discard = false;
        is_avaiable = false;
        
        if (network_conn.is_packet_avaiable) {
            if ((source != NETWORK_ADDRESS_NONE
                && source != network_conn.last_packet_header.source)
                || (status != NETWORK_STATUS_ANY
                && status != network_conn.last_packet_header.status)
                || (command != NETWORK_COMMAND_NONE
                && command != network_conn.last_packet_header.command)
                || (id != NETWORK_ID_ANY
                && id != network_conn.last_packet_header.id)) {
                discard = true;
            } else {
                discard = packet_header == NULL;
                is_avaiable = true;
            }
            
            if (!discard)
                memcpy(packet_header, &network_conn.last_packet_header,
                    sizeof(struct network_packet_header));
            
            if (!discard && packet_data != NULL)
                *packet_data = network_conn.last_packet_data;
            else
                free(network_conn.last_packet_data);
            
            network_conn.last_packet_data = NULL;
            network_conn.is_packet_avaiable = false;
        }
        
        if (!is_avaiable)
            network_delay_ms();
            
    } while (!is_avaiable && (timeout == NETWORK_TIMEOUT_INFINITE
        || timeout-- > 0));
        
    return is_avaiable ? NETWORK_NO_ERROR : NETWORK_TIMEOUT_EXCEEDED;
}

void network_write_packet(struct network_packet_header *packet_header,
    const uint8_t *packet_data, bool blocking)
{
    packet_header->checksum = network_calculate_checksum(packet_header);

    while (network_conn.is_sending_packet);
    
    network_conn.is_sending_packet = true;
    memcpy(&network_conn.packet_to_send_header, packet_header,
        sizeof(struct network_packet_header));
    network_conn.packet_to_send_data = packet_data;
    
    network_start_writing();
    
    if (blocking)
        network_wait_writing_complete();
}

void network_process_byte(uint8_t error, uint8_t data)
{
    static uint8_t packet_header_index = 0;
    static uint8_t packet_data_index = 0;
    static struct network_packet_header packet_header;
    static uint8_t *packet_data = NULL;
    static uint8_t bytes_to_skip = 0;
    uint8_t mode = NETWORK_PROCESS_MODE_PROCESS;
	uint8_t need_write_call = false;

    if (error == NETWORK_NO_ERROR) {
        if (bytes_to_skip > 0) {
            mode = NETWORK_PROCESS_MODE_SKIP;
            if (--bytes_to_skip == 0)
                mode = NETWORK_PROCESS_MODE_RESET;
        }
    } else {
        mode = NETWORK_PROCESS_MODE_RESET;
    }
    
    if (mode == NETWORK_PROCESS_MODE_PROCESS) {
        if (packet_header_index < sizeof(struct network_packet_header)) {
            *(((uint8_t*)&packet_header) + packet_header_index++) = data;

            if (packet_header_index == 2
                && packet_header.destination != network_conn.address
                && packet_header.destination != NETWORK_ADDRESS_BROADCAST)
                mode = NETWORK_PROCESS_MODE_SKIP;
            else if (packet_header_index
                == sizeof(struct network_packet_header)
                && network_calculate_checksum(&packet_header)
                != packet_header.checksum)
                mode = NETWORK_PROCESS_MODE_SKIP;
            
            if (mode == NETWORK_PROCESS_MODE_SKIP)
                bytes_to_skip = sizeof(struct network_packet_header)
                    + packet_header.length - packet_header_index;
        } else if (packet_data_index < packet_header.length) {
            if (packet_data == NULL)
                packet_data = (uint8_t*)malloc(packet_header.length);
            packet_data[packet_data_index++] = data;
        }
    }
    
    if (mode == NETWORK_PROCESS_MODE_PROCESS
        && packet_header_index + packet_data_index
        == sizeof(struct network_packet_header) + packet_header.length) {
        if (packet_header.status == NETWORK_STATUS_CHECK) {
            packet_header.destination = packet_header.source;
            packet_header.source = network_conn.address;
            packet_header.status = NETWORK_STATUS_ACKNOWLEDGE;
            packet_header.length = 0;
            // does not work currently, check request is not acknowledged
            // network_write_packet(&packet_header, NULL, false);
        } else if (!network_conn.is_packet_avaiable) {
            memcpy(&network_conn.last_packet_header, &packet_header,
                sizeof(struct network_packet_header));
            network_conn.last_packet_data = packet_data;
            packet_data = NULL;
            network_conn.is_packet_avaiable = true;
        }
        mode = NETWORK_PROCESS_MODE_RESET;
    }
    
    if (mode == NETWORK_PROCESS_MODE_RESET) {
        packet_header_index = 0;
        packet_data_index = 0;
        bytes_to_skip = 0;
    }
}

bool network_get_next_byte(uint8_t *byte)
{
    static uint8_t packet_index = 0;
    bool has_byte = false;
    
    if (network_conn.is_sending_packet) {
        if (packet_index < sizeof(struct network_packet_header))
            *byte = *(((uint8_t*)&network_conn.packet_to_send_header)
                + packet_index++);
        else if (packet_index - sizeof(struct network_packet_header)
            < network_conn.packet_to_send_header.length)
            *byte = network_conn.packet_to_send_data[packet_index++
                - sizeof(struct network_packet_header)];
        
        if (packet_index == sizeof(struct network_packet_header)
            + network_conn.packet_to_send_header.length) {
            network_conn.is_sending_packet = false;
            packet_index = 0;
        }
        
        has_byte = true;
    }
    
    return has_byte;
}

uint8_t network_calculate_checksum(struct network_packet_header *packet)
{
    uint8_t checksum = 0;
    uint8_t packet_checksum = packet->checksum;
    uint8_t *current;
    uint8_t *packet_end;

    packet->checksum = 0;
    packet_end = (uint8_t*)packet + sizeof(struct network_packet_header);
    for (current = (uint8_t*)packet; current < packet_end; current++)
        checksum ^= *current;
    packet->checksum = packet_checksum;
    return checksum;
}
