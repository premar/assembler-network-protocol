/******************************************************************************
 * File:        network.c
 *
 * Description: Code file of the network api for communication between
 *              multiple microcontrollers.
 *
 * Author:      Marius Preuss, Viktor Puselja
 *
 * Date:        2014-01-21
 *
******************************************************************************/

// TODO / Improvements :
// - implement wait_after_collision
// - write_bytes with two source buffer (malloc)
// - implement max packet_size option (security)
// - optimize if statements
// - check malloc return value

#include "protocol.h"

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Volatile Variable
volatile uint8_t sendMode = false;
volatile uint8_t readMode = false;

// Network Attempts
#define NETWORK_WRITE_PACKET_ATTEMPTS   5
#define NETWORK_SEND_ATTEMPTS           3

// Represent a network connection.
struct network_connection {
  uint8_t address;
  uint8_t last_id;
  bool is_packet_avaiable;
  struct network_packet_header last_packet_header;
  uint8_t *last_packet_data;
  bool is_initialized;
};

// The global network connection.
struct network_connection network_conn;

#define NETWORK_CHECK_IS_INITIALIZED if (!network_conn.is_initialized)  \
{ return NETWORK_NOT_INITIALIZED; }

int main(void)
{
    sei();
    network_pull_up();
    while(1)
    {
    }
    return 0;
}

uint8_t network_initialize(void)
{
  uint8_t error = NETWORK_NO_ERROR;
  bool is_address_used = true;
  uint8_t address;

  if (!network_conn.is_initialized) {
    // initialize connection
    network_conn.address = NETWORK_ADDRESS_NONE;
    network_conn.last_id = 0;
    network_conn.is_packet_avaiable = false;
    network_conn.is_initialized = true;

    // determine address
    address = NETWORK_ADDRESS_MIN - 1;
    do
    {
      address++;
      is_address_used = network_check(address);
    } while (address <= NETWORK_ADDRESS_MAX && is_address_used);

    if (!is_address_used) {
      network_conn.address = address;
      //network_hello();
    } else {
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

  // send check
  error = network_send(address, NETWORK_STATUS_CHECK, NETWORK_COMMAND_NONE,
    NULL, 0);
  if (error == NETWORK_NO_ERROR) {
    // wait for check acknowledge
    error = network_wait_for_packet(address, NETWORK_STATUS_ACKNOWLEDGE,
      NETWORK_COMMAND_NONE, NETWORK_ID_ANY, &packet_header, NULL, NETWORK_TIMEOUT_CHECK);
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
  uint8_t **response_data, uint8_t *response_length,
  int16_t timeout)
{
  uint8_t error = NETWORK_NO_ERROR;
  struct network_packet_header response_packet;

  NETWORK_CHECK_IS_INITIALIZED

  // send request
  error = network_send(address, NETWORK_STATUS_REQUEST, command, request_data,
    request_length);
  if (error == NETWORK_NO_ERROR && address != NETWORK_ADDRESS_BROADCAST) {
    // receive response
    error = network_wait_for_packet(address, NETWORK_STATUS_RESPONSE, command,
      NETWORK_ID_ANY, &response_packet, response_data, timeout);
    if (error == NETWORK_NO_ERROR && response_data != NULL)
      *response_length = response_packet.length;
  }

  return error;
}

uint8_t network_get_request(struct network_request_data *request,
  int16_t timeout)
{
  uint8_t error = NETWORK_NO_ERROR;
  struct network_packet_header request_packet;

  NETWORK_CHECK_IS_INITIALIZED

  // receive request
  error = network_wait_for_packet(NETWORK_ADDRESS_NONE, NETWORK_STATUS_REQUEST,
    NETWORK_COMMAND_NONE, NETWORK_ID_ANY, &request_packet, &request->data,
    timeout);
  if (error == NETWORK_NO_ERROR) {
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

  // send response
  return network_send(request->source, NETWORK_STATUS_RESPONSE,
    request->command, data, length);
}

uint8_t network_send(uint8_t destination, uint8_t status,
  uint8_t command, const uint8_t *data, uint8_t length)
{
  uint8_t error = NETWORK_NO_ERROR;
  struct network_packet_header packet_to_send;
  uint8_t attempts = 0;

  NETWORK_CHECK_IS_INITIALIZED

  // build packet
  packet_to_send.destination = destination;
  packet_to_send.source = network_conn.address;
  packet_to_send.status = status;
  packet_to_send.command = command;
  packet_to_send.id = ++network_conn.last_id;
  packet_to_send.length = length;

  // send
  do {
    error = network_write_packet(&packet_to_send, data);
    if (error == NETWORK_NO_ERROR)
        error = network_wait_for_acknowledge(&packet_to_send);

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

  // check and acknowledge packets must not be awaited for acknowledge
  if (packet->status != NETWORK_STATUS_CHECK && packet->status
    != NETWORK_STATUS_ACKNOWLEDGE)
    error = network_wait_for_packet(packet->destination,
      NETWORK_STATUS_ACKNOWLEDGE, packet->command, packet->id,
      &acknowledge_packet, NULL, NETWORK_TIMEOUT_ACKNOWLEDGE);

  // - implement max packet_size option (security)
  // add NETWORK_STATUS_DATA_TO_BIG to check
  // change network_wait_for_packet to wait for DATA_TO_BIG too
  // return error if DATA_TO_BIG is returned

  return error;
}

uint8_t network_wait_for_packet(uint8_t source, uint8_t status,
  uint8_t command, uint8_t id, struct network_packet_header *packet_header,
  uint8_t **packet_data, int16_t timeout)
{
  uint8_t error = NETWORK_NO_ERROR;

  NETWORK_CHECK_IS_INITIALIZED

  do {
    error = network_get_last_packet(packet_header, packet_data);
    if (error == NETWORK_NO_ERROR) {
      // check if it is the desired packet
      if (packet_header != NULL &&
        ((source != NETWORK_ADDRESS_NONE && source != packet_header->source)
        || (status != NETWORK_STATUS_ANY && status != packet_header->status)
        || (command != NETWORK_COMMAND_NONE
        && command != packet_header->command)
        || (id != NETWORK_ID_ANY && id != packet_header->id)))
          error = NETWORK_NO_DATA;
    }

    if (error == NETWORK_NO_DATA) {
      _delay_ms(1);
    }
  } while (error == NETWORK_NO_DATA
    && (timeout == NETWORK_TIMEOUT_INFINITE || timeout-- > 0));

  if (error == NETWORK_NO_ERROR)
    network_acknowledge_packet(packet_header);
  else if (timeout < 0)
    error = NETWORK_TIMEOUT_EXCEEDED;

  return error;
}

uint8_t network_get_last_packet(struct network_packet_header *packet_header,
  uint8_t **packet_data)
{
  uint8_t error = NETWORK_NO_ERROR;

  NETWORK_CHECK_IS_INITIALIZED

  // copy packet if avaiable
  if (network_conn.is_packet_avaiable) {
    if (packet_header != NULL)
      memcpy(packet_header, &network_conn.last_packet_header,
        sizeof(struct network_packet_header));
    if (packet_data != NULL)
      *packet_data = network_conn.last_packet_data;
    else
      network_free_data(network_conn.last_packet_data);
    network_conn.last_packet_data = NULL;
    network_conn.is_packet_avaiable = false;
  } else {
    error = NETWORK_NO_DATA;
  }

  return error;
}

uint8_t network_acknowledge_packet(
  const struct network_packet_header *packet)
{
  uint8_t error = NETWORK_NO_ERROR;
  struct network_packet_header acknowledge_packet;

  NETWORK_CHECK_IS_INITIALIZED

  // broadcast and acknowledge packets must not be acknowledged
  if (packet != NULL && packet->source != NETWORK_ADDRESS_BROADCAST
    && packet->status != NETWORK_STATUS_ACKNOWLEDGE) {
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
  uint8_t *data;
  uint8_t attempts = 0;

  packet_header->checksum = network_calculate_checksum(packet_header);
  // copy header and data into buffer
  data = (uint8_t*)malloc(sizeof(struct network_packet_header)
    + packet_header->length);
  memcpy(data, packet_header, sizeof(struct network_packet_header));
  if (packet_header->length > 0)
    memcpy(data + sizeof(struct network_packet_header), packet_data,
      packet_header->length);

  // write buffer to the network
  do {
    error = network_write_bytes(data, sizeof(struct network_packet_header)
      + packet_header->length);
    if (error == NETWORK_COLLISION_DETECTED) {
      //network_wait_after_collision();
    }
    attempts++;
  } while (error == NETWORK_COLLISION_DETECTED
    && attempts < NETWORK_WRITE_PACKET_ATTEMPTS);

  free(data);

  return error;
}

// - write_bytes with two source buffer (malloc)
//uint8_t network_write_packet(struct network_packet_header *packet_header,
//  const uint8_t *packet_data)
//{
//  uint8_t error = NETWORK_NO_ERROR;
//  uint8_t *data;
//  uint8_t attempts = 0;
//
//  packet_header->checksum = network_calculate_checksum(packet_header);
//
//  // write buffer to the network
//  do {
//    error = network_write_bytes((const uint8_t*)packet_header,
//      sizeof(struct network_packet_header), data, packet_header->length);
//    if (error == NETWORK_COLLISION_DETECTED) {
//      network_signal_collision();
//      network_wait_after_collision();
//    }
//    attempts++;
//  } while (error == NETWORK_COLLISION_DETECTED
//    && attempts < NETWORK_WRITE_PACKET_ATTEMPTS);
//
//  return error;
//}

uint8_t network_process_byte(uint8_t error, uint8_t data)
{
  static uint8_t packet_index = 0;
  static struct network_packet_header packet_header;
  static uint8_t *packet_data = NULL;
  uint8_t bytes_to_skip = 0;
  bool reset = false;

  if (error == NETWORK_NO_ERROR) {
    if (packet_index < sizeof(struct network_packet_header)) {
      // write byte to header
      ((uint8_t*)&packet_header)[packet_index++] = data;

      if (packet_index == 2) {
        // when destination and length read check destination address
        if (packet_header.destination != network_conn.address
          && packet_header.destination != NETWORK_ADDRESS_BROADCAST)
          error = NETWORK_INVALID_PACKET;

        // - implement max packet_size option (security)
        //else if (packet_header.length > NETWORK_MAX_DATA_SIZE) {
        //  network_send(packet_header.source, NETWORK_STATUS_DATA_TO_BIG,
        //    NETWORK_COMMAND_NONE, NULL, 0);
        //  error = NETWORK_INVALID_PACKET;
        //}
      }
      else if (packet_index == sizeof(struct network_packet_header)) {
        // when header read check checksum
        if (network_calculate_checksum(&packet_header)
          != packet_header.checksum)
          error = NETWORK_INVALID_PACKET;
      }
    }
    else if (packet_index - sizeof(struct network_packet_header)
      < packet_header.length) {
      if (packet_data == NULL)
        packet_data = (uint8_t*)malloc(packet_header.length);
      // write byte to data
      packet_data[packet_index++ - sizeof(struct network_packet_header)]
        = data;
    }
  }

  if (error == NETWORK_NO_ERROR) {
    if (packet_index == sizeof(struct network_packet_header)
      +  packet_header.length) { // packet read
      if (packet_header.status == NETWORK_STATUS_CHECK) {
        // respond to check
        network_send(packet_header.source, NETWORK_STATUS_ACKNOWLEDGE,
          NETWORK_COMMAND_NONE, NULL, 0);
      } else if (!network_conn.is_packet_avaiable) {
        memcpy(&network_conn.last_packet_header, &packet_header,
          sizeof(struct network_packet_header));
        network_conn.last_packet_data = packet_data;
        packet_data = NULL;
        network_conn.is_packet_avaiable = true;
      }

      reset = true;
    }
  }
  else {
    bytes_to_skip = sizeof(struct network_packet_header) + packet_header.length
      - packet_index;
    reset = true;
  }

  if (reset)
    packet_index = 0;

  return bytes_to_skip;
}

uint8_t network_calculate_checksum(struct network_packet_header *packet)
{
  uint8_t checksum = 0;
  uint8_t packet_checksum = packet->checksum;
  uint8_t *current;

  packet->checksum = 0;
  for (current = (uint8_t*)packet;
    current < (uint8_t*)packet + sizeof(struct network_packet_header);
    current++)
    checksum ^= *current;
  packet->checksum = packet_checksum;

  return checksum;
}

void network_free_data(uint8_t *data)
{
  free(data);
}

void network_pull_up()
{
    SEND_DDR &= ~(1 << SEND_DB);
    SEND_PORT |= (1 << SEND_DB);
}

void network_pull_down()
{
    SEND_PORT &= ~(1 << SEND_DB);
}

uint8_t network_write_bytes(const uint8_t *data, uint8_t length)
{
    uint8_t error = NETWORK_NO_ERROR;
    const uint8_t *counter;

    for (counter = data; error == NETWORK_NO_ERROR && counter < data + length; counter++)
        error = network_write_byte(*data);

    return error;
}

uint8_t network_write_byte(uint8_t byte)
{
    cli();

    sendMode = true;

    uint8_t i;
    uint8_t error = NETWORK_NO_ERROR;

    network_send_zero();                         // Impuls das Daten gesendet werden

    do
    {
        if (byte & (1 << i))
        {
            error = network_send_one();
        }
        else
        {
            error = network_send_zero();
        }
        i++;
    } while ((i < 7) && (error == 0));


    // TODO Schlecht impelemtiert -> Bessere Lösung SREG-Sicherung zurückholen
    sei();

    return error;
}

uint8_t network_send_zero()
{
    uint8_t error = NETWORK_NO_ERROR;

    network_pull_down();

    // TODO Ueberpruefung auf SLAM
    _delay_ms(NETWORK_SEND_DELAY);

    network_pull_up();

    return error;
}

uint8_t network_send_one()
{
    uint8_t error = NETWORK_NO_ERROR;

    // TODO Ueberpruefung auf SLAM

    _delay_ms(NETWORK_SEND_DELAY);

    return error;
}

ISR(INT0_vect)
{
    if(readMode == false) {
        // Trigger für Interrupt
        //TCCR2  = (1<<CS22) | (1<<CS21);
        //TIMSK |= (1<<TOIE2);
    }
}

ISR(TIMER1_COMPA_vect) {
    static uint8_t byte = 0;
    static uint8_t counter = 0;

    byte NETWORK_SEND_DB





    // uint8_t network_process_byte(uint8_t error, uint8_t data)
    // TODO Einlesen des Wert
    // TODO Schieben
}
