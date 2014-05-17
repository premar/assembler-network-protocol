/******************************************************************************
 * File:        network.h
 *
 * Description: Header file of the network api for communication between
 *              multiple microcontrollers.
 *
 * Author:      Marius Preuss, Viktor Puselja
 *
 * Date:        2014-01-21
 *
******************************************************************************/

#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdint.h>
#include <stdbool.h>

#define NETWORK_SEND_PORT       PORTA
#define NETWORK_SEND_DDR        DDRA
#define NETWORK_SEND_DB         PD0

/*#define NETWORK_VALIDATE_PORT   PORTA
#define NETWORK_VALIDATE_DDR    DDRA
#define NETWORK_VALIDATE_DB     PD1*/

// Network Address
#define NETWORK_ADDRESS_NONE        0x00
#define NETWORK_ADDRESS_MIN         0x01
#define NETWORK_ADDRESS_MAX         0xFE
#define NETWORK_ADDRESS_BROADCAST   0xFF

// Network Status
#define NETWORK_STATUS_CHECK        0x00
#define NETWORK_STATUS_REQUEST      0x01
#define NETWORK_STATUS_RESPONSE     0x02
#define NETWORK_STATUS_ACKNOWLEDGE  0x03
#define NETWORK_STATUS_ANY          0xFF

// Network Commands
#define NETWORK_COMMAND_NONE        0x00
#define NETWORK_COMMAND_HELLO       0x00

// Network ID
#define NETWORK_ID_ANY              0xFF

// Network Error Codes
#define NETWORK_NO_ERROR                0x00
#define NETWORK_NOT_INITIALIZED         0x01
#define NETWORK_NONE_ADDRESS_AVAILABLE  0x02
#define NETWORK_ADDRESS_IN_USE          0x03
#define NETWORK_WRITE_ERROR				0x10
#define NETWORK_COLLISION_DETECTED      0x11
#define NETWORK_NO_DATA                 0x20
#define NETWORK_TIMEOUT_EXCEEDED        0x21
#define NETWORK_INVALID_PACKET          0x22

// Network Timeouts
#define NETWORK_TIMEOUT_INFINITE    ((int16_t)UINT16_MAX)
#define NETWORK_TIMEOUT_CHECK       200
#define NETWORK_TIMEOUT_ACKNOWLEDGE 100

//#define NETWORK_SEND_DELAY      300

// Network Attempts
#define NETWORK_WRITE_PACKET_ATTEMPTS   5
#define NETWORK_SEND_ATTEMPTS           3

// Network Timer Interrupt Modes
#define NETWORK_TIMER_INTERRUPT_MODE_NONE   0
#define NETWORK_TIMER_INTERRUPT_MODE_READ   1
#define NETWORK_TIMER_INTERRUPT_MODE_WRITE  2

// Represents the data of a request.
struct network_request_data {
  uint8_t source;
  uint8_t destination;
  uint8_t command;
  uint8_t *data;
  uint8_t length;
};

// Represent the header of a network packet.
struct network_packet_header {
  uint8_t destination;
  uint8_t length;
  uint8_t source;
  uint8_t status;
  uint8_t id;
  uint8_t checksum;
  uint8_t command;
};

// Represents a network connection.
struct network_connection 
{
    uint8_t address;
    uint8_t last_id;
    bool is_packet_avaiable;
    struct network_packet_header last_packet_header;
    uint8_t *last_packet_data;
    bool is_initialized;
};

// The global network connection.
extern struct network_connection network_conn;

// Network timer interrupt mode
extern volatile uint8_t network_timer_int_mode;

// Network timer interrupt byte buffer
extern volatile uint8_t network_timer_int_byte;

/*
 * Function: network_initialize()
 * Description: Initializes the network connection and assigns an address.
 */
uint8_t network_initialize(void);

/*
 * Function: network_check()
 * Description: Returns true if the address is used.
 */
bool network_check(uint8_t address);

/*
 * Function: network_get_address()
 * Description: Returns the address of the network connection.
 */
uint8_t network_get_address(void);

/*
 * Function: network_get_response()
 * Description: Sends a request over the network and receives the
 *              corresponding response.
 */
uint8_t network_get_response(uint8_t address, uint8_t command,
  const uint8_t *request_data, uint8_t request_length,
  uint8_t **response_data, uint8_t *response_length,
  int16_t timeout);

/*
 * Function: network_get_request()
 * Description: Receives a request form the network.
 */
uint8_t network_get_request(struct network_request_data *request,
  int16_t timeout);

/*
 * Function: network_send_response()
 * Description: Sends a response to a request over the network.
 */
uint8_t network_send_response(const struct network_request_data *request,
  const uint8_t *data, uint8_t length);

/*
 * Function: network_send()
 * Description: Sends a packet to the specified destination and with the
 *              specified status, command and data.
 */
uint8_t network_send(uint8_t destination, uint8_t status,
  uint8_t command, const uint8_t *data, uint8_t length);

/*
 * Function: network_wait_for_acknowledge()
 * Description: Waits for the acknowledge of the specified packet.
 */
uint8_t network_wait_for_acknowledge(
  const struct network_packet_header *packet);

/*
 * Function: network_wait_for_packet()
 * Description: Waits for a packet from the specified source and with the
 *              specified status and command. Other packets are discarded.
 */
uint8_t network_wait_for_packet(uint8_t source, uint8_t status,
  uint8_t command, uint8_t id, struct network_packet_header *packet_header,
  uint8_t **packet_data, int16_t timeout);

/*
 * Function: network_get_last_packet()
 * Description: Gets the last received packet.
 */
uint8_t network_get_last_packet(struct network_packet_header *packet_header,
  uint8_t **packet_data);

/*
 * Function: network_acknowledge_packet()
 * Description: Acknowledges receipt of a packet.
 */
uint8_t network_acknowledge_packet(
  const struct network_packet_header *packet);

/*
 * Function: network_write_packet()
 * Description: Writes a packet to the network.
 */
uint8_t network_write_packet(struct network_packet_header *packet_header,
  const uint8_t *packet_data);

/*
 * Function: network_wait_after_collision()
 * Description: Waits a pseudo random time. To use after a collision.
 */
void network_wait_after_collision(void);

/*
 * Function: network_process_byte()
 * Description: Processes a byte received from the network.
 */
uint8_t network_process_byte(uint8_t error, uint8_t data);

/*
 * Function: network_calculate_checksum()
 * Description: Returns the checksum of the packet.
 */
uint8_t network_calculate_checksum(struct network_packet_header *packet);

/*
 * Function: network_free_data()
 * Description: Releases the memory allocated by the network api.
 */
void network_free_data(uint8_t *data);

/*
 * Function: network_write_bytes()
 * Description: Writes bytes to the network.
 */
uint8_t network_write_bytes(const uint8_t *data, uint8_t length);

/*
 * Function: network_write_byte()
 * Description: Write single byte.
 */
uint8_t network_write_byte(uint8_t byte);

/*
 * Function: network_pull_up()
 * Description: Toogle pull up impulse
 */
void network_pull_up();

/*
 * Function: network_pull_down()
 * Description: Toogle pull down impulse
 */
void network_pull_down();

/*
 * Function: network_send_zero()
 * Description: Sends on Bit (0) on Bus
 */
uint8_t network_send_zero();

/*
 * Function: network_send_one()
 * Description: Sends on Bit (1) on Bus
 */
uint8_t network_send_one();

#endif /* _NETWORK_H_ */
