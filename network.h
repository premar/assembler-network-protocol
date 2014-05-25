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

// Network Attempts
#define NETWORK_WRITE_PACKET_ATTEMPTS   5
#define NETWORK_SEND_ATTEMPTS           3

// Network Timer Interrupt Modes
#define NETWORK_TIMER_INTERRUPT_MODE_NONE   0
#define NETWORK_TIMER_INTERRUPT_MODE_READ   1
#define NETWORK_TIMER_INTERRUPT_MODE_WRITE  2

// network external interrupt constants
#define NETWORK_INT_VECT        INT0_vect
#define NETWORK_INT             INT0
#define NETWORK_INTF            INTF0
#define NETWORK_ISC1            ISC01
#define NETWORK_ISC0            ISC00

// network timer interrupt constants
#define NETWORK_TIMER_OVF_VECT  TIMER0_OVF_vect
#define NETWORK_TIMER_COMP_VECT TIMER0_COMP_vect
#define NETWORK_TOIE            TOIE0
#define NETWORK_OCIE            OCIE0
#define NETWORK_TOV             TOV0
#define NETWORK_OCF             OCF0
#define NETWORK_TCCR            TCCR0
#define NETWORK_CS0             CS00
#define NETWORK_CS2             CS02
#define NETWORK_TCNT            TCNT0
#define NETWORK_OCR             OCR0

#define NETWORK_PORT            PORTD
#define NETWORK_DDR             DDRD
#define NETWORK_PIN             PIND
#define NETWORK_PORT_PIN        PD2

#define NETWORK_VALIDATE_PORT   PORTA
#define NETWORK_VALIDATE_DDR    DDRA
#define NETWORK_VALIDATE_DB     PD1

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

void network_initialize_external_int(void);
void network_disable_external_int(void);
void network_enable_external_int(void);
void network_initialize_timer_int(void);
void network_disable_timer_ovf_int(void);
void network_disable_timer_cpm_int(void);
void network_enable_timer_ovf_int(void);
void network_enable_timer_cpm_int(void);

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

void network_set_port_mode(bool read);
uint8_t network_write_bit(uint8_t bit);


#endif /* _NETWORK_H_ */
