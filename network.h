/******************************************************************************
 * File:            network.h
 *
 * Description:     Network protocol for communication between
 *                  microcontrollers on a bus.
 *
 * Author:          Marius Preuss, Viktor Puselja
 *
 * Date:            2014-01-21
 *
******************************************************************************/

#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdint.h>

#ifndef VISUAL_STUDIO
#include <stdbool.h>
#else
#define false   0
#define true    1
#define bool    uint8_t
#endif

// network address
#define NETWORK_ADDRESS_NONE        0x00
#define NETWORK_ADDRESS_MIN         0x01
#define NETWORK_ADDRESS_MAX         0xFE
#define NETWORK_ADDRESS_BROADCAST   0xFF

// network commands
#define NETWORK_COMMAND_NONE        0x00
#define NETWORK_COMMAND_HELLO       0x00

// network id
#define NETWORK_ID_ANY              0xFF

// network error codes
#define NETWORK_NO_ERROR                0x00
#define NETWORK_NONE_ADDRESS_AVAILABLE  0x01
#define NETWORK_TIMEOUT_EXCEEDED        0x02
#define NETWORK_INVALID_STATUS          0x03
#define NETWORK_END_OF_DATA				0x04

// represents the data of a request
struct network_request_data {
  uint8_t source;
  uint8_t destination;
  uint8_t command;
  uint8_t *data;
  uint8_t length;
};

/*
 * Function: network_initialize()
 * Description: Initializes the network connection and assigns an address.
 */
uint8_t network_initialize(void);

/*
 * Function: network_check()
 * Description: Checks if the address is used.
 */
bool network_check(uint8_t address);

/*
 * Function: network_get_response()
 * Description: Sends a request over the network and receives the
 *              corresponding response.
 */
uint8_t network_get_response(uint8_t address, uint8_t command,
    const uint8_t *request_data, uint8_t request_length, bool blocking,
    uint8_t **response_data, uint8_t *response_length, int16_t timeout);

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
    const uint8_t *data, uint8_t length, bool blocking);
  
/*
 * Function: network_free_data()
 * Description: Releases the memory allocated by the network api.
 */
void network_free_data(uint8_t *data);

#endif /* _NETWORK_H_ */
