/******************************************************************************
 * File:            network_hardware.h
 *
 * Description:     Interface between the hardware dependent and hardware
 *                  independent implementation parts of the network protocol.
 *
 * Author:          Marius Preuss, Viktor Puselja
 *
 * Creation date:   2014-08-31
 *
******************************************************************************/

#ifndef _NET_HARDWARE_H_
#define _NET_HARDWARE_H_

#include "network.h"

/*
 * Function: network_initialize_hardware()
 * Description: Initializes the hardware dependent part.
 */
void network_initialize_hardware(void);

/*
 * Function: network_delay_ms()
 * Description: Waits one millisecond.
 */
void network_delay_ms(void);

/*
 * Function: network_start_writing()
 * Description: Initializes the writing process.
 */
void network_start_writing(void);

/*
 * Function: network_wait_writing_complete()
 * Description: Waits for the current writing process to complete.
 */
void network_wait_writing_complete(void);

/*
 * Function: network_process_byte()
 * Description: Processes a read byte.
 */
void network_process_byte(uint8_t error, uint8_t data);

/*
 * Function: network_wait_writing_complete()
 * Description: Gets a byte to write.
 */
bool network_get_next_byte(uint8_t *byte);

#endif /* _NET_HARDWARE_H_ */
