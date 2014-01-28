/***************************************************************************************************
Filename:       protocol_header.h
Funktion:       Deklarationen f�r die Benutzung von protocol_function.c, zum kommunizieren
                von Mikrocontrollern
Ersteller:      Viktor Puselja � Marius Preuss
Datum:          21.01.2014
***************************************************************************************************/

#ifndef PROTOCOL_HEADER_H
#define PROTOCOL_HEADER_H

/***************************************************************************************************
Benoetigte Includes
***************************************************************************************************/
#include <avr/io.h>                 // Standart AVR Library
#include <stdlib.h>                 // Standart C Library
#include <stdint.h>                 // Standart Datentypen 
#include <util/delay.h>             // Wartefunktionen
#include <stdbool.h>                // Bool bekannt machen

/***************************************************************************************************
Pinbelegung
***************************************************************************************************/

#define SEND_PORT      PORTD
#define SEND_DDR       DDRD
#define SEND_DB        PD0

/***************************************************************************************************

***************************************************************************************************/

#define NETWORK_NO_ADDRESS              0x00
#define NETWORK_MIN_ADDRESS             0x01
#define NETWORK_MAX_ADDRESS             0xFE
#define NETWORK_BROADCAST               0xFF

#define NETWORK_STATUS_CHECK            0b00
#define NETWORK_STATUS_REQUEST          0b01
#define NETWORK_STATUS_RESPONSE         0b10
#define NETWORK_STATUS_ACKNOWLEDGE      0b11

#define NETWORK_COMMAND_NONE            0x00
#define NETWORK_COMMAND_HELLO           0x00

#define NETWORK_NO_ERROR                0x00
#define NETWORK_NO_ADDRESS_AVAILABLE    0x01
#define NETWORK_ADDRESS_IN_USE          0x02
#define NETWORK_COLLISION_DETECTED      0x03
#define NETWORK_GET_ERROR               0x04

#define NETWORK_TIMEOUT_INFINITE        0
#define NETWORK_TIMEOUT_CHECK           500

/***************************************************************************************************
Structs
***************************************************************************************************/

typedef struct
{
  byte Source;
  byte Destination;
  byte ID;
  byte Command;
  byte *Data;
} RequestData, *PRequestData;

struct
{
  byte Address;
  byte LastId;
} Connection;

typedef struct
{
  byte Source;
  byte Destination;
  byte Status;
  byte ID;
  byte Checksum;
  byte Command;
  byte *Data;
} Packet, *PPacket;

/***************************************************************************************************
Funktionsprototyp
***************************************************************************************************/

// function: connect()
byte connect();

// function: check(byte address)
bool check(byte address);

// function: hello()
byte hello();

// function: doRequest(byte destination, byte command, byte *data)
PRequestData doRequest(byte destination, byte command, byte *data);

// function: getRequest(PRequestData *request)
byte getRequest(PRequestData *request);

// function: doResponse(PRequestData, byte *response)
byte doResponse(PRequestData request, byte *response);

// function: getResponse(PRequestData request, byte *response)
byte getResponse(PRequestData request, byte *response, bool blocking);

// function: writePacket(byte destination, byte status, byte command, byte *data)
byte writePacket(byte destination, byte status, byte command, byte *data);

// function: receivePacket(byte *source, byte *status, byte *command, byte **data)
byte receivePacket(byte *source, byte *status, byte *command, byte **data);

// function: receivePacketBlocking(byte *source, byte *status, byte *command, byte **data, short timeout)
byte receivePacketBlocking(byte *source, byte *status, byte *command, byte **data, short timeout);

// function: write(byte *data, int count)
byte write(byte *data, int count);

// function: read(byte **data)
byte read(byte **data);

// function: writeByte(byte data)
byte writeByte(byte data);

// function: readByte(byte *data)
byte readByte(byte *data);

// function: writeBit(byte bit)
byte writeBit(byte bit);

// function: readBit()
byte readBit();

// function: signalCollision()
byte signalCollision();

// function: waitAfterCollision()
void waitAfterCollision();

// function: calculateChecksum(PPacket packet)
byte calculateChecksum(PPacket packet);

// function: closeRequest(PRequestData data)
void closeRequest(PRequestData data);

// function: setLastError(byte errorcode)
byte setLastError(byte errorcode);

// function: getLastError()
byte getLastError();

#endif /* PROTOCOL_HEADER_H */
