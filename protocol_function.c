/***************************************************************************************************
Filename:       protocol_function.c
Funktion:       Routinen zum kommunizieren von Mikrocontrollern
Ersteller:      Viktor Puselja ¦ Marius Preuss
Datum:          21.01.2014
***************************************************************************************************/

// function: connect()
byte connect(){

}

// function: check(byte address)
bool check(byte address){

}

// function: hello()
byte hello(){

}

// function: doRequest(byte destination, byte command, byte *data)
PRequestData doRequest(byte destination, byte command, byte *data){

}

// function: getRequest(PRequestData *request)
byte getRequest(PRequestData *request){

}

// function: doResponse(PRequestData, byte *response)
byte doResponse(PRequestData request, byte *response){

}

// function: getResponse(PRequestData request, byte *response)
byte getResponse(PRequestData request, byte *response, bool blocking){

}

// function: writePacket(byte destination, byte status, byte command, byte *data)
byte writePacket(byte destination, byte status, byte command, byte *data){

}

// function: receivePacket(byte *source, byte *status, byte *command, byte **data)
byte receivePacket(byte *source, byte *status, byte *command, byte **){

}

// function: receivePacketBlocking(byte *source, byte *status, byte *command, byte **data, short timeout)
byte receivePacketBlocking(byte *source, byte *status, byte *command, byte **data, short timeout){

}

// function: write(byte *data, int count)
byte write(byte *data, int count){

}

// function: read(byte **data)
byte read(byte **data){

}

// function: writeByte(byte data)
byte writeByte(byte data);

// function: readByte(byte *data)
byte readByte(byte *data){

}

// function: writeBit(byte bit)
byte writeBit(byte bit){

}

// function: readBit()
byte readBit(){

}

// function: signalCollision()
byte signalCollision(){

}

// function: waitAfterCollision()
void waitAfterCollision(){

}

// function: calculateChecksum(PPacket packet)
byte calculateChecksum(PPacket packet){

}

// function: closeRequest(PRequestData data)
void closeRequest(PRequestData data){

}

// function: setLastError(byte errorcode)
byte setLastError(byte errorcode){

}

// function: getLastError()
byte getLastError(){

}