#include "network.h"

#include <avr/interrupt.h>

int main(void)
{
    sei();

    network_initialize();
    network_write_byte(56);

    while(1)
    {
    }
    return 0;
}
