#include "network.h"

#include <avr/interrupt.h>

int main(void)
{
    sei();

    network_initialize_external_int();
    network_initialize_timer_int();
    network_set_port_mode(true);

    while(1)
    {
        network_write_byte(0b10101010);
    }
    return 0;
}
