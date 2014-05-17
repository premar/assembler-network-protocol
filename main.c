#include "network.h"

#include <avr/interrupt.h>

int main(void)
{
    sei();
    network_pull_up();
    while(1)
    {
    }
    return 0;
}
