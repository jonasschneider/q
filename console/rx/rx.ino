#include "VirtualWire.h"

void setup()
{
    Serial.begin(9600);   // Debugging only
    Serial.println("setup");
    delay(1);

    pinMode(13, OUTPUT);
    vw_set_rx_pin(3);

    // with STM32 TX:
    vw_setup(8*8);

    // for Arduino TX
    // The TX's timer0 fires (16 000 000 / 64) / 256 =~ 976 times per second.
    // Since we need 8 timer firings to transfer a single bit, divide by 8.
    // TODO: fix the 2 factor by removing nticks*2 from the implementation.
//    vw_setup(122*2);

    vw_rx_start();
}

void loop()
{
    vw_wait_rx();

    uint8_t buf[100];
    uint8_t buflen = 100;
    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
        int i;
        Serial.print("\nGot data:");
        Serial.print(buflen, DEC);
        Serial.print(":\n");

        for (i = 0; i < buflen; i++)
        {
                char c = (buf[i]);
                Serial.print(c);
        }
        Serial.println("\n");
    }
}
