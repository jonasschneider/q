
#include "VirtualWire.h"

void setup()
{
    Serial.begin(9600);	  // Debugging only
    Serial.println("setup");
    delay(1);

    // Initialise the IO and ISR
//    vw_set_tx_pin(8);
    pinMode(13, OUTPUT);
    vw_set_rx_pin(3);
//    vw_set_ptt_inverted(true); // Required for DR3100
//    vw_setup(8*8);	 // Bits per sec
// for arduino
    vw_setup(122*2);
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
	// Message with a good checksum received, dump it.
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

