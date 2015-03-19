// PWM-modulierender und deswegen super-fehlertoleranter Software-UART. Oder so. V1.
// Verkabelung: Draht zwischen Pins 3 & 8.

const int aTxPin = 8;

//assume that pin 32 is receiving PWM input
#define CHANNEL_1_PIN 3
//micros when the pin goes HIGH
volatile unsigned long timer_start;
//difference between timer_start and micros() is the length of time that the pin 
//was HIGH - the PWM pulse length. 
volatile unsigned long pulse_time; 

//this is the time that the last interrupt occurred. 
//you can use this to determine if your receiver has a signal or not.
volatile unsigned long last_interrupt_time;

//calcSignal is the interrupt handler
void calcSignal()
{
//record the interrupt time so that we can tell if the receiver has a signal from the transmitter
 last_interrupt_time = micros();

//if the pin has gone HIGH, record the microseconds since the Arduino started up
 if(digitalRead(CHANNEL_1_PIN) == HIGH)
    {
        timer_start = micros();
    }
//otherwise, the pin has gone LOW
    else
    {
        //only worry about this if the timer has actually started
        if(timer_start > 0)
        {
            //record the pulse time
            pulse_time = micros() - timer_start;
            //restart the timer
            timer_start = 0;
        }
    }
}



void setup() {
  pinMode(aTxPin, OUTPUT);

  attachInterrupt(1, calcSignal, CHANGE);

  Serial.begin(9600);
  //analogWrite(aTxPin, 120);
  delay(500);
}


unsigned long n = 0;
int pos = -10;
unsigned char to_send = '#';
void txloop() {
  if(pos > 7) {
    pos = -5;
  }
  if(pos < -1) {
    pos++;
    return;
  }
  if(pos == -1) {
    analogWrite(aTxPin, 240);
    pos = 0;
    Serial.print(".");
    return;
  }
  char bit_to_send = (to_send >> (pos++)) & 1;
  analogWrite(aTxPin, bit_to_send ? 160 : 80);
  //analogWrite(aTxPin,255);
}

// tx=10 pw=80, tx=255 pw=53444
int rxpos = -1;
unsigned char rcv = 0;
void rxloop() {
  unsigned char eightbitval = pulse_time / 8;
//  Serial.print(eightbitval, DEC);
//  Serial.print(" -- ");
//  Serial.print(pulse_time, DEC);
//  Serial.print("\n");
  if(rxpos == -1) {
    if(eightbitval > 200) { // sync!
      rxpos = 0;
    }
    return;
  }
  char thisbit = eightbitval > 120;
  rcv |= thisbit << rxpos++;
  
  if(rxpos == 7) {
//    Serial.print("GOT:");
//    Serial.print(rcv, DEC);
//    Serial.print(" aka: ");
    Serial.write(rcv);
//    Serial.print("\n");
    rxpos = -1;
    rcv = 0;
  }
}

void loop() {
  n++;
  txloop();
  delayMicroseconds(10);
  rxloop();
//  delay(1);
delay(2);
//delayMicroseconds(500);
//  Serial.write(".");
}

