// Demonstrate how we can use the AVR's timer0 used by arduino for keeping time
// to do some work in specific time intervals as well.

// this interrupt will trigger whenever timer0 reaches the value stored in OCR0A.
// That value is zero by default, so it will just trigger whenever the timer
// rolls over, i.e. overflows. So this is just a "secondary" ISR that is called
// at about the same time as ISR(TIM0_OVF_vect)... that's the plan at least.
// In practice, of course we don't care about the phase of this signal,
// only about the frequency.
int n = 0;
volatile bool stateA = false;
ISR(TIMER0_COMPA_vect)
{
    digitalWrite(13, stateA);
    if(n++ % 100 == 0)
      stateA = !stateA;
}

void setup()
{
    // enable timer compare interrupt in channel A of timer 0
    TIMSK0 |= (1 << OCIE0A);

    pinMode(13, OUTPUT);
}

void loop() {
}
