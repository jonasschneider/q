const int rxPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
const int txPin = 53;

int rxState = 0;         // variable for reading the pushbutton status

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  Serial.begin(9600);
}

int on = 0;
unsigned long n = 0;
unsigned long f = 0;
void loop() {
  n++;
  rxState = digitalRead(rxPin);
  digitalWrite(ledPin, rxState);
  if(rxState != (on ? HIGH : LOW)) {
    f++;
  }
  if(n % 1000 == 0) {
    Serial.print(f);
    Serial.print("\n");
    f = 0;
  }
  
  on = !on;
  digitalWrite(txPin, on);
  delayMicroseconds(100);
}
