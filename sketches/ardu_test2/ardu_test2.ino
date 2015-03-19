const int rxPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
const int txPin = 53;

int rxState = 0;         // variable for reading the pushbutton status

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  Serial.begin(9600);
  Serial.write("hi!\n");
    Serial1.begin(9600);
        Serial2.begin(9600);
}

int on = 0;
void loop() {
  Serial1.write('.');
  Serial.write(Serial2.read());
}
