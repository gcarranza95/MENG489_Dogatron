// Based on buzzer example (Rob Faludi) for CEM-1203 buzzer

const int SPEAKER_PIN = 4;

void setup() {
  pinMode(SPEAKER_PIN, OUTPUT);
}

void loop() {
  buzz(SPEAKER_PIN, 1760, 500); // buzz at 1760 Hz for 1000 milliseconds
  delay(1000); // wait a bit between buzzes
}

void buzz(int pin, long frequency, long len) {
  long delayVal = 1000000/frequency/2; // Delay value between transitions; unitless (1 second in us)/(1/s)
  long numCycles = frequency*len/1000; // Number of cycles; unitless (1/s)/(s)
  for (long i=0; i < numCycles; i++) {
    digitalWrite(pin, HIGH);      // high (push out the diaphram)
    delayMicroseconds(delayVal);  // wait  
    digitalWrite(pin, LOW);       // low (pull back the diaphram)
    delayMicroseconds(delayVal);  // wait  
  }
}
