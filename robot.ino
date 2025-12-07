#include <Servo.h>

Servo myServo;

const int servoPin = 3;
const int buttonPin = 2;

bool servoState = false;   // false = 0°, true = 90°
bool lastButton = HIGH;

void setup() {
  myServo.attach(servoPin);
  pinMode(buttonPin, INPUT_PULLUP);
  myServo.write(0);
}

void loop() {
  bool buttonNow = digitalRead(buttonPin);

  // deteksi saat tombol baru ditekan (edge)
  if (lastButton == HIGH && buttonNow == LOW) {
    servoState = !servoState;   // toggle state

    if (servoState) {
      myServo.write(90);
    } else {
      myServo.write(0);
    }

    delay(200); // debounce sederhana
  }

  lastButton = buttonNow;
}
