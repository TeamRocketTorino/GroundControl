#include <SoftwareSerial.h>

// hardwired LED: power and RX
#define READY_LED 10
#define LAUNCHED_LED 11

#define LAUNCH_BTN 2
#define AUTH_SW 3

#define LAUNCH_RL 16

#define RKT_RX 9
#define RKT_TX 8

SoftwareSerial rocketSerial(RKT_RX, RKT_TX);

bool authorized, launched;
bool ta, tl, error;

unsigned long readingTime;
float sensors[13];

void errorBlink() {
  int count = 3;
  while (count-- > 0) {
    digitalWrite(READY_LED, HIGH);
    delay(150);
    digitalWrite(READY_LED, LOW);
    delay(150);
  }
  digitalWrite(READY_LED, authorized);
}

void launch() {
  tl = true;
    
  if (authorized) {
    digitalWrite(READY_LED, LOW);
    digitalWrite(LAUNCH_RL, HIGH);
    digitalWrite(LAUNCHED_LED, HIGH);

    launched = true;
  } else {
    error = true;
  }
}

void auth() {
  ta = true;
  
  if (!launched) {
    authorized = !digitalRead(AUTH_SW);
    digitalWrite(READY_LED, authorized);
  }
}


void setup() {
  Serial.begin(115200);
  rocketSerial.begin(38400);

  pinMode(LAUNCH_BTN, INPUT_PULLUP);
  pinMode(AUTH_SW, INPUT_PULLUP);

  pinMode(READY_LED, OUTPUT);
  pinMode(LAUNCHED_LED, OUTPUT);

  //pinMode(LAUNCH_RL, INPUT_PULLUP); // makes the output start at a HIGH state, since launch is when out is LOW
  pinMode(LAUNCH_RL, OUTPUT);

  authorized = false;
  launched = false;

  ta = false;
  tl = false;
  error = false;

  digitalWrite(READY_LED, LOW);
  digitalWrite(LAUNCHED_LED, LOW);

  // we should not start authorized!
  if (authorized = !digitalRead(AUTH_SW)) {
    while(1) {
      errorBlink();
      delay(500);
    }
  }

  attachInterrupt(digitalPinToInterrupt(LAUNCH_BTN), launch, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUTH_SW), auth, CHANGE);

}

void loop() {
  if (rocketSerial.available()) {
    Serial.write(rocketSerial.read());
  }

  if (ta) {
    ta = false;
    Serial.print("auth: ");
    Serial.println(authorized);
  }

  if (tl) {
    tl = false;
    Serial.println("launch");
  }

  if (error) {
    error = false;
    errorBlink();
  }

}
