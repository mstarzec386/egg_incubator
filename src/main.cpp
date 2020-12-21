#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#include "DHT.h"

#define DHTTYPE DHT11
#define DHTPIN 9
#define BACKLIGHT_PIN 10
#define BULB_PIN A0
#define MOTOR_PIN A1
#define ROTARY_A 2
#define ROTARY_B 3
#define ROTARY_BUTTON 11
#define MOTOR 12
#define HEATER 13

// Declare LCD object for software SPI
// Adafruit_PCD8544(CLK,DIN,D/C,CE,RST);
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 8);
unsigned long lastBacklight;
bool backlightOn;

long oldPosition = 0;
long diffPosition = 0;
Encoder rotaryEncoder(ROTARY_B, ROTARY_A);
unsigned long lastDebounceTime = 0;
int lastButtonState = HIGH;

enum MenuPosition { menuTemp,
                    menuMinutes,
                    menuDuration,
                    menuDebugPid,
                    menuDebugRotate };

MenuPosition menuPosition = menuTemp;

struct DHT_OUTPUT {
  float t;
  float h;
};

DHT dht(DHTPIN, DHTTYPE);
DHT_OUTPUT dhtOutput = {0.0, 0.0};

unsigned long rotateDuration = 2000;
unsigned int rotateEveryMinutes;
unsigned long lastRotate;
bool rotateState = false;

double Setpoint, Input, Output;
double Kp = 1, Ki = 20, Kd = 30;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 10000;
unsigned long windowStartTime;

int MIN_STATE_CHANGE = 1000;
bool oldChangeState = false;
unsigned long lastChangeStart = 0;

double handlePID();
void readDHT();
void handleMenu(double pidOut, long rotaryPosition);
void handleRotate();
void readEEPROM();
void saveEEPROM();
void turnOffBacklight();
void turnOnBacklight();

void setup() {
  Serial.begin(9600);
  pinMode(ROTARY_BUTTON, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BULB_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, HIGH);
  digitalWrite(BULB_PIN, HIGH);

  pinMode(BACKLIGHT_PIN, OUTPUT);
  turnOnBacklight();

  readEEPROM();

  lastRotate = millis();
  lastBacklight = millis();

  long oldPosition = (long)rotaryEncoder.read();

  dht.begin();

  windowStartTime = millis();
  Input = (double)dhtOutput.t;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  //Initialize Display
  display.begin();

  // you can change the contrast around to adapt the display for the best viewing!
  display.setContrast(57);

  // Clear the buffer.
  display.clearDisplay();

  // Display Text
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.println("Egg Incubator");
  display.println("ver. 0.0.1");
  display.display();
  delay(1000);
  display.clearDisplay();
}

void loop() {
  long newPosition = (long)rotaryEncoder.read();
  if (newPosition != oldPosition) {
    turnOnBacklight();
    diffPosition += newPosition - oldPosition;
    oldPosition = newPosition;
  }

  readDHT();
  double out = handlePID();
  handleMenu(out, oldPosition);
  handleRotate();
  turnOffBacklight();
}

void readEEPROM() {
  int idx = 0;
  // read temp
  double t;
  EEPROM.get(idx, t);

  if (t > -20 && t < 120) {
    Setpoint = t;
  } else {
    Setpoint = 33.0;
    EEPROM.put(0, Setpoint);
  }
  idx += sizeof(double);

  //read rotate every minutes
  unsigned int m;
  EEPROM.get(idx, m);
  if (m > 0 && m < 60 * 25) {
    rotateEveryMinutes = m;
  } else {
    rotateEveryMinutes = 1;
    EEPROM.put(idx, rotateEveryMinutes);
  }
  idx += sizeof(unsigned int);

  unsigned int d;
  EEPROM.get(idx, d);
  if (m > 0 && m < 100000) {
    rotateDuration = d;
  } else {
    rotateDuration = 2000;
    EEPROM.put(idx, rotateDuration);
  }
  idx += sizeof(unsigned int);
}

void saveEEPROM() {
  int idx = 0;
  EEPROM.put(idx, Setpoint);
  idx += sizeof(double);
  EEPROM.put(idx, rotateEveryMinutes);
  idx += sizeof(unsigned int);
  EEPROM.put(idx, rotateDuration);
  idx += sizeof(unsigned int);
}

void turnOffBacklight() {
  if (backlightOn && millis() - lastBacklight > 30000) {
    analogWrite(BACKLIGHT_PIN, 0);
    backlightOn = false;
  }
}

void turnOnBacklight() {
  lastBacklight = millis();
  analogWrite(BACKLIGHT_PIN, 100);
  backlightOn = true;
}

double handlePID() {
  Input = (double)dhtOutput.t;
  myPID.Compute();

  if (millis() - windowStartTime > WindowSize) {
    windowStartTime += WindowSize;
  }

  double out = Output * 100;

  if (out < 500) {
    out = 0;
  }

  if (out < millis() - windowStartTime) {
    if (oldChangeState && millis() - lastChangeStart > MIN_STATE_CHANGE) {
      oldChangeState = false;
      lastChangeStart = millis();
      digitalWrite(BULB_PIN, HIGH);
    }
  } else {
    if (!oldChangeState && millis() - lastChangeStart > MIN_STATE_CHANGE) {
      oldChangeState = true;
      lastChangeStart = millis();
      digitalWrite(BULB_PIN, LOW);
    }
  }

  return out;
}

void handleRotate() {
  if (millis() - lastRotate > rotateEveryMinutes * 60 * 1000) {
    rotateState = true;
    digitalWrite(MOTOR_PIN, LOW);
  }

  if (rotateState && millis() - lastRotate > (unsigned long)(rotateEveryMinutes * 60 * 1000) + rotateDuration) {
    digitalWrite(MOTOR_PIN, HIGH);
    lastRotate = millis();
    rotateState = false;
  }
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (h > 0 && h < 101) {
    dhtOutput.h = h;
  }
  if (t > -30 && t < 200) {
    dhtOutput.t = t;
  }
}

void handleMenu(double pidOut, long rotaryPosition) {
  //handle menu key
  int button = digitalRead(ROTARY_BUTTON);

  if (button == LOW && millis() - lastDebounceTime > 500) {
    lastDebounceTime = millis();
    diffPosition = 0;
    turnOnBacklight();

    switch (menuPosition) {
    case menuTemp:
      menuPosition = menuMinutes;
      break;
    case menuMinutes:
      menuPosition = menuDuration;
      break;
    case menuDuration:
      menuPosition = menuDebugPid;
      break;
    case menuDebugPid:
      menuPosition = menuDebugRotate;
      break;
    case menuDebugRotate:
      menuPosition = menuTemp;
      break;
    }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(dhtOutput.h);
  display.println("%");
  display.print(dhtOutput.t);
  display.println("C");

  switch (menuPosition) {
  case menuTemp:
    if (diffPosition != 0) {
      double r = roundf((double)diffPosition / 4) / 10.0;
      double d = roundf(r * 10.0) / 10.0;
      Setpoint += d;
      diffPosition = 0;
      saveEEPROM();
    }
    display.setTextSize(2);
    display.print("T:");
    display.println(Setpoint);
    break;
  case menuMinutes:
    if (diffPosition != 0) {
      unsigned int d = round((double)diffPosition / 4);
      rotateEveryMinutes += d;
      if (rotateEveryMinutes < 1) {
        rotateEveryMinutes = 1;
      }

      lastRotate = millis();
      diffPosition = 0;
      saveEEPROM();
    }
    display.setTextSize(2);
    display.print("M:");
    display.print(rotateEveryMinutes);
    break;
  case menuDuration:
    if (diffPosition != 0) {
      unsigned long d = round((double)diffPosition / 4);
      rotateDuration += d * 100;
      if (rotateDuration < 100) {
        rotateDuration = 100;
      }

      lastRotate = millis();
      diffPosition = 0;
      saveEEPROM();
    }
    display.setTextSize(2);
    display.print("D:");
    display.print(roundf(((float)rotateDuration / 1000.0) * 10.0) / 10.0);
    break;
  case menuDebugPid:
    display.setTextSize(1);
    display.print(millis() - windowStartTime);
    display.print("; ");
    display.println(round(pidOut));
    display.print(rotaryPosition);
    display.print(";");
    display.print(diffPosition);
    display.print(";");
    display.print(digitalRead(ROTARY_BUTTON));
    /* code */
    break;
  case menuDebugRotate:
    display.setTextSize(1);
    display.print(millis() - lastRotate);
    display.print(";");
    display.println(rotateDuration);
    display.print(rotateEveryMinutes * 60 * 1000);
    display.print(";");
    display.print((unsigned long)(rotateEveryMinutes * 60 * 1000) + rotateDuration);
    display.print(";");
    display.print(rotateState);
    /* code */
    break;
  }

  display.display();
}