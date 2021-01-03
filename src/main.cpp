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

#define COUNTER_MEMORY_ADDR() (EEPROM.length() - 10 * sizeof(unsigned int))

byte cus[] = {
    B00000,
    B11111,
    B10001,
    B10001,
    B10001,
    B11111,
    B00000,
    B00000};

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

enum MenuPosition { menuDashboard,
                    menuTemp,
                    menuMinutes,
                    menuDuration,
                    menuMaxIterations,
                    menuManualRotate,
                    menuSave,
                    menuSaved };

MenuPosition menuPosition = menuDashboard;

struct DHT_OUTPUT {
  float t;
  float h;
};

DHT dht(DHTPIN, DHTTYPE);
DHT_OUTPUT dhtOutput = {0.0, 0.0};

unsigned long rotateDuration = 2000;
unsigned long rotateEveryMinutes = 10;
unsigned long lastRotate = 0;
unsigned int maxIterations = 10;
unsigned int iterateCount = 0;
bool rotateState = false;

double Setpoint, Input, Output;
double Kp = 1, Ki = 20, Kd = 30;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

void handlePID();
void readDHT();
void handleMenu(long rotaryPosition);
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
  digitalWrite(BULB_PIN, LOW);

  pinMode(BACKLIGHT_PIN, OUTPUT);
  turnOnBacklight();

  readEEPROM();

  lastRotate = millis();
  lastBacklight = millis();

  oldPosition = (long)rotaryEncoder.read();

  dht.begin();

  windowStartTime = millis();
  Input = (double)dhtOutput.t;
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

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
  display.println("ver. 0.1.0");
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
  handlePID();
  handleMenu(oldPosition);
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

  // read motor interval
  unsigned long m;
  EEPROM.get(idx, m);
  if (m > 0 && m < 60 * 25) {
    rotateEveryMinutes = m;
  } else {
    rotateEveryMinutes = 1;
    EEPROM.put(idx, rotateEveryMinutes);
  }
  idx += sizeof(unsigned long);

  // read motor-on duration
  unsigned int d;
  EEPROM.get(idx, d);
  if (m > 0 && m < 100000) {
    rotateDuration = d;
  } else {
    rotateDuration = 2000;
    EEPROM.put(idx, rotateDuration);
  }
  idx += sizeof(unsigned long);

  // read max motor iterations
  unsigned int maxIter;
  EEPROM.get(idx, maxIter);
  if (maxIter >= 0 && maxIter < 1000) {
    maxIterations = maxIter;
  } else {
    maxIterations = 10;
    EEPROM.put(idx, maxIterations);
  }
  idx += sizeof(unsigned int);

  // get current timers
  // get iteration count at the end of eeprom
  unsigned int iterCount;
  EEPROM.get(COUNTER_MEMORY_ADDR(), iterCount);
  if (iterCount >= 0 && iterCount < 1000) {
    iterateCount = iterCount;
  } else {
    iterateCount = iterCount;
    EEPROM.put(COUNTER_MEMORY_ADDR(), iterateCount);
  }
}

void saveEEPROM() {
  int idx = 0;
  // set temperature setpoint
  EEPROM.put(idx, Setpoint);
  idx += sizeof(double);
  // set motor interval
  EEPROM.put(idx, rotateEveryMinutes);
  idx += sizeof(unsigned long);
  // set motor-on duration
  EEPROM.put(idx, rotateDuration);
  idx += sizeof(unsigned long);
  // set max iterations
  EEPROM.put(idx, maxIterations);
  idx += sizeof(unsigned int);

  // at the end of eeprom
  // set iteration count
  EEPROM.put(COUNTER_MEMORY_ADDR(), iterateCount);
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

void handlePID() {
  Input = (double)dhtOutput.t;
  myPID.Compute();

  if (millis() - windowStartTime > (unsigned long)WindowSize) {
    windowStartTime += WindowSize;
  }

  if (Output < millis() - windowStartTime) {
    digitalWrite(BULB_PIN, LOW);
  } else {
    digitalWrite(BULB_PIN, HIGH);
  }
}

void handleRotate() {
  if (iterateCount >= maxIterations)
    return;

  if (millis() - lastRotate > rotateEveryMinutes * 60 * 1000) {
    rotateState = true;
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
  }

  if (rotateState && millis() - lastRotate > (rotateEveryMinutes * 60 * 1000) + rotateDuration) {
    digitalWrite(MOTOR_PIN, HIGH);
    lastRotate = millis();
    rotateState = false;
    iterateCount += 1;
    // saveEEPROM();
    // writes optimization - writes counter only
    EEPROM.put(COUNTER_MEMORY_ADDR(), iterateCount);
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

void handleMenu(long rotaryPosition) {
  // handle menu key
  int button = digitalRead(ROTARY_BUTTON);

  if (button == LOW && millis() - lastDebounceTime > 500) {
    lastDebounceTime = millis();
    diffPosition = 0;
    turnOnBacklight();

    switch (menuPosition) {
    case menuDashboard:
      menuPosition = menuTemp;
      break;
    case menuTemp:
      menuPosition = menuMinutes;
      break;
    case menuMinutes:
      menuPosition = menuDuration;
      break;
    case menuDuration:
      menuPosition = menuMaxIterations;
      break;
    case menuMaxIterations:
      menuPosition = menuManualRotate;
      break;
    case menuManualRotate:
      menuPosition = menuSave;
      break;
    case menuSave:
      menuPosition = menuDashboard;
      break;
    case menuSaved:
      menuPosition = menuDashboard;
      break;
    }
  }

  display.clearDisplay();

  switch (menuPosition) {
  case menuDashboard: {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(dhtOutput.h);
    display.println("%");
    display.print(dhtOutput.t);
    display.println("C");
    display.setTextSize(1);
    display.print(Setpoint);
    display.print("C ");
    display.print(iterateCount);
    display.print("/");
    display.println(maxIterations);

    if (iterateCount >= maxIterations) {
      display.print("KONIEC");
    } else {
      unsigned long nextRotate = (rotateEveryMinutes * 60 * 1000 - (millis() - lastRotate)) / 1000;

      if (nextRotate < 1000000) {
        unsigned long hh = nextRotate / (60 * 60);
        unsigned long mm = (nextRotate - (hh * 60 * 60)) / 60;
        unsigned long ss = nextRotate - (hh * 60 * 60 + mm * 60);
        display.print(hh);
        display.print("h");
        display.print(mm);
        display.print("m");
        display.print(ss);
        display.print("s");
      } else {
        display.print("Obrot");
      }
    }

    break;
  }
  case menuTemp: {
    if (diffPosition != 0) {
      double r = roundf((double)diffPosition / 4) / 10.0;
      double d = roundf(r * 10.0) / 10.0;
      Setpoint += d;
      diffPosition = 0;
    }

    display.setTextSize(1);
    display.println(" Temperatura");
    display.println("  ustawiona");
    display.println("");
    display.setTextSize(2);
    display.print(Setpoint);
    display.println("C");

    break;
  }
  case menuMinutes: {
    if (diffPosition != 0) {
      int d = round((double)diffPosition / 4);
      rotateEveryMinutes += d;

      if (rotateEveryMinutes < 1 || rotateEveryMinutes > 9999) {
        rotateEveryMinutes = 1;
      }

      lastRotate = millis();
      diffPosition = 0;
    }

    unsigned int h = rotateEveryMinutes / 60;
    unsigned int m = rotateEveryMinutes - (h * 60);
    display.setTextSize(1);
    display.println("  Obrot co");
    display.setTextSize(2);
    display.println("");
    display.print(h);
    display.print("h");
    display.print(m);
    display.print("m");

    break;
  }
  case menuDuration: {
    if (diffPosition != 0) {
      int d = round((double)diffPosition / 4);
      rotateDuration += d * 100;
      if (rotateDuration < 100) {
        rotateDuration = 100;
      }

      lastRotate = millis();
      diffPosition = 0;
    }

    display.setTextSize(1);
    display.println(" Obrot przez");
    display.setTextSize(2);
    display.println("");
    display.print(roundf(((float)rotateDuration / 1000.0) * 10.0) / 10.0);
    display.print("s");
    break;
  }
  case menuMaxIterations: {
    if (diffPosition != 0) {
      int d = round((double)diffPosition / 4);
      maxIterations += d;
      if (maxIterations < 1) {
        maxIterations = 1;
      }

      iterateCount = 0;
      diffPosition = 0;
    }
    display.setTextSize(1);
    display.println("Max obrotow");
    display.setTextSize(2);
    display.print(" ");
    display.println(maxIterations);
    display.setTextSize(1);
    display.println("Zrobionych");
    display.setTextSize(2);
    display.print(" ");
    display.print(iterateCount);

    break;
  }
  case menuManualRotate: {
    if (diffPosition > 0) {
      digitalWrite(MOTOR_PIN, LOW);
    } else {
      digitalWrite(MOTOR_PIN, HIGH);
    }

    display.setTextSize(1);
    display.println("Manualny Obrot");
    display.setTextSize(2);
    display.println("");
    display.print("   ");
    display.println(diffPosition > 0 ? "X" : "-");
    break;
  }
  case menuSave: {
    display.setTextSize(1);
    display.println("");
    display.setTextSize(2);
    display.println("Zapisz");
    display.setTextSize(1);
    display.println("obroc w prawo");

    if (diffPosition > 5) {
      saveEEPROM();
      menuPosition = menuSaved;
    }
    break;
  }
  case menuSaved: {
    display.setTextSize(1);
    display.println("");
    display.println(" ------------");
    display.println("| Ustawienia |");
    display.println("|  zapisane  |");
    display.println(" ------------");
    break;
  }
  }

  display.display();
}
