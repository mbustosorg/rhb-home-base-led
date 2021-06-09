#include <Arduino.h>

#define DEVICE_COUNT (3)
#define DEFAULT_PULSE_FREQUENCY (0.1) //(0.30)
#define DEFAULT_PULSE_DURATION (10000.0) //(1500.0) // Must be <= 1.0 / DEFAULT_PULSE_FREQUENCY * 1000.0
#define IN_RANGE_DISTANCE (1.0)

#define TEST_LENGTH_SAMPLES (1)
#define FRAME_RATE (60)

int pwmPins[DEVICE_COUNT] = {20, 21, 22};
int pwmValues[DEVICE_COUNT] = {50, 50, 50};
float pulseDuration[DEVICE_COUNT] = {DEFAULT_PULSE_DURATION, DEFAULT_PULSE_DURATION, DEFAULT_PULSE_DURATION};
float defaultPulseDuration[DEVICE_COUNT] = {DEFAULT_PULSE_DURATION, DEFAULT_PULSE_DURATION, DEFAULT_PULSE_DURATION};
float pulseFrequency[DEVICE_COUNT] = {DEFAULT_PULSE_FREQUENCY,
                                      DEFAULT_PULSE_FREQUENCY + 0.01,
                                      DEFAULT_PULSE_FREQUENCY + 0.02};
float defaultPulseFrequency[DEVICE_COUNT] = {DEFAULT_PULSE_FREQUENCY,
                                             DEFAULT_PULSE_FREQUENCY + 0.01,
                                             DEFAULT_PULSE_FREQUENCY + 0.02};
long pulseStart[DEVICE_COUNT] = {0, 0, 0};

bool light = false;
long initialDistanceTimestamp = 0;
bool inRange = false;
bool led = true;


void continuePatternDisplay() {
  for (int i = 0; i < DEVICE_COUNT; i++) {
    float proportion = 1.0;
    if (millis() - pulseStart[i] > pulseDuration[i]) {
      proportion = 0.0;
      if (millis() > pulseStart[i] + 1000 / pulseFrequency[i]) {
        pulseStart[i] = millis();
        proportion = 1.0;
      }
    } else {
      proportion = min((float)(pulseDuration[i] - (millis() - pulseStart[i])) / (float)pulseDuration[i], 1.0);
    }
    analogWrite(pwmPins[i], int(pwmValues[i] * proportion));
  }
}

void setPatternData(float distance) {
  bool checkTransition = distance < IN_RANGE_DISTANCE;
  if (checkTransition != inRange) {
    inRange = checkTransition;
    if (!inRange) {
      Serial.println("OUTOFRANGE");
      for (int i = 0; i < DEVICE_COUNT; i++) {
        pulseFrequency[i] = defaultPulseFrequency[i];
        pulseDuration[i] = defaultPulseDuration[i];
      }
    } else {
      Serial.println("INRANGE");
      for (int i = 0; i < DEVICE_COUNT; i++) {
        pulseStart[i] = millis();
      }
    }
  }
  if (inRange) {
    float newFrequency = min(2.0, IN_RANGE_DISTANCE / distance * DEFAULT_PULSE_FREQUENCY);
    float newDuration = 1.0 / newFrequency * 500.0;
    for (int i = 0; i < DEVICE_COUNT; i++) {
      pulseFrequency[i] = newFrequency;
      pulseDuration[i] = newDuration;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0) {
    String msg = "";
    bool numeric = true;
    while (Serial.available() > 0) {
      char read = Serial.read();
      numeric = numeric && isDigit(read);
      msg += read;
    }
    if (msg.length() > 0) {
      if (numeric) {
        setPatternData(msg.toFloat() / 1000.0);
        Serial.print("ACK ");
        Serial.println(msg);
      } else {
        Serial.println("NACK");
      }
    }
    if (led) {
      led = false;
      digitalWrite(13, LOW);
    } else {
      led = true;
      digitalWrite(13, HIGH);
    }
  }
  continuePatternDisplay();
}
