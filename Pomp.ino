#include <EEPROM.h>
#include "Timer.h"

enum ToggleState {
  TOGGLE_OFF = 0,
  TOGGLE_ON = 1,
  TOGGLE_CAL = 2
};

struct ButtonState {
  int previousState;
  int currentState;
  unsigned long lastDebounceTime;
  unsigned long pressStartTime;
  int toggleState;
  bool handled;
  bool justToggledOn;
};

bool isPumpOn = false;


Timer timer;
const int LED_FILL = 11;
const int LED_DRAIN = 12;
const int BUTTON_PIN_PUMP_FILL = 4;
const int BUTTON_PIN_PUMP_DRAIN = 7;
const int PUMP_PIN = 13;
const unsigned long debounceDelay = 50;
const unsigned long calibrationDelay = 2000;
const unsigned long blinkInterval = 500;
const int EEPROM_FILL_ADDRESS = 0;
const int EEPROM_DRAIN_ADDRESS = sizeof(unsigned long);


ButtonState fillButtonState = { HIGH, HIGH, 0, 0, TOGGLE_OFF, true, false };
ButtonState drainButtonState = { HIGH, HIGH, 0, 0, TOGGLE_OFF, true, false };

unsigned long storedFillDuration = 0;
unsigned long storedDrainDuration = 0;

bool fillCalMode = false;
bool drainCalMode = false;
unsigned long fillCalStart = 0;
unsigned long drainCalStart = 0;

bool pumpRunning = false;
unsigned long pumpStartedAt = 0;
unsigned long pumpDuration = 0;
int activePumpLed = -1;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup");

  EEPROM.get(EEPROM_FILL_ADDRESS, storedFillDuration);
  EEPROM.get(EEPROM_DRAIN_ADDRESS, storedDrainDuration);

  Serial.print("Stored Fill Calibration: ");
  Serial.println(storedFillDuration);
  Serial.print("Stored Drain Calibration: ");
  Serial.println(storedDrainDuration);

  pinMode(LED_FILL, OUTPUT);
  pinMode(LED_DRAIN, OUTPUT);
  pinMode(BUTTON_PIN_PUMP_DRAIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN_PUMP_FILL, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);
}

void loop() {
  fillButtonState = handleButton(BUTTON_PIN_PUMP_FILL, fillButtonState, debounceDelay, calibrationDelay);
  drainButtonState = handleButton(BUTTON_PIN_PUMP_DRAIN, drainButtonState, debounceDelay, calibrationDelay);

  // Handle Fill Calibration Mode
  if (fillButtonState.toggleState == TOGGLE_CAL) {
    handleCalibrationMode(fillCalMode, fillCalStart, LED_FILL, blinkInterval);
  } else {
    if (fillCalMode) {
      finishCalibration(fillCalStart, storedFillDuration, EEPROM_FILL_ADDRESS, "Fill");
      fillCalMode = false;
    }

    if (fillButtonState.justToggledOn && !pumpRunning) {
      runPump(LED_FILL, storedFillDuration);
    }
  }

  // Handle Drain Calibration Mode
  if (drainButtonState.toggleState == TOGGLE_CAL) {
    handleCalibrationMode(drainCalMode, drainCalStart, LED_DRAIN, blinkInterval);
  } else {
    if (drainCalMode) {
      finishCalibration(drainCalStart, storedDrainDuration, EEPROM_DRAIN_ADDRESS, "Drain");
      drainCalMode = false;
    }

    if (drainButtonState.justToggledOn && !pumpRunning) {
      runPump(LED_DRAIN, storedDrainDuration);
    }
  }

  if (pumpRunning && (millis() - pumpStartedAt >= pumpDuration) &&
      fillButtonState.toggleState != TOGGLE_CAL &&
      drainButtonState.toggleState != TOGGLE_CAL) {
    digitalWrite(activePumpLed, LOW);
    pumpRunning = false;
    setPump(false);
    fillButtonState.toggleState = TOGGLE_OFF;
    drainButtonState.toggleState = TOGGLE_OFF;
    activePumpLed = -1;
  }
}

void runPump(int outputPin, unsigned long durationInMs) {
  digitalWrite(outputPin, HIGH);
  pumpStartedAt = millis();
  pumpDuration = durationInMs;
  activePumpLed = outputPin;
  pumpRunning = true;
  

  setPump(true);
}

ButtonState handleButton(int pin, ButtonState state, unsigned long debounceDelay, unsigned long calibrationDelay) {
  int reading = digitalRead(pin);
  state.justToggledOn = false;

  if (reading != state.previousState) {
    state.lastDebounceTime = millis();
  }

  if (millis() - state.lastDebounceTime > debounceDelay) {
    if (reading != state.currentState) {
      state.currentState = reading;

      if (state.currentState == LOW) {
        state.pressStartTime = millis();
        state.handled = false;
      } else {
        if (!state.handled) {
          if (millis() - state.pressStartTime < calibrationDelay) {
            int oldState = state.toggleState;
            state.toggleState = !state.toggleState;
            if (oldState == TOGGLE_OFF && state.toggleState == TOGGLE_ON) {
              state.justToggledOn = true;
            }
          }
          state.handled = true;
        }
      }
    }
  }

  if (state.currentState == LOW && !state.handled) {
    if (millis() - state.pressStartTime >= calibrationDelay) {
      state.toggleState = TOGGLE_CAL;
      state.handled = true;
    }
  }

  state.previousState = reading;
  return state;
}

void handleCalibrationMode(bool &wasInCalMode, unsigned long &startedCalAt, int ledPin, unsigned long blinkInterval) {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;

  if (!wasInCalMode) {
    startedCalAt = millis();
    wasInCalMode = true;
    Serial.println("Cal mode active");
  

    // Start pump
    digitalWrite(ledPin, HIGH);
    pumpStartedAt = millis(); // <-- added this line
    setPump(true);


    activePumpLed = ledPin;

    // Reset blink variables
    lastBlinkTime = millis();
    ledState = false;
  }

  // Blink LED while pump is running
  if (millis() - lastBlinkTime >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);
    lastBlinkTime = millis();
  }
}

void setPump(bool turnOnPump) {
  pumpRunning = turnOnPump;
  digitalWrite(PUMP_PIN, turnOnPump);
  Serial.println("Pump is now " + String(turnOnPump ? "ON" : "OFF"));
}

void finishCalibration(unsigned long startedCalAt, unsigned long &storedDuration, int eepromAddr, const char* label) {
  unsigned long duration = millis() - startedCalAt;
  EEPROM.put(eepromAddr, duration);
  storedDuration = duration;
  Serial.print(label);
  Serial.print(" calibration duration saved: ");
  Serial.println(duration);

  // Stop pump
  if (pumpRunning) {
    digitalWrite(activePumpLed, LOW);
    pumpRunning = false;
    activePumpLed = -1;
    setPump(false);
  }
}
