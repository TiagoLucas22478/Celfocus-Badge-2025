#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

// --- Constants ---
#define NEOPIXEL_PIN   4  // ATtiny85 Arduino Pin 4 (PB4)
#define IR_RECEIVE_PIN 3  // ATtiny85 Arduino Pin 3 (PB3)
#define IR_SEND_PIN    2  // ATtiny85 Arduino Pin 2 (PB2)

#define NUM_LEDS 16 // Number of LEDs in the strip

// The NEC code to trigger on and send.
// 41103371 (decimal) is 0x271C0F43.
// We receive and send this as an LSB-first 32-bit value.
#define TRIGGER_CODE 41103371UL

// --- EEPROM ---
#define EEPROM_ADDR       0
#define TRIGGERED_VALUE   0x42 // The meaning of life, the universe and all things
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool isTriggered = false;

// --- Manual IR Receiver Globals ---
// We need to capture 1 start pulse (LOW) + 1 start space (HIGH)
// + 32 bits * (1 mark (LOW) + 1 space (HIGH)) + 1 stop pulse (LOW)
// Total = 1 + 1 + 64 + 1 = 67 pulse/space timings
#define NEC_PULSE_COUNT 67
volatile unsigned int necPulses[NEC_PULSE_COUNT];
volatile uint8_t necPulseIndex = 0;
volatile bool necDataReady = false;

// --- Manual IR Sender Defines ---
#define IR_SEND_PORT PORTB
#define IR_SEND_DDR  DDRB
#define IR_SEND_BIT  (1 << PB2)
#define IR_RECEIVE_PORT_PIN PINB
#define IR_RECEIVE_BIT (1 << PB3)

// --- LED Helper Functions ---
void setHuman() {
  uint32_t pastelPink = strip.Color(200, 140, 150);
  strip.fill(pastelPink);
  strip.show();
}

void setZombie() {
  uint32_t green = strip.Color(0, 255, 0);
  uint32_t red = strip.Color(255, 0, 0);
  strip.fill(green);

  // Set the eyes to red
  strip.setPixelColor(5, red);
  strip.setPixelColor(10, red);

  strip.show();
}

void allLedsOff() {
  strip.fill(strip.Color(0, 0, 0));
  strip.show();
}

void goToSleep() {
  allLedsOff();
  delay(10);
  ADCSRA &= ~(1 << ADEN); // Disable ADC
  power_timer0_disable();  // Disable Timer0 (used by millis()/micros())
  power_timer1_disable();  // Disable Timer1
  power_usi_disable();     // Disable USI (I2C, SPI)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode to max power saving
  cli(); // Disable interrupts
  sleep_enable();
  sleep_cpu(); // Go to sleep and never wake up
}

// --- Manual IR Sender Functions ---

/**
 * @brief Sends a 38kHz IR carrier wave (a "mark") for a duration.
 * @param us Duration in microseconds to send the mark.
 */
void sendMark(uint16_t us) {
  unsigned long start = micros();
  while (micros() - start < us) {
    IR_SEND_PORT |= IR_SEND_BIT;  // Set pin HIGH
    delayMicroseconds(10);        // Approx. half-period (13us) for 38kHz @ 8MHz
    IR_SEND_PORT &= ~IR_SEND_BIT; // Set pin LOW
    delayMicroseconds(10);        // Approx. half-period (13us) for 38kHz @ 8MHz
  }
}

/**
 * @brief Sends no signal (a "space") for a duration.
 * @param us Duration in microseconds for the space.
 */
void sendSpace(uint16_t us) {
  IR_SEND_PORT &= ~IR_SEND_BIT; // Ensure pin is LOW
  delayMicroseconds(us);
}

/**
 * @brief Sends a 32-bit NEC code (LSB first).
 * @param data The 32-bit code to send.
 */
void sendNEC(uint32_t data) {
  // 1. Send NEC Start Pulse
  sendMark(9000); // 9ms Mark
  sendSpace(4500); // 4.5ms Space

  // 2. Send 32 bits of data, LSB first
  for (int i = 0; i < 32; i++) {
    sendMark(562); // 562.5us Mark
    if (data & (1UL << i)) {
      sendSpace(1687); // '1' bit (1.6875ms Space)
    } else {
      sendSpace(562);  // '0' bit (562.5us Space)
    }
  }

  // 3. Send Stop Bit
  sendMark(562); // 562.5us Mark
  sendSpace(100);  // Small gap
}


// --- Manual IR Receiver Functions ---
bool checkTiming(unsigned int actual, unsigned int expected) {
  // Allow a 25% tolerance window
  return (actual > (expected * 0.75)) && (actual < (expected * 1.25));
}

uint32_t receiveNEC() {
  unsigned long startTime, markDuration, spaceDuration;
  unsigned long timeoutStart;

  // Wait for the pin to go LOW (start of 9ms pulse)
  // The TSOP38238 output is active LOW.
  // We'll wait up to 100ms for a signal to start.
  timeoutStart = micros();
  while (IR_RECEIVE_PORT_PIN & IR_RECEIVE_BIT) {
    // Pin is HIGH (idle)
    if (micros() - timeoutStart > 100000) {
      return 0; // 100ms timeout
    }
  }

  // --- Start Pulse Detected ---

  // 1. Measure the 9ms Mark (LOW)
  startTime = micros();
  while (!(IR_RECEIVE_PORT_PIN & IR_RECEIVE_BIT)) {
    // Pin is LOW
    if (micros() - startTime > 10000) {
      return 0; // 10ms timeout (pulse too long)
    }
  }
  markDuration = micros() - startTime;

  // 2. Measure the 4.5ms Space (HIGH)
  startTime = micros();
  while (IR_RECEIVE_PORT_PIN & IR_RECEIVE_BIT) {
    // Pin is HIGH
    if (micros() - startTime > 5000) {
      return 0; // 5ms timeout (space too long)
    }
  }
  spaceDuration = micros() - startTime;

  // 3. Check if it's a valid NEC Start Condition
  if (!checkTiming(markDuration, 9000) || !checkTiming(spaceDuration, 4500)) {
    return 0; // Not a valid start
  }

  // --- Data Bits ---
  uint32_t data = 0;
  for (int i = 0; i < 32; i++) {
    // 1. Measure the 562us Bit Mark (LOW)
    startTime = micros();
    while (!(IR_RECEIVE_PORT_PIN & IR_RECEIVE_BIT)) {
      // Pin is LOW
      if (micros() - startTime > 700) {
        return 0; // 700us timeout
      }
    }
    markDuration = micros() - startTime;

    // 2. Measure the Bit Space (HIGH)
    startTime = micros();
    while (IR_RECEIVE_PORT_PIN & IR_RECEIVE_BIT) {
      // Pin is HIGH
      if (micros() - startTime > 2000) {
        return 0; // 2ms timeout
      }
    }
    spaceDuration = micros() - startTime;

    // 3. Check Mark and determine bit value from Space
    if (!checkTiming(markDuration, 562)) {
      return 0; // Invalid bit mark
    }

    if (checkTiming(spaceDuration, 1687)) {
      // It's a '1' (long space)
      data |= (1UL << i); // Set the bit (LSB first)
    } else if (checkTiming(spaceDuration, 562)) {
      // It's a '0' (short space)
      // We do nothing, the bit in 'data' is already 0
    } else {
      return 0; // Invalid space duration
    }
  }

  // If we got here, all 32 bits were received successfully
  return data;
}

// --- Main Program ---

void setup() {
  strip.begin();
  strip.show();
  if (EEPROM.read(EEPROM_ADDR) == TRIGGERED_VALUE) {
    isTriggered = true;
  } else {
    isTriggered = false;
  }

  if (isTriggered) {
    setZombie();
    // 1. Initialize IR Sender Pin
    IR_SEND_DDR |= IR_SEND_BIT; // Set IR_SEND_PIN (PB2) to OUTPUT
    IR_SEND_PORT &= ~IR_SEND_BIT; // Start LOW

    // 2. Send the IR code repeatedly for 10 seconds
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      sendNEC(TRIGGER_CODE);
      delay(110); // Standard NEC repeat gap is ~108ms
    }
    goToSleep();

  } else {
    setHuman();
    pinMode(IR_RECEIVE_PIN, INPUT_PULLUP);
  }
}

void loop() {
  if(!isTriggered) {
  uint32_t decodedData = receiveNEC();
    if (decodedData == TRIGGER_CODE) {
      // --- !!! INFECTED !!! ---
      setZombie();
      EEPROM.write(EEPROM_ADDR, TRIGGERED_VALUE);
      isTriggered = true;
    }
  }
}
