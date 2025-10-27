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
  power_timer0_disable();  // Disable Timer0 (used by millis()/micros())
  power_timer1_disable();  // Disable Timer1
  power_usi_disable();     // Disable USI (I2C, SPI)
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

/**
 * @brief Pin Change Interrupt Service Routine.
 * This fires on EVERY pin change for PB0-PB5 (if enabled).
 */
ISR(PCINT0_vect) {
  static unsigned long lastInterruptTime = 0;

  // Check if the change was on our IR pin (optional but good practice)
  if (!(PINB & (1 << IR_RECEIVE_PIN))) {
    // Pin is LOW (start of a mark)
  } else {
    // Pin is HIGH (start of a space)
  }

  unsigned long now = micros();
  unsigned int pulseDuration = now - lastInterruptTime;
  lastInterruptTime = now;

  // Ignore glitches (anything under 100us)
  if (pulseDuration < 100) {
    return;
  }

  // Check for a long gap (timeout) which signifies a new code
  if (pulseDuration > 12000) {
    necPulseIndex = 0; // Reset
  }

  // Store the pulse duration
  if (necPulseIndex < NEC_PULSE_COUNT) {
    necPulses[necPulseIndex] = pulseDuration;
    necPulseIndex++;
  }

  // If we have collected all 67 pulses, set the flag for the main loop
  if (necPulseIndex == NEC_PULSE_COUNT) {
    necDataReady = true;
    // Main loop will reset necPulseIndex after processing
  }
}

/**
 * @brief Checks if an actual timing is within tolerance of an expected timing.
 * @param actual The measured duration in microseconds.
 * @param expected The NEC protocol's expected duration in microseconds.
 * @return True if within ~25% tolerance, false otherwise.
 */
bool checkTiming(unsigned int actual, unsigned int expected) {
  // Allow a 25% tolerance window
  return (actual > (expected * 0.75)) && (actual < (expected * 1.25));
}

/**
 * @brief Decodes the raw pulse data stored in necPulses[].
 * @return The 32-bit decoded NEC code (LSB first), or 0 if decoding fails.
 */
uint32_t decodeNEC() {
  // Step 1: Check the 9ms LOW (Mark) + 4.5ms HIGH (Space) start condition
  // necPulses[0] is the 9ms mark, necPulses[1] is the 4.5ms space
  if (!checkTiming(necPulses[0], 9000) || !checkTiming(necPulses[1], 4500)) {
    return 0; // Invalid start pulse
  }

  uint32_t data = 0;
  uint8_t pulseIdx = 2; // Start checking data from the 3rd pulse (index 2)

  // Step 2: Loop through all 32 bits
  for (int i = 0; i < 32; i++) {
    // All bits start with a 562.5us Mark (LOW pulse)
    if (!checkTiming(necPulses[pulseIdx], 562)) {
      return 0; // Error: bit mark is wrong duration
    }
    pulseIdx++;

    // Now check the following Space (HIGH pulse) to determine 0 or 1
    if (checkTiming(necPulses[pulseIdx], 1687)) {
      // It's a '1' (long space)
      data |= (1UL << i); // Set the bit (LSB first)
    } else if (checkTiming(necPulses[pulseIdx], 562)) {
      // It's a '0' (short space)
      // We do nothing, the bit in 'data' is already 0
    } else {
      // Error: space is not a valid '0' or '1'
      return 0;
    }
    pulseIdx++;
  }

  // Step 3: Check the final stop bit (a 562.5us Mark)
  if (!checkTiming(necPulses[pulseIdx], 562)) {
     // This check is optional but good for validation
  }

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

    // 2. Initialize IR Receiver Pin and Interrupts
    pinMode(IR_RECEIVE_PIN, INPUT_PULLUP); // Enable internal pull-up
    GIMSK |= (1 << PCIE);                  // Enable Pin Change Interrupts
    PCMSK |= (1 << IR_RECEIVE_PIN);        // Enable PCINT for our pin (PB3)
    sei();                                 // Enable global interrupts
    goToSleep();
  }
}

void loop() {
  if (!isTriggered && necDataReady) {
    cli();
    
    // Copy volatile data to local variables
    unsigned int localPulses[NEC_PULSE_COUNT];
    memcpy(localPulses, (const void*)necPulses, sizeof(necPulses));
    
    // Reset receiver state
    necDataReady = false;
    necPulseIndex = 0;
    
    // Re-enable interrupts
    sei();

    // Decode the copied data
    uint32_t decodedData = decodeNEC();

    // Check if the decoded code matches our trigger code
    if (decodedData == TRIGGER_CODE) {

      // --- !!! DEVICE TRIGGERED !!! ---

      // 1. Set the triggered color pattern
      setZombie();

      // 2. Save the triggered state to EEPROM permanently
      EEPROM.write(EEPROM_ADDR, TRIGGERED_VALUE);

      // 3. Update our state variable
      isTriggered = true;

      // 4. Stop listening for IR (disable Pin Change Interrupt)
      GIMSK &= ~(1 << PCIE);
    }
  }
}
