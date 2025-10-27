/*
  ATtiny85 Custom Circuit Firmware
  
  Controls a 16-LED NeoPixel strip, an IR receiver, and an IR transmitter.
  
  Hardware Connections (Arduino Pin Numbers):
  - PIN 4 (PB4, Physical Pin 3): NeoPixel Data In
  - PIN 3 (PB3, Physical Pin 2): IR Receiver (TSOP38238) Data Out
  - PIN 2 (PB2, Physical Pin 7): IR LED Anode (via a current-limiting resistor)
*/

// --- Include Libraries ---
#include <Adafruit_NeoPixel.h>
#include <IRremote.hpp> // Use the modern IRremote library (v4.0.0 or newer)
#include <EEPROM.h>
#include <avr/sleep.h> // For power-saving mode
#include <avr/power.h> // For peripheral power-down

// --- Constants ---
#define NEOPIXEL_PIN   4  // ATtiny85 Arduino Pin 4 (PB4)
#define IR_RECEIVE_PIN 3  // ATtiny85 Arduino Pin 3 (PB3)
#define IR_SEND_PIN    2  // ATtiny85 Arduino Pin 2 (PB2)

#define NUM_LEDS 16 // Number of LEDs in the strip

// The NEC code to trigger on and send.
// 41103371 (decimal) is 0x271C0F43. We define it as an Unsigned Long.
#define TRIGGER_CODE 41103371UL

// --- EEPROM ---
#define EEPROM_ADDR       0     // Address to store the trigger state
#define TRIGGERED_VALUE   0x42  // A "magic number" to signify "triggered"

// --- Global Objects ---
// Parameter 1 = Number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = Pixel type flags, add together as needed:
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixels)
//   NEO_KHZ800  800 KHz bitstream (most modern NeoPixels)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- Global State ---
bool isTriggered = false;

// --- Helper Functions ---

/**
 * @brief Sets all LEDs to a pastel pink color.
 */
void setPastelPink() {
  // A pastel pink (R=255, G=182, B=193) dimmed for a "pastel" look
  uint32_t pastelPink = strip.Color(200, 140, 150);
  strip.fill(pastelPink);
  strip.show();
}

/**
 * @brief Sets the "triggered" color pattern: all green, 6th and 11th red.
 */
void setTriggeredColors() {
  uint32_t green = strip.Color(0, 255, 0);
  uint32_t red = strip.Color(255, 0, 0);

  strip.fill(green);
  
  // Set 6th LED (index 5) and 11th LED (index 10) to red
  strip.setPixelColor(5, red); 
  strip.setPixelColor(10, red);
  
  strip.show();
}

/**
 * @brief Turns all LEDs off.
 */
void allLedsOff() {
  strip.fill(strip.Color(0, 0, 0));
  strip.show();
}

/**
 * @brief Puts the ATtiny85 into deep sleep mode (power-down).
 * This function will not return. The chip must be power-cycled to restart.
 */
void goToSleep() {
  allLedsOff();
  delay(10); // Allow strip to update

  // Disable all peripherals for maximum power saving
  ADCSRA &= ~(1 << ADEN); // Disable ADC
  power_timer0_disable();  // Disable Timer0 (used by millis() and IR send)
  power_timer1_disable();  // Disable Timer1 (used by IR receive)
  power_usi_disable();     // Disable USI (I2C, SPI)

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode to max power saving
  cli(); // Disable interrupts
  sleep_enable();
  sleep_cpu(); // Go to sleep and never wake up
}

// --- Main Program ---

void setup() {
  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Check EEPROM to see if we've been triggered before
  if (EEPROM.read(EEPROM_ADDR) == TRIGGERED_VALUE) {
    isTriggered = true;
  } else {
    isTriggered = false;
  }

  // --- Handle startup logic based on state ---
  if (isTriggered) {
    // === TRIGGERED STARTUP ===
    // This runs if the chip was already triggered and is restarting.
    
    // 1. Initialize IR Sender
    IrSender.begin(IR_SEND_PIN); // No feedback LED

    // 2. Send the IR code repeatedly for 10 seconds
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      // Send the raw 32-bit NEC code. 0 repeats.
      IrSender.sendNECRaw(TRIGGER_CODE, 0);
      delay(110); // Standard NEC repeat gap is ~108ms
    }

    // 3. Go to deep sleep
    goToSleep(); // This function does not return

  } else {
    // === NORMAL STARTUP ===
    // This runs on first boot or if never triggered.
    
    // 1. Set initial colors
    setPastelPink();

    // 2. Initialize IR Receiver
    IrReceiver.begin(IR_RECEIVE_PIN); // Starts listening
  }
}

void loop() {
  if (isTriggered) {
    // If we were triggered *during this session*, we just stop.
    // The next reboot will handle the IR transmit and sleep.
    // This loop will just show the triggered colors.
    while (true) {
      // Do nothing
    }
  }

  // --- Main Listening Loop (only runs if not triggered) ---
  if (IrReceiver.decode()) {
    
    // Check if the received code is NEC protocol AND matches our trigger code
    if (IrReceiver.decodedIRData.protocol == NEC &&
        IrReceiver.decodedIRData.decodedRawData == TRIGGER_CODE) {

      // --- !!! DEVICE TRIGGERED !!! ---

      // 1. Set the triggered color pattern
      setTriggeredColors();

      // 2. Save the triggered state to EEPROM permanently
      EEPROM.write(EEPROM_ADDR, TRIGGERED_VALUE);

      // 3. Update our state variable
      isTriggered = true;

      // 4. Stop the IR receiver (saves power and stops listening)
      IrReceiver.disableIRIn();
    }
    
    // Resume listening for the next code
    IrReceiver.resume();
  }
}
