// Required Libraries
#include <Adafruit_NeoPixel.h> // For controlling the NeoPixel LED strip
#include <EEPROM.h>            // For persistent storage of the seed index
#include <stdlib.h>            // For random() function, often included by Arduino.h, but explicit for clarity

// --- Pin Definitions ---
// NeoPixel LED strip connected to ATTiny85 PB4 (Physical Pin 3 on the IC)
#define NEOPIXEL_PIN 4
// IR LEDs connected via MOSFET to ATTiny85 PB2 (Physical Pin 7 on the IC)
// Mapped to Arduino digital pin 2 for clarity.
#define IR_TX_PIN 2
// TSOP38238 IR receiver connected to ATTiny85 PB3 (Physical Pin 2 on the IC)
// Mapped to Arduino digital pin 3 for clarity.
#define IR_RX_PIN 3

// --- NeoPixel Configuration ---
// Define the number of LEDs in your NeoPixel strip. Adjusted to 16.
#define NUM_PIXELS 16
// Initialize the NeoPixel strip object:
// NUM_PIXELS: Number of pixels in the strip
// NEOPIXEL_PIN: The digital pin connected to the strip's data input
// NEO_GRB + NEO_KHZ800: Pixel type (GRB color order, 800 KHz data rate)
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- IR Transmission Configuration (NEC Protocol) ---
#define IR_FREQ 38000 // Target carrier frequency for IR (38 kHz)
// Calculate half-period for the 38kHz carrier wave.
// This is the ON/OFF duration for each pulse to create the 38kHz square wave.
#define IR_HALF_PERIOD_US (1000000 / IR_FREQ / 2) // ~13 microseconds

// --- Seed Management ---
// Array of 5 hardcoded unsigned long "seeds" - NEW VALUES.
// These values will be used to generate LED patterns and for IR communication.
const unsigned long SEEDS[] = {
    0xF00DCAFEUL, // New Seed 1
    0xBADF00D5UL, // New Seed 2
    0xBEAFFACEUL, // New Seed 3
    0xDEADC0DEUL, // New Seed 4
    0xFEEDFACEUL  // New Seed 5
};
// Calculate the total number of seeds in the array
#define NUM_SEEDS (sizeof(SEEDS) / sizeof(SEEDS[0]))

// EEPROM addresses for storing seed index and a "magic byte" for power-on detection
#define EEPROM_SEED_INDEX_ADDR 0   // Stores the current index of the seed
#define EEPROM_MAGIC_ADDR 1        // Stores a magic value to detect first power-on
#define EEPROM_MAGIC_VALUE 0xAC    // A unique byte value for detection

// Variables to hold the currently active seed and its index
unsigned long currentSeed;
int currentSeedIndex = 0;

// Flag to track if the LEDs are currently showing the "matched" red state
bool isMatchedState = false;

// Global variable to store the MCU Status Register (MCUSR) value.
// This is read very early in setup() to determine the reset source.
byte mcu_status_reg;

// --- Function Prototypes ---
// IR Transmission
void generate38kHzBurst(unsigned int duration_us);
void sendNECBit(int bitValue);
void sendNEC(unsigned long data);

// IR Reception
long measurePulse(int pin, int type, unsigned long timeout_us);
unsigned long receiveNEC();

// NeoPixel Control
void applySeedToLEDs(unsigned long seed);
void showAllRed();
void showBootPattern(); // Kept for consistency, but not used in the simplified setup

// Seed Management (EEPROM & Reset Detection)
void readSeedIndexFromEEPROM();

void setup() {
    // IMPORTANT: Read MCUSR immediately at the very beginning of setup().
    // This register contains flags about the last reset cause.
    mcu_status_reg = MCUSR;
    // Clear the reset flags for the next reset (so we can detect it next time)
    MCUSR = 0;

    // --- Seed the random number generator using analog noise from IR_RX_PIN ---
    // Even though IR_RX_PIN is usually a digital input for the TSOP,
    // it can also be read as an analog input (ADC3 on PB3).
    // Ambient electrical noise or minor variations in sensor output can provide entropy.
    long seed_value = 0;
    // Perform multiple analog reads and sum them to create a more robust seed.
    // This introduces variability based on subtle environmental/electrical noise.
    for (int i = 0; i < 500; i++) { // Read 500 times for a stronger seed
        seed_value += analogRead(IR_RX_PIN);
    }
    randomSeed(seed_value); // Use the collected analog noise as the seed for random()

    // Initialize pin modes using Arduino digital pin numbers
    pinMode(IR_TX_PIN, OUTPUT); // IR LED control pin (Arduino Digital Pin 2 / PB2)
    digitalWrite(IR_TX_PIN, LOW); // Ensure IR LED is off initially
    pinMode(IR_RX_PIN, INPUT);    // IR receiver data pin (Arduino Digital Pin 3 / PB3)
    // Note: pinMode(IR_RX_PIN, INPUT) is typically used for digitalRead,
    // but analogRead() can still access the ADC on this pin.


    // Initialize NeoPixel strip
    strip.begin(); // Initializes the NeoPixel library
    strip.show();  // Turns off all pixels initially

    // Determine the current seed index based on EEPROM and reset cause
    readSeedIndexFromEEPROM();

    // Set the current seed based on the determined index
    currentSeed = SEEDS[currentSeedIndex];

    // On startup (including soft reset), show the seed-derived pattern.
    // This will re-apply the same pattern if it was a soft reset, preserving visual state.
    applySeedToLEDs(currentSeed);

    // Removed the explicit IR blast on External Reset (RESET button press)
    // to align with the request "not update ... when the RESET button is clicked".
    // Periodic transmission in loop() will handle sending the seed.
}

void loop() {
    // --- Periodic IR Transmission Logic (every 60 seconds) ---
    // This ensures the board is regularly transmitting its seed for testing/functionality.
    static unsigned long lastPeriodicTxTime = 0;
    const unsigned long PERIODIC_TX_INTERVAL = 60000; // 60 seconds

    if (millis() - lastPeriodicTxTime >= PERIODIC_TX_INTERVAL) {
        sendNEC(currentSeed);
        lastPeriodicTxTime = millis();
    }

    // --- LED Update Timeout and IR Reception Logic ---
    static unsigned long lastLEDUpdateTime = 0;
    const unsigned long LED_UPDATE_INTERVAL = 9000; // 9 seconds timeout for LED updates

    // Attempt to receive IR signal
    unsigned long receivedSeed = receiveNEC();

    if (receivedSeed != 0) { // If a valid NEC signal was received
        if (receivedSeed == currentSeed) {
            // If the received seed matches the current device's seed,
            // change all LEDs to red and set the matched state flag.
            // LEDs will stay red until a different valid seed is received.
            if (!isMatchedState) { // Only update if not already in the matched state
                showAllRed();
                isMatchedState = true;
                lastLEDUpdateTime = millis(); // Reset update time for next pattern display
            }
        } else {
            // If a valid, but *different* seed is received, revert to the seed-derived pattern.
            // This also handles the case where the board was red (matched) and now receives a mismatch.
            if (isMatchedState) { // Only revert if previously in the matched state
                applySeedToLEDs(currentSeed);
                isMatchedState = false;
                lastLEDUpdateTime = millis(); // Reset update time for next pattern display
            }
            // If not in matched state and receives a different seed, simply do nothing (stays on current seed pattern).
            // The next default pattern update will still abide by its 9-second interval.
        }
    }

    // --- Default Pattern Update (if not in matched state and timeout elapsed) ---
    // This updates the LED pattern only if no IR event caused an update
    // AND it's been longer than LED_UPDATE_INTERVAL since the last update.
    // This prevents constantly redrawing the pattern, saving CPU cycles.
    if (!isMatchedState && (millis() - lastLEDUpdateTime >= LED_UPDATE_INTERVAL)) {
        applySeedToLEDs(currentSeed);
        lastLEDUpdateTime = millis(); // Reset the timer
    }
}

// =========================================================================
//                          IR Transmission Functions
// =========================================================================

/**
 * @brief Generates a 38kHz IR carrier burst for a specified duration.
 * This function turns the IR LED on and off rapidly to create the 38kHz modulation.
 * @param duration_us The total duration in microseconds for which the burst should last.
 */
void generate38kHzBurst(unsigned int duration_us) {
    // Calculate the number of ON/OFF cycles needed for the specified duration.
    // Each cycle is 2 * IR_HALF_PERIOD_US (e.g., 13us ON + 13us OFF = 26us per cycle for 38kHz).
    unsigned long num_pulses = duration_us / (IR_HALF_PERIOD_US * 2);

    // Disable interrupts to ensure precise timing for the 38kHz carrier.
    // This prevents other tasks from interfering with the pulse generation.
    noInterrupts();
    for (unsigned long i = 0; i < num_pulses; i++) {
        digitalWrite(IR_TX_PIN, HIGH);           // Turn IR LED ON
        delayMicroseconds(IR_HALF_PERIOD_US);    // Wait for half a period
        digitalWrite(IR_TX_PIN, LOW);            // Turn IR LED OFF
        delayMicroseconds(IR_HALF_PERIOD_US);    // Wait for the other half period
    }
    interrupts(); // Re-enable interrupts after the burst is complete.
}

/**
 * @brief Sends a single bit (0 or 1) using the NEC protocol timings.
 * NEC protocol uses different space durations for 0 and 1 bits.
 * @param bitValue The bit to send (0 or 1).
 */
void sendNECBit(int bitValue) {
    // All bits start with a 560 microsecond mark (burst).
    generate38kHzBurst(560);

    // The space duration determines if it's a 0 or a 1.
    if (bitValue == 0) {
        delayMicroseconds(560);  // 560 us space for a '0' bit
    } else {
        delayMicroseconds(1690); // 1690 us space for a '1' bit
    }
}

/**
 * @brief Sends a 32-bit data word using the NEC IR protocol.
 * The NEC protocol consists of a header, 32 data bits (LSB first), and an end burst.
 * @param data The 32-bit unsigned long data (seed) to be transmitted.
 */
void sendNEC(unsigned long data) {
    // NEC Header: 9ms Mark (burst) followed by 4.5ms Space (no burst).
    generate38kHzBurst(9000);
    delayMicroseconds(4500);

    // Send the 32 data bits. NEC protocol sends the Least Significant Bit (LSB) first.
    for (int i = 0; i < 32; i++) {
        // Extract the i-th bit from 'data' and send it.
        sendNECBit((data >> i) & 0x01);
    }

    // End burst: A final 560 microsecond mark to indicate the end of the transmission.
    generate38kHzBurst(560);

    // A minimum silence period before the next transmission can begin.
    // This helps receivers reset.
    delayMicroseconds(40000);
}

// =========================================================================
//                          IR Reception Functions
// =========================================================================

/**
 * @brief Measures the duration of a pulse (HIGH or LOW) on a given pin.
 * @param pin The digital pin to measure the pulse on.
 * @param type The type of pulse to measure (HIGH or LOW).
 * @param timeout_us The maximum time to wait for the pulse to start or end, in microseconds.
 * @return The duration of the pulse in microseconds, or 0 if a timeout occurs.
 */
long measurePulse(int pin, int type, unsigned long timeout_us) {
    unsigned long start_time = micros();

    // Wait for the pulse to start (i.e., the pin to transition to 'type')
    while (digitalRead(pin) != type) {
        if (micros() - start_time > timeout_us) {
            return 0; // Timeout waiting for pulse start
        }
    }
    unsigned long pulse_start_time = micros(); // Mark the exact start of the pulse

    // Wait for the pulse to end (i.e., the pin to transition away from 'type')
    while (digitalRead(pin) == type) {
        // Timeout check: Use the actual pulse start time for the timeout
        if (micros() - pulse_start_time > timeout_us) {
            return 0; // Timeout waiting for pulse end
        }
    }
    // Return the duration of the pulse
    return micros() - pulse_start_time;
}

/**
 * @brief Attempts to receive and decode a 32-bit data word using the NEC IR protocol.
 * The TSOP38238 receiver outputs LOW when it detects a 38kHz burst and HIGH when idle.
 * @return The 32-bit unsigned long data (seed) if successfully received, otherwise 0.
 * Returns 0 for timeouts or invalid pulse durations, indicating no valid NEC signal.
 */
unsigned long receiveNEC() {
    unsigned long received_data = 0;
    long pulse_duration;
    const unsigned long NEC_TIMEOUT = 15000; // General timeout for reception phases

    // Step 1: Wait for the initial 9ms Mark (receiver outputs LOW).
    // The receiver is normally HIGH. It goes LOW when it detects IR.
    unsigned long initial_wait_start = micros();
    while (digitalRead(IR_RX_PIN) == HIGH) {
        if (micros() - initial_wait_start > NEC_TIMEOUT) return 0; // Timeout waiting for signal start
    }

    // Measure the LOW pulse (Mark) of the header. Expected: ~9000us
    pulse_duration = measurePulse(IR_RX_PIN, LOW, 10000);
    // Allow for some tolerance: 8ms to 10ms for 9ms mark
    if (pulse_duration < 8000 || pulse_duration > 10000) {
        return 0; // Invalid header mark
    }

    // Measure the HIGH pulse (Space) of the header. Expected: ~4500us
    pulse_duration = measurePulse(IR_RX_PIN, HIGH, 5000);
    // Allow for some tolerance: 4ms to 5ms for 4.5ms space
    if (pulse_duration < 4000 || pulse_duration > 5000) {
        return 0; // Invalid header space
    }

    // Step 2: Read 32 data bits.
    // Each bit starts with a 560us Mark (LOW pulse) followed by a Space (HIGH pulse).
    // Space duration: ~560us for '0', ~1690us for '1'.
    for (int i = 0; i < 32; i++) {
        // Measure the Mark (LOW pulse). Expected: ~560us
        pulse_duration = measurePulse(IR_RX_PIN, LOW, 1000); // Max 1ms timeout for a bit mark
        // Tolerance for 560us mark: 400us to 700us
        if (pulse_duration < 400 || pulse_duration > 700) {
            return 0; // Invalid bit mark
        }

        // Measure the Space (HIGH pulse). This determines the bit value.
        pulse_duration = measurePulse(IR_RX_PIN, HIGH, 2000); // Max 2ms timeout for a bit space

        if (pulse_duration > 400 && pulse_duration < 800) {
            // If space is ~560us (within 400-800us range), it's a '0' bit.
            // No action needed as the bit is already 0 in 'received_data' for this position.
        } else if (pulse_duration > 1400 && pulse_duration < 1900) {
            // If space is ~1690us (within 1400-1900us range), it's a '1' bit.
            // Set the corresponding bit in 'received_data'. (LSB first)
            received_data |= (1UL << i);
        } else {
            // If space duration does not match either 0 or 1, it's an invalid signal.
            return 0;
        }
    }

    // Step 3: Measure the final 560us Mark (trailing burst).
    pulse_duration = measurePulse(IR_RX_PIN, LOW, 1000);
    // Tolerance for 560us mark: 400us to 700us
    if (pulse_duration < 400 || pulse_duration > 700) {
        return 0; // Invalid trailing mark
    }

    // Successfully received and decoded a 32-bit NEC signal.
    return received_data;
}

// =========================================================================
//                          NeoPixel Control Functions
// =========================================================================

/**
 * @brief Applies a visual pattern to the NeoPixel strip based on the given seed.
 * The pattern generated is unique for each seed, ensuring consistency.
 * Now implements the color constraints: R_max = 255-G-B, G_max = 150-B, B_any 0-100.
 * @param seed The 32-bit unsigned long seed to use for pattern generation.
 */
void applySeedToLEDs(unsigned long seed) {
    strip.clear(); // Turn off all pixels first to clear previous pattern.

    // Extract base values from the seed for overall color tint.
    // These values are used to influence the "randomness" or variety.
    uint8_t seed_val_r_influence = (seed >> 24) & 0xFF; // Top byte for red influence
    uint8_t seed_val_g_influence = (seed >> 16) & 0xFF; // Next byte for green influence
    uint8_t seed_val_b_influence = (seed >> 8) & 0xFF;  // Next byte for blue influence

    for (int i = 0; i < NUM_PIXELS; i++) {
        // 1. Calculate Blue channel: any number from 0 to 100
        // Use seed_val_b_influence and pixel index for variation within the 0-100 range.
        uint8_t b_raw = (seed_val_b_influence + (i * 7)) % 101; // Vary within 0-100
        uint8_t b = min(b_raw, (uint8_t)100); // Ensure it does not exceed 100
        b = max(b, (uint8_t)0);   // Ensure it's not less than 0

        // 2. Calculate Green channel: max 150 - B
        // Determine the maximum allowed value for Green based on the calculated Blue
        uint8_t g_max_allowed = 150 - b;
        g_max_allowed = max(g_max_allowed, (uint8_t)0); // Ensure max allowed is not negative

        // Use seed_val_g_influence and pixel index for variation within g_max_allowed.
        uint8_t g_raw = (seed_val_g_influence + (i * 9)) % (g_max_allowed + 1); // Vary within 0-g_max_allowed
        uint8_t g = min(g_raw, g_max_allowed);             // Ensure it does not exceed g_max_allowed
        g = max(g, (uint8_t)0);                // Ensure it's not less than 0

        // 3. Calculate Red channel: max 255 - G - B
        // Determine the maximum allowed value for Red based on the calculated Green and Blue
        uint8_t r_max_allowed = 255 - g - b;
        r_max_allowed = max(r_max_allowed, (uint8_t)0); // Ensure max allowed is not negative

        // Use seed_val_r_influence and pixel index for variation within r_max_allowed.
        uint8_t r_raw = (seed_val_r_influence + (i * 11)) % (r_max_allowed + 1); // Vary within 0-r_max_allowed
        uint8_t r = min(r_raw, r_max_allowed);             // Ensure it does not exceed r_max_allowed
        r = max(r, (uint8_t)0);                // Ensure it's not less than 0

        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show(); // Update the NeoPixel strip with the new pattern.
}

/**
 * @brief Sets all NeoPixels to a solid, full red color.
 * Used to indicate a successful IR seed match.
 */
void showAllRed() {
    strip.fill(strip.Color(255, 0, 0)); // Set all pixels to full bright red (255,0,0)
    strip.show(); // Update the strip
}

/**
 * @brief Displays a brief pattern on boot-up (e.g., green for a short duration).
 * This function is included for completeness but is not called in the current setup() for simplicity.
 */
void showBootPattern() {
    strip.fill(strip.Color(0, 100, 0)); // Fill with a dimmer green
    strip.show();                       // Display green
    delay(500);                         // Hold for 0.5 seconds
    strip.clear();                      // Turn off
    strip.show();
}

// =========================================================================
//                         Seed Management (EEPROM) Functions
// =========================================================================

/**
 * @brief Reads the seed index from EEPROM and determines the current seed.
 * This function differentiates between a "power down" (cold boot) and a "reset" (soft boot).
 * - On a "power down" (PORF/BORF flags set), the seed index is chosen randomly.
 * - On a "reset" (EXTRF/WDRF flags set), the seed index remains the same as before the reset.
 * - On first power-up or corrupted EEPROM, it also picks a random seed.
 * The determined index is then stored back into EEPROM for the next boot cycle.
 */
void readSeedIndexFromEEPROM() {
    byte magic = EEPROM.read(EEPROM_MAGIC_ADDR);
    int storedIndex = EEPROM.read(EEPROM_SEED_INDEX_ADDR);

    // If magic byte is not set, or EEPROM data is out of bounds/corrupt,
    // it's considered a fresh start or corrupted EEPROM. Randomize the seed.
    if (magic != EEPROM_MAGIC_VALUE || storedIndex < 0 || storedIndex >= NUM_SEEDS) {
        currentSeedIndex = random(NUM_SEEDS); // Randomly select an index
        EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE); // Write magic byte to mark EEPROM as initialized
    }
    // Check if it was a Power-On Reset (PORF) or Brown-Out Reset (BORF) and EEPROM was valid.
    else if ((mcu_status_reg & (1 << PORF)) || (mcu_status_reg & (1 << BORF))) {
        // This was a cold boot (power down and up). Choose a new random seed.
        currentSeedIndex = random(NUM_SEEDS);
    } else {
        // This was a soft reset (External Reset via reset button, or Watchdog Reset)
        // and EEPROM was valid. Retain the previously stored seed index.
        currentSeedIndex = storedIndex;
    },
    // Always update the EEPROM with the 'currentSeedIndex' for the next boot cycle.
    EEPROM.update(EEPROM_SEED_INDEX_ADDR, currentSeedIndex);
}
