#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>

// =============================================================================
// Pin Definitions - ESP32-S3 SuperMini
// =============================================================================

// I2S Audio (Shared Bus)
#define I2S_WS          4   // Word Select (INMP441 + MAX98357A)
#define I2S_BCK         5   // Bit Clock (INMP441 + MAX98357A)
#define I2S_DIN         6   // Data In from INMP441 (mic)
#define I2S_DOUT        7   // Data Out to MAX98357A (amp)

// I2C Display
#define I2C_SDA         8   // SSD1306 OLED
#define I2C_SCL         9   // SSD1306 OLED

// Servos
#define SERVO_PAN       1   // SG90 horizontal
#define SERVO_TILT      2   // SG90 vertical

// LED & Button
#define WS2812_DATA     48  // RGB LED
#define BUTTON_PIN      0   // Activation button (active low, internal pullup)

// =============================================================================
// I2S Configuration
// =============================================================================

#define I2S_SAMPLE_RATE     16000
#define I2S_SAMPLE_BITS     I2S_BITS_PER_SAMPLE_16BIT
#define I2S_CHANNEL_NUM     1
#define I2S_READ_LEN        1024

// I2S port numbers
#define I2S_PORT_MIC        I2S_NUM_0
#define I2S_PORT_AMP        I2S_NUM_1

// =============================================================================
// I2C Configuration
// =============================================================================

#define OLED_ADDRESS        0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

// =============================================================================
// Servo Configuration
// =============================================================================

#define SERVO_MIN_PULSE     500   // microseconds
#define SERVO_MAX_PULSE     2500  // microseconds
#define SERVO_CENTER        90    // degrees

// =============================================================================
// Function Prototypes
// =============================================================================

void setupI2S();
void setupI2C();
void setupServos();
void setupLED();
void setupButton();

// =============================================================================
// Setup
// =============================================================================

void setup() {
    // Initialize serial for debugging and host communication
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait up to 3s for serial

    Serial.println();
    Serial.println("=================================");
    Serial.println("  Hinze Robot Companion v0.1");
    Serial.println("  ESP32-S3 SuperMini");
    Serial.println("=================================");

    // Initialize subsystems
    Serial.println("[INIT] Setting up I2C...");
    setupI2C();

    Serial.println("[INIT] Setting up I2S audio...");
    setupI2S();

    Serial.println("[INIT] Setting up servos...");
    setupServos();

    Serial.println("[INIT] Setting up LED...");
    setupLED();

    Serial.println("[INIT] Setting up button...");
    setupButton();

    Serial.println("[INIT] Setup complete!");
    Serial.println();
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
    // TODO: Implement main state machine
    // - Check for serial commands
    // - Handle button press
    // - Update animations
    // - Process audio

    delay(10);
}

// =============================================================================
// Initialization Functions
// =============================================================================

void setupI2C() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // 400kHz fast mode

    // Scan for I2C devices
    Serial.println("[I2C] Scanning...");
    int deviceCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C] Device found at 0x%02X", addr);
            if (addr == OLED_ADDRESS) {
                Serial.print(" (OLED)");
            }
            Serial.println();
            deviceCount++;
        }
    }
    Serial.printf("[I2C] %d device(s) found\n", deviceCount);
}

void setupI2S() {
    // Configure I2S for microphone input (INMP441)
    i2s_config_t i2s_config_mic = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_READ_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config_mic = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN
    };

    esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_config_mic, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] Mic driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT_MIC, &pin_config_mic);
    if (err != ESP_OK) {
        Serial.printf("[I2S] Mic pin config failed: %d\n", err);
        return;
    }

    Serial.println("[I2S] Microphone configured");

    // Configure I2S for amplifier output (MAX98357A)
    i2s_config_t i2s_config_amp = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_READ_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config_amp = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    err = i2s_driver_install(I2S_PORT_AMP, &i2s_config_amp, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] Amp driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT_AMP, &pin_config_amp);
    if (err != ESP_OK) {
        Serial.printf("[I2S] Amp pin config failed: %d\n", err);
        return;
    }

    Serial.println("[I2S] Amplifier configured");
}

void setupServos() {
    // Configure servo pins as outputs
    pinMode(SERVO_PAN, OUTPUT);
    pinMode(SERVO_TILT, OUTPUT);

    // TODO: Initialize servo library (ESP32Servo or ledc-based PWM)
    // For now, just configure the pins

    Serial.printf("[SERVO] Pan on GPIO %d, Tilt on GPIO %d\n", SERVO_PAN, SERVO_TILT);
}

void setupLED() {
    pinMode(WS2812_DATA, OUTPUT);
    digitalWrite(WS2812_DATA, LOW);

    // TODO: Initialize FastLED or Adafruit_NeoPixel library

    Serial.printf("[LED] WS2812 on GPIO %d\n", WS2812_DATA);
}

void setupButton() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // TODO: Attach interrupt for button press detection

    Serial.printf("[BUTTON] Input on GPIO %d (active low)\n", BUTTON_PIN);
}
