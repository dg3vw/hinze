#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>
#include <driver/i2s.h>
#include <ArduinoJson.h>

// =============================================================================
// Pin Definitions - ESP32-S3 SuperMini
// =============================================================================

// I2S Audio
#define I2S_WS          5
#define I2S_BCK         6
#define I2S_DIN         4
#define I2S_DOUT        7

// I2C Display
#define I2C_SDA         8
#define I2C_SCL         9

// Servos (for future use)
#define SERVO_PAN       1
#define SERVO_TILT      2

// LED & Touch Sensor
#define WS2812_DATA     48
#define TOUCH_PIN       10  // TTP223 touch sensor (active HIGH)

// =============================================================================
// I2S Audio Configuration
// =============================================================================

#define I2S_PORT_MIC    I2S_NUM_0
#define SAMPLE_RATE     16000
#define SAMPLE_BITS     16
#define I2S_BUFFER_SIZE 1024
#define AUDIO_BUFFER_SIZE 4096

// Audio packet header bytes for serial protocol
#define AUDIO_HEADER_1  0xAA
#define AUDIO_HEADER_2  0x55

// =============================================================================
// Audio State Machine
// =============================================================================

enum AudioState {
    AUDIO_IDLE = 0,
    AUDIO_LISTENING,
    AUDIO_STREAMING
};

AudioState audioState = AUDIO_IDLE;
int32_t rawAudioBuffer[AUDIO_BUFFER_SIZE];  // 32-bit buffer for INMP441
int16_t audioBuffer[AUDIO_BUFFER_SIZE];     // 16-bit buffer for output
unsigned long recordingStartTime = 0;
#define MAX_RECORDING_MS 10000  // 10 second max recording

// Silence detection
#define SILENCE_THRESHOLD   1000    // Audio level threshold
#define SILENCE_DURATION_MS 1500    // Stop after this much silence (ms)
#define MIN_SPEECH_MS       300     // Minimum speech before silence detection kicks in

// Software gain for microphone (after 32-bit to 16-bit conversion)
#define MIC_GAIN            4       // Small additional gain if needed

unsigned long lastSoundTime = 0;
bool speechDetected = false;

// =============================================================================
// Serial Command Buffer
// =============================================================================

#define SERIAL_BUFFER_SIZE 256
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// =============================================================================
// Display Configuration
// =============================================================================

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDRESS    0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =============================================================================
// LED Configuration
// =============================================================================

#define NUM_LEDS        1
#define LED_BRIGHTNESS  128

CRGB leds[NUM_LEDS];
uint8_t rainbowHue = 0;

// =============================================================================
// Button Configuration
// =============================================================================

volatile bool buttonPressed = false;
volatile unsigned long lastButtonPress = 0;
#define DEBOUNCE_MS     200  // Increased to handle release bounce

void IRAM_ATTR buttonISR() {
    unsigned long now = millis();
    if (now - lastButtonPress > DEBOUNCE_MS) {
        buttonPressed = true;
        lastButtonPress = now;
    }
}

// =============================================================================
// Emotion States
// =============================================================================

enum Emotion {
    EMOTION_IDLE = 0,
    EMOTION_LISTENING,
    EMOTION_THINKING,
    EMOTION_HAPPY,
    EMOTION_SAD,
    EMOTION_ANGRY,
    EMOTION_SURPRISED,
    EMOTION_CONFUSED,
    EMOTION_SLEEPY,
    EMOTION_EXCITED
};

Emotion currentEmotion = EMOTION_IDLE;

// =============================================================================
// Eye Configuration
// =============================================================================

#define EYE_WIDTH       40
#define EYE_HEIGHT      40
#define EYE_SPACING     48
#define PUPIL_SIZE      10
#define PUPIL_MAX_OFFSET 8

// Eye state
float pupilX = 0.0;
float pupilY = 0.0;
float targetPupilX = 0.0;
float targetPupilY = 0.0;

// Blink state
bool isBlinking = false;
int blinkFrame = 0;
#define BLINK_FRAMES    4

// Animation state
unsigned long lastLookChange = 0;
unsigned long lastBlink = 0;
unsigned long nextLookInterval = 2000;
unsigned long nextBlinkInterval = 3000;
unsigned long animationFrame = 0;

// =============================================================================
// Function Prototypes
// =============================================================================

void setupDisplay();
void setupLED();
void setupButton();
void setupI2SMic();
void updateEyes();
void drawEyes();
void updateLED();
void handleButton();
void updateIdleBehavior();
void handleSerialCommands();
void processCommand(const char* json);
void setEmotionByName(const char* name);
void sendStatus();
void sendEvent(const char* event, const char* action = nullptr);
void updateAudioCapture();
void startRecording();
void stopRecording();
void streamAudioPacket(int16_t* data, size_t samples);

// Eye drawing functions for each emotion
void drawIdleEyes();
void drawListeningEyes();
void drawThinkingEyes();
void drawHappyEyes();
void drawSadEyes();
void drawAngryEyes();
void drawSurprisedEyes();
void drawConfusedEyes();
void drawSleepyEyes();
void drawExcitedEyes();

// Helper functions
void drawRoundEye(int cx, int cy, int w, int h, int pupilOffX, int pupilOffY, int cornerRadius);
void drawPupil(int cx, int cy, int eyeH, int offX, int offY);

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("=================================");
    Serial.println("  Hinze Robot Companion v0.5");
    Serial.println("  ESP32-S3 SuperMini");
    Serial.println("  I2S Mic + Gain Boost");
    Serial.println("=================================");

    Wire.begin(I2C_SDA, I2C_SCL);

    setupDisplay();
    setupLED();
    setupButton();
    setupI2SMic();

    Serial.println("[INIT] Setup complete!");
    sendEvent("ready");
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
    handleSerialCommands();
    handleButton();
    updateAudioCapture();

    if (currentEmotion == EMOTION_IDLE && audioState == AUDIO_IDLE) {
        updateIdleBehavior();
    }

    updateEyes();
    drawEyes();
    updateLED();

    animationFrame++;
    delay(33);  // ~30 FPS
}

// =============================================================================
// Display Setup
// =============================================================================

void setupDisplay() {
    Serial.println("[INIT] Setting up OLED...");

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("[OLED] ERROR: Display not found!");
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(30, 28);
    display.print("Hinze v0.5");
    display.display();
    delay(1000);

    Serial.println("[OLED] Display initialized");
}

// =============================================================================
// Idle Behavior (look around + blink)
// =============================================================================

void updateIdleBehavior() {
    unsigned long now = millis();

    if (now - lastLookChange > nextLookInterval) {
        targetPupilX = random(-100, 101) / 100.0;
        targetPupilY = random(-100, 101) / 100.0;
        lastLookChange = now;
        nextLookInterval = random(1500, 4000);
    }

    if (!isBlinking && now - lastBlink > nextBlinkInterval) {
        isBlinking = true;
        blinkFrame = 0;
        lastBlink = now;
        nextBlinkInterval = random(2000, 6000);
    }
}

void updateEyes() {
    // Smooth pupil movement
    float speed = 0.15;
    pupilX += (targetPupilX - pupilX) * speed;
    pupilY += (targetPupilY - pupilY) * speed;

    // Blink animation
    if (isBlinking) {
        blinkFrame++;
        if (blinkFrame >= BLINK_FRAMES * 2) {
            isBlinking = false;
            blinkFrame = 0;
        }
    }
}

// =============================================================================
// Main Eye Drawing - Routes to emotion-specific functions
// =============================================================================

void drawEyes() {
    display.clearDisplay();

    switch (currentEmotion) {
        case EMOTION_IDLE:       drawIdleEyes(); break;
        case EMOTION_LISTENING:  drawListeningEyes(); break;
        case EMOTION_THINKING:   drawThinkingEyes(); break;
        case EMOTION_HAPPY:      drawHappyEyes(); break;
        case EMOTION_SAD:        drawSadEyes(); break;
        case EMOTION_ANGRY:      drawAngryEyes(); break;
        case EMOTION_SURPRISED:  drawSurprisedEyes(); break;
        case EMOTION_CONFUSED:   drawConfusedEyes(); break;
        case EMOTION_SLEEPY:     drawSleepyEyes(); break;
        case EMOTION_EXCITED:    drawExcitedEyes(); break;
    }

    display.display();
}

// =============================================================================
// Helper Functions
// =============================================================================

void drawRoundEye(int cx, int cy, int w, int h, int pupilOffX, int pupilOffY, int cornerRadius) {
    int x = cx - w / 2;
    int y = cy - h / 2;
    display.fillRoundRect(x, y, w, h, cornerRadius, SSD1306_WHITE);

    // Draw pupil
    if (h > 8) {
        int maxOff = (h / 2) - (PUPIL_SIZE / 2) - 2;
        if (maxOff < 0) maxOff = 0;
        int py = constrain(cy + pupilOffY, cy - maxOff, cy + maxOff);
        int px = cx + pupilOffX;
        display.fillCircle(px, py, PUPIL_SIZE / 2, SSD1306_BLACK);
    }
}

// =============================================================================
// Emotion: IDLE - Look around with natural blink
// =============================================================================

void drawIdleEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    int blinkState = 0;
    if (isBlinking) {
        blinkState = (blinkFrame < BLINK_FRAMES) ? blinkFrame : (BLINK_FRAMES * 2 - blinkFrame - 1);
    }

    int h = EYE_HEIGHT - (blinkState * EYE_HEIGHT / (BLINK_FRAMES + 1));
    if (h < 4) h = 4;

    int pupilOffX = (int)(pupilX * PUPIL_MAX_OFFSET);
    int pupilOffY = (int)(pupilY * PUPIL_MAX_OFFSET);

    drawRoundEye(leftX, eyeY, EYE_WIDTH, h, pupilOffX, pupilOffY, 8);
    drawRoundEye(rightX, eyeY, EYE_WIDTH, h, pupilOffX, pupilOffY, 8);
}

// =============================================================================
// Emotion: LISTENING - Wide open, alert eyes
// =============================================================================

void drawListeningEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    // Slightly larger, very round eyes
    int w = 44;
    int h = 44;

    // Subtle size pulse
    int pulse = (animationFrame % 30 < 15) ? 2 : 0;
    h += pulse;
    w += pulse;

    drawRoundEye(leftX, eyeY, w, h, 0, -2, 10);
    drawRoundEye(rightX, eyeY, w, h, 0, -2, 10);
}

// =============================================================================
// Emotion: THINKING - Eyes look up-left, slightly squinted
// =============================================================================

void drawThinkingEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    // Squinted eyes
    int w = 38;
    int h = 28;

    // Looking up-left
    int pupilOffX = -6;
    int pupilOffY = -4;

    drawRoundEye(leftX, eyeY, w, h, pupilOffX, pupilOffY, 6);
    drawRoundEye(rightX, eyeY, w, h, pupilOffX, pupilOffY, 6);

    // Draw thinking dots
    int dotY = 50;
    int numDots = ((animationFrame / 15) % 4);
    for (int i = 0; i < numDots && i < 3; i++) {
        display.fillCircle(52 + i * 12, dotY, 3, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: HAPPY - Curved ^ ^ anime-style happy eyes
// =============================================================================

void drawHappyEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    // Draw happy curved eyes (like ^ ^)
    // Left eye - arc curving up
    for (int i = -18; i <= 18; i++) {
        int y = eyeY - 8 + abs(i) / 2;
        display.fillRect(leftX + i - 2, y, 5, 8, SSD1306_WHITE);
    }

    // Right eye - arc curving up
    for (int i = -18; i <= 18; i++) {
        int y = eyeY - 8 + abs(i) / 2;
        display.fillRect(rightX + i - 2, y, 5, 8, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: SAD - Droopy eyes angled down
// =============================================================================

void drawSadEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Droopy eyes
    int w = 36;
    int h = 30;

    // Draw eyes
    drawRoundEye(leftX, eyeY, w, h, 0, 4, 6);
    drawRoundEye(rightX, eyeY, w, h, 0, 4, 6);

    // Sad eyebrows (angled down toward center)
    // Left eyebrow
    display.drawLine(leftX - 18, eyeY - 22, leftX + 12, eyeY - 18, SSD1306_WHITE);
    display.drawLine(leftX - 18, eyeY - 21, leftX + 12, eyeY - 17, SSD1306_WHITE);

    // Right eyebrow
    display.drawLine(rightX - 12, eyeY - 18, rightX + 18, eyeY - 22, SSD1306_WHITE);
    display.drawLine(rightX - 12, eyeY - 17, rightX + 18, eyeY - 21, SSD1306_WHITE);

    // Tear drop on left eye
    if ((animationFrame / 20) % 2 == 0) {
        int tearY = eyeY + 20 + (animationFrame % 20) / 2;
        display.fillCircle(leftX + 10, tearY, 3, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: ANGRY - Narrow eyes with angled eyebrows
// =============================================================================

void drawAngryEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Narrow, intense eyes
    int w = 42;
    int h = 20;

    drawRoundEye(leftX, eyeY, w, h, 0, 0, 4);
    drawRoundEye(rightX, eyeY, w, h, 0, 0, 4);

    // Angry eyebrows (angled down toward center, thick)
    // Left eyebrow - angled down to right
    for (int t = 0; t < 4; t++) {
        display.drawLine(leftX - 20, eyeY - 16 + t, leftX + 15, eyeY - 24 + t, SSD1306_WHITE);
    }

    // Right eyebrow - angled down to left
    for (int t = 0; t < 4; t++) {
        display.drawLine(rightX - 15, eyeY - 24 + t, rightX + 20, eyeY - 16 + t, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: SURPRISED - Very wide, round eyes
// =============================================================================

void drawSurprisedEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Very large round eyes
    int w = 46;
    int h = 46;

    // Small pupils for surprised look
    display.fillRoundRect(leftX - w/2, eyeY - h/2, w, h, 12, SSD1306_WHITE);
    display.fillRoundRect(rightX - w/2, eyeY - h/2, w, h, 12, SSD1306_WHITE);

    // Small pupils
    display.fillCircle(leftX, eyeY - 2, 4, SSD1306_BLACK);
    display.fillCircle(rightX, eyeY - 2, 4, SSD1306_BLACK);

    // Raised eyebrows
    display.fillRoundRect(leftX - 18, eyeY - 32, 36, 4, 2, SSD1306_WHITE);
    display.fillRoundRect(rightX - 18, eyeY - 32, 36, 4, 2, SSD1306_WHITE);
}

// =============================================================================
// Emotion: CONFUSED - Asymmetric eyes, one raised
// =============================================================================

void drawConfusedEyes() {
    int leftX = 40;
    int rightX = 88;

    // Left eye - normal but looking to side
    drawRoundEye(leftX, 34, 38, 36, 6, 0, 8);

    // Right eye - raised and smaller (questioning)
    drawRoundEye(rightX, 28, 34, 32, -4, -2, 8);

    // One raised eyebrow on right
    display.fillRoundRect(rightX - 16, 8, 32, 4, 2, SSD1306_WHITE);

    // Question mark
    display.setTextSize(2);
    display.setCursor(108, 8);
    display.print("?");
}

// =============================================================================
// Emotion: SLEEPY - Half-closed eyes
// =============================================================================

void drawSleepyEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Half-closed droopy eyes
    int w = 40;
    int h = 14;  // Very squished

    // Slight droop animation
    int droop = (animationFrame / 40) % 2 == 0 ? 0 : 2;
    h -= droop;
    if (h < 8) h = 8;

    drawRoundEye(leftX, eyeY + 2, w, h, 0, 0, 4);
    drawRoundEye(rightX, eyeY + 2, w, h, 0, 0, 4);

    // Heavy eyelids (top of eye darker/covered)
    display.fillRect(leftX - 22, eyeY - 12, 44, 12, SSD1306_BLACK);
    display.fillRect(rightX - 22, eyeY - 12, 44, 12, SSD1306_BLACK);

    // Z's floating
    display.setTextSize(1);
    int zOffset = (animationFrame / 10) % 10;
    display.setCursor(100, 10 - zOffset/2);
    display.print("Z");
    display.setCursor(108, 5 - zOffset/2);
    display.print("z");
    display.setCursor(114, 0);
    display.print("z");
}

// =============================================================================
// Emotion: EXCITED - Sparkly bouncy eyes
// =============================================================================

void drawExcitedEyes() {
    int leftX = 40;
    int rightX = 88;

    // Bouncy vertical offset
    int bounce = sin(animationFrame * 0.3) * 4;
    int eyeY = 32 + bounce;

    // Bright wide eyes
    int w = 42;
    int h = 42;

    drawRoundEye(leftX, eyeY, w, h, 0, -2, 10);
    drawRoundEye(rightX, eyeY, w, h, 0, -2, 10);

    // Sparkle effects around eyes
    int sparklePhase = (animationFrame / 5) % 4;

    // Sparkles at different positions based on phase
    if (sparklePhase == 0) {
        display.drawPixel(leftX - 24, eyeY - 20, SSD1306_WHITE);
        display.drawPixel(rightX + 24, eyeY - 18, SSD1306_WHITE);
    } else if (sparklePhase == 1) {
        display.drawLine(leftX + 20, eyeY - 24, leftX + 24, eyeY - 28, SSD1306_WHITE);
        display.drawLine(rightX - 20, eyeY - 24, rightX - 24, eyeY - 28, SSD1306_WHITE);
    } else if (sparklePhase == 2) {
        display.fillCircle(leftX - 26, eyeY, 2, SSD1306_WHITE);
        display.fillCircle(rightX + 26, eyeY, 2, SSD1306_WHITE);
    } else {
        display.drawLine(leftX - 22, eyeY + 18, leftX - 18, eyeY + 22, SSD1306_WHITE);
        display.drawLine(rightX + 22, eyeY + 18, rightX + 18, eyeY + 22, SSD1306_WHITE);
    }

    // Star sparkle in eyes
    display.drawPixel(leftX + 6, eyeY - 8, SSD1306_WHITE);
    display.drawPixel(rightX + 6, eyeY - 8, SSD1306_WHITE);
}

// =============================================================================
// LED Setup & Update
// =============================================================================

void setupLED() {
    Serial.println("[INIT] Setting up LED...");
    FastLED.addLeds<WS2812, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.printf("[LED] WS2812 on GPIO %d\n", WS2812_DATA);
}

void updateLED() {
    switch (currentEmotion) {
        case EMOTION_IDLE:
            leds[0] = CHSV(rainbowHue++, 255, 255);
            break;
        case EMOTION_LISTENING:
            leds[0] = CHSV(160, 255, beatsin8(60, 100, 255));
            break;
        case EMOTION_THINKING:
            leds[0] = CHSV(43, 255, beatsin8(90, 100, 255));
            break;
        case EMOTION_HAPPY:
            leds[0] = CHSV(96, 255, 255);
            break;
        case EMOTION_SAD:
            leds[0] = CHSV(160, 255, 180);
            break;
        case EMOTION_ANGRY:
            leds[0] = CHSV(0, 255, 255);
            break;
        case EMOTION_SURPRISED:
            leds[0] = CRGB::White;
            break;
        case EMOTION_CONFUSED:
            leds[0] = CHSV(192, 255, 255);
            break;
        case EMOTION_SLEEPY:
            leds[0] = CHSV(32, 255, 80);
            break;
        case EMOTION_EXCITED:
            leds[0] = CHSV(rainbowHue, 255, 255);
            rainbowHue += 5;
            break;
    }
    FastLED.show();
}

// =============================================================================
// Button Handling
// =============================================================================

void setupButton() {
    Serial.println("[INIT] Setting up touch sensor...");
    pinMode(TOUCH_PIN, INPUT);  // TTP223 has its own output driver
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), buttonISR, RISING);  // Active HIGH
    Serial.printf("[TOUCH] TTP223 on GPIO %d (interrupt enabled)\n", TOUCH_PIN);
}

void handleButton() {
    // Push-to-talk: press to start, auto-stop on silence or timeout

    if (buttonPressed) {
        buttonPressed = false;

        if (audioState == AUDIO_IDLE) {
            sendEvent("button", "press");
            startRecording();

            // Reset animation states
            targetPupilX = 0;
            targetPupilY = 0;
            animationFrame = 0;
        } else if (audioState == AUDIO_STREAMING) {
            // Only allow manual stop after minimum recording time (prevent bounce)
            if (millis() - recordingStartTime > 500) {
                sendEvent("button", "press");
                stopRecording();
            }
        }
    }
}

// =============================================================================
// I2S Microphone Setup
// =============================================================================

void setupI2SMic() {
    Serial.println("[INIT] Setting up I2S microphone...");

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // INMP441 outputs 24-bit in 32-bit frame
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // L/R pin to GND = LEFT channel
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN
    };

    esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: Driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT_MIC, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: Pin config failed: %d\n", err);
        return;
    }

    Serial.printf("[I2S] Microphone configured: %dHz, %d-bit\n", SAMPLE_RATE, SAMPLE_BITS);
    Serial.printf("[I2S] Pins: WS=%d, BCK=%d, DIN=%d\n", I2S_WS, I2S_BCK, I2S_DIN);
}

// =============================================================================
// Audio Capture Functions
// =============================================================================

void startRecording() {
    Serial.println("[AUDIO] Starting recording...");
    audioState = AUDIO_LISTENING;
    currentEmotion = EMOTION_LISTENING;
    recordingStartTime = millis();
    lastSoundTime = millis();
    speechDetected = false;
    sendEvent("audio_start");

    // Small delay then start streaming
    delay(50);
    audioState = AUDIO_STREAMING;
}

void stopRecording() {
    Serial.println("[AUDIO] Stopping recording...");
    audioState = AUDIO_IDLE;
    currentEmotion = EMOTION_THINKING;
    sendEvent("audio_end");
}

// Calculate average audio level (simple absolute mean)
uint16_t calculateAudioLevel(int16_t* samples, size_t count) {
    if (count == 0) return 0;

    uint32_t sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += abs(samples[i]);
    }
    return sum / count;
}

void updateAudioCapture() {
    if (audioState != AUDIO_STREAMING) {
        return;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - recordingStartTime;

    // Check for timeout
    if (elapsed > MAX_RECORDING_MS) {
        Serial.println("[AUDIO] Recording timeout");
        stopRecording();
        return;
    }

    // Read 32-bit audio data from I2S (INMP441 outputs 24-bit in 32-bit frame)
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT_MIC, rawAudioBuffer,
                             AUDIO_BUFFER_SIZE * sizeof(int32_t),
                             &bytesRead, 10 / portTICK_PERIOD_MS);

    if (err == ESP_OK && bytesRead > 0) {
        size_t samplesRead = bytesRead / sizeof(int32_t);

        // Convert 32-bit to 16-bit (INMP441 data is in upper 24 bits, left-justified)
        // Shift right by 14 to get 18 bits, then take lower 16 bits
        for (size_t i = 0; i < samplesRead; i++) {
            int32_t sample = rawAudioBuffer[i] >> 14;  // Extract upper bits
            // Apply small gain if needed
            sample = sample * MIC_GAIN;
            // Clamp to 16-bit range
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            audioBuffer[i] = (int16_t)sample;
        }

        // Calculate audio level for silence detection
        uint16_t level = calculateAudioLevel(audioBuffer, samplesRead);

        // Debug: print audio level every ~500ms
        static unsigned long lastLevelPrint = 0;
        if (now - lastLevelPrint > 500) {
            Serial.printf("[MIC] Level: %5d (threshold: %d)\n", level, SILENCE_THRESHOLD);
            lastLevelPrint = now;
        }

        // Check if sound detected
        if (level > SILENCE_THRESHOLD) {
            lastSoundTime = now;
            if (!speechDetected && elapsed > MIN_SPEECH_MS) {
                speechDetected = true;
                Serial.println("[AUDIO] Speech detected");
            }
        }

        // Check for silence (only after minimum speech time)
        if (speechDetected && (now - lastSoundTime > SILENCE_DURATION_MS)) {
            Serial.println("[AUDIO] Silence detected - stopping");
            stopRecording();
            return;
        }

        // Stream the audio
        streamAudioPacket(audioBuffer, samplesRead);
    }
}

void streamAudioPacket(int16_t* data, size_t samples) {
    // Send audio packet with header: [0xAA][0x55][len_high][len_low][pcm_data...]
    size_t dataBytes = samples * sizeof(int16_t);

    Serial.write(AUDIO_HEADER_1);
    Serial.write(AUDIO_HEADER_2);
    Serial.write((uint8_t)(dataBytes >> 8));
    Serial.write((uint8_t)(dataBytes & 0xFF));
    Serial.write((uint8_t*)data, dataBytes);
}

// =============================================================================
// Serial Command Handling
// =============================================================================

void handleSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialBufferIndex > 0) {
                serialBuffer[serialBufferIndex] = '\0';
                processCommand(serialBuffer);
                serialBufferIndex = 0;
            }
        } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
            serialBuffer[serialBufferIndex++] = c;
        }
    }
}

void processCommand(const char* json) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        Serial.printf("{\"error\":\"JSON parse failed: %s\"}\n", error.c_str());
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
        Serial.println("{\"error\":\"Missing cmd field\"}");
        return;
    }

    if (strcmp(cmd, "emotion") == 0) {
        const char* state = doc["state"];
        if (state) {
            setEmotionByName(state);
            Serial.printf("{\"ok\":true,\"emotion\":\"%s\"}\n", state);
        } else {
            Serial.println("{\"error\":\"Missing state field\"}");
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        sendStatus();
    }
    else if (strcmp(cmd, "record_start") == 0) {
        if (audioState == AUDIO_IDLE) {
            startRecording();
            Serial.println("{\"ok\":true,\"action\":\"recording\"}");
        } else {
            Serial.println("{\"error\":\"Already recording\"}");
        }
    }
    else if (strcmp(cmd, "record_stop") == 0) {
        if (audioState == AUDIO_STREAMING) {
            stopRecording();
            Serial.println("{\"ok\":true,\"action\":\"stopped\"}");
        } else {
            Serial.println("{\"error\":\"Not recording\"}");
        }
    }
    else {
        Serial.printf("{\"error\":\"Unknown command: %s\"}\n", cmd);
    }
}

void setEmotionByName(const char* name) {
    if (strcmp(name, "idle") == 0) currentEmotion = EMOTION_IDLE;
    else if (strcmp(name, "listening") == 0) currentEmotion = EMOTION_LISTENING;
    else if (strcmp(name, "thinking") == 0) currentEmotion = EMOTION_THINKING;
    else if (strcmp(name, "happy") == 0) currentEmotion = EMOTION_HAPPY;
    else if (strcmp(name, "sad") == 0) currentEmotion = EMOTION_SAD;
    else if (strcmp(name, "angry") == 0) currentEmotion = EMOTION_ANGRY;
    else if (strcmp(name, "surprised") == 0) currentEmotion = EMOTION_SURPRISED;
    else if (strcmp(name, "confused") == 0) currentEmotion = EMOTION_CONFUSED;
    else if (strcmp(name, "sleepy") == 0) currentEmotion = EMOTION_SLEEPY;
    else if (strcmp(name, "excited") == 0) currentEmotion = EMOTION_EXCITED;

    // Reset animation states
    targetPupilX = 0;
    targetPupilY = 0;
    animationFrame = 0;
}

void sendStatus() {
    const char* emotionNames[] = {
        "idle", "listening", "thinking", "happy", "sad",
        "angry", "surprised", "confused", "sleepy", "excited"
    };
    const char* audioStateNames[] = {"idle", "listening", "streaming"};

    Serial.printf("{\"status\":\"ok\",\"version\":\"0.5\",\"emotion\":\"%s\",\"audio\":\"%s\"}\n",
                  emotionNames[currentEmotion],
                  audioStateNames[audioState]);
}

void sendEvent(const char* event, const char* action) {
    if (action) {
        Serial.printf("{\"event\":\"%s\",\"action\":\"%s\"}\n", event, action);
    } else {
        Serial.printf("{\"event\":\"%s\"}\n", event);
    }
}
